
#include <plan_manage/planner_manager.h>
#include <limits>
#include <exploration_manager/fast_exploration_manager.h>
#include <traj_utils/planning_visualization.h>

#include <exploration_manager/fast_exploration_fsm.h>
#include <exploration_manager/expl_data.h>
#include <plan_env/edt_environment.h>
#include <plan_env/sdf_map.h>

using Eigen::Vector4d;

namespace fast_planner {
void FastExplorationFSM::init(ros::NodeHandle& nh) {
  fp_.reset(new FSMParam);
  fd_.reset(new FSMData);

  nh.param<std::string>("world_frame", world_frame_, "world");
  nh.param("odom_twist_in_body", odom_twist_in_body_, false);
  /*  Fsm param  */
  nh.param("fsm/thresh_replan1", fp_->replan_thresh1_, -1.0);
  nh.param("fsm/thresh_replan2", fp_->replan_thresh2_, -1.0);
  nh.param("fsm/thresh_replan3", fp_->replan_thresh3_, -1.0);
  nh.param("fsm/replan_time", fp_->replan_time_, -1.0);
  nh.param("exploration/finish_confirm_cycles", finish_confirm_cycles_, 3);
  nh.param("exploration/retry_interval", retry_interval_, 1.0);
  nh.param("trajectory_handover/timeout", handover_timeout_, 2.0);
  nh.param("return_home/retry_interval", return_retry_interval_, 1.0);
  nh.param("return_home/urgent_retry_interval", return_urgent_retry_interval_, 0.1);
  nh.param("return_home/handover_margin", handover_margin_, 0.1);
  nh.param("return_home/replan_period", return_replan_period_, 1.0);
  nh.param<string>("source_id", source_id_, "fuel_planner");
  nh.param("return_home/position_tolerance", home_tolerance_, 0.20);
  nh.param("return_home/speed_tolerance", home_speed_tolerance_, 0.15);
  vector<double> home;
  if (nh.getParam("home_position", home) && home.size() == 3)
    return_goal_ << home[0], home[1], home[2];
  if (finish_confirm_cycles_ <= 0 || handover_timeout_ <= 0.0 ||
      retry_interval_ <= 0.0 || return_retry_interval_ <= 0.0 ||
      return_urgent_retry_interval_ <= 0.0 || handover_margin_ < 0.0 ||
      !std::isfinite(return_urgent_retry_interval_) || !std::isfinite(handover_margin_) ||
      return_replan_period_ <= 0.0 || home_tolerance_ <= 0.0 ||
      home_speed_tolerance_ <= 0.0 || !return_goal_.allFinite() || source_id_.empty())
    throw std::invalid_argument("invalid mission planning parameters");

  /* Initialize main modules */
  expl_manager_.reset(new FastExplorationManager);
  expl_manager_->initialize(nh);
  visualization_.reset(new PlanningVisualization(nh));

  planner_manager_ = expl_manager_->planner_manager_;
  state_ = EXPL_STATE::INIT;
  fd_->have_odom_ = false;
  fd_->state_str_ = { "INIT", "WAIT_TRIGGER", "PLAN_TRAJ", "PUB_TRAJ", "EXEC_TRAJ", "FINISH", "WAIT_EXEC" };
  fd_->static_state_ = true;
  fd_->trigger_ = false;

  /* Ros sub, pub and timer */
  exec_timer_ = nh.createTimer(ros::Duration(0.01), &FastExplorationFSM::FSMCallback, this);
  safety_timer_ = nh.createTimer(ros::Duration(0.05), &FastExplorationFSM::safetyCallback, this);
  frontier_timer_ = nh.createTimer(ros::Duration(0.5), &FastExplorationFSM::frontierCallback, this);

  stop_sub_ = nh.subscribe<std_msgs::Empty>("/fuel/stop_planner", 1,
      [this](const std_msgs::Empty::ConstPtr&) {
        if (context_.manager_epoch != 0) {
          closed_context_ = context_;
          have_closed_context_ = true;
        }
        fd_->static_state_ = true;
        fd_->trigger_ = false;
        transitState(WAIT_TRIGGER, "stop");
      });
  trigger_sub_ =
      nh.subscribe("/waypoint_generator/waypoints", 1, &FastExplorationFSM::triggerCallback, this);
  mission_sub_ = nh.subscribe("/fuel/mission_command", 10,
      &FastExplorationFSM::missionCallback, this);
  authorization_sub_ = nh.subscribe("/fuel/authorized_context", 10,
      &FastExplorationFSM::authorizationCallback, this);
  trajectory_sub_ = nh.subscribe("/fuel/trajectory_status", 20,
      &FastExplorationFSM::trajectoryCallback, this);
  odom_sub_ = nh.subscribe("/odom_world", 1, &FastExplorationFSM::odometryCallback, this);

  replan_pub_ = nh.advertise<std_msgs::Empty>("/planning/replan", 10);
  new_pub_ = nh.advertise<std_msgs::Empty>("/planning/new", 10);
  bspline_pub_ = nh.advertise<bspline::Bspline>("/planning/bspline", 10);
  planner_status_pub_ = nh.advertise<uav_mission_msgs::PlannerStatus>("/fuel/planner_status", 10);
}

void FastExplorationFSM::FSMCallback(const ros::TimerEvent& e) {
  ROS_DEBUG_STREAM_THROTTLE(1.0, "[FSM]: state: " << fd_->state_str_[int(state_)]);

  switch (state_) {
    case INIT: {
      // Wait for odometry ready
      if (!fd_->have_odom_) {
        ROS_WARN_THROTTLE(1.0, "no odom.");
        return;
      }
      // Go to wait trigger when odom is ok
      transitState(WAIT_TRIGGER, "FSM");
      break;
    }

    case WAIT_TRIGGER: {
      // Do nothing but wait for trigger
      ROS_DEBUG_THROTTLE(1.0, "wait for trigger.");
      break;
    }

    case FINISH: {
      ROS_DEBUG_THROTTLE(1.0, "finish exploration.");
      break;
    }

    case PLAN_TRAJ: {
      const ros::Time now = ros::Time::now();
      if (!next_plan_attempt_.isZero() && now < next_plan_attempt_) break;
      if (!preparePlanningStart(now)) break;

      publishPlannerStatus(uav_mission_msgs::PlannerStatus::PLANNING,
                           return_home_ ? "planning return segment" : "planning exploration segment");
      const int res = callExplorationPlanner();
      if (res == SUCCEED) {
        if (!return_home_) plan_failures_ = 0;
        no_frontier_cycles_ = 0;
        transitState(PUB_TRAJ, "FSM");
      } else if (res == NO_FRONTIER && !return_home_) {
        ++no_frontier_cycles_;
        next_plan_attempt_ = now + ros::Duration(retry_interval_);
        if (no_frontier_cycles_ >= finish_confirm_cycles_) {
          publishPlannerStatus(uav_mission_msgs::PlannerStatus::EXPLORATION_DONE,
                               "no usable frontier for configured consecutive map updates");
          transitState(FINISH, "exploration complete");
          clearVisMarker();
        }
      } else {
        no_frontier_cycles_ = 0;
        if (plan_failures_ < std::numeric_limits<int>::max()) ++plan_failures_;
        publishPlannerStatus(uav_mission_msgs::PlannerStatus::FAILED,
                             return_home_ ? "return planning failed" : "exploration planning failed");
        // Failure counts never end a task. The manager still owns battery,
        // completion and timeout returns; planning failures keep retrying.
        scheduleRetry(ros::Time::now(), "planning failed");
      }
      break;
    }

    case PUB_TRAJ: {
        submitted_traj_.reset(new LocalTrajData(planner_manager_->local_data_));
        submitted_at_ = ros::Time::now();
        bspline_pub_.publish(fd_->newest_traj_);
        publishPlannerStatus(uav_mission_msgs::PlannerStatus::TRAJECTORY_READY,
                             "validated B-spline submitted for handover");
        transitState(WAIT_EXEC, "waiting for actual trajectory activation");

        thread vis_thread(&FastExplorationFSM::visualize, this);
        vis_thread.detach();
      break;
    }

    case WAIT_EXEC: {
      const ros::Time now = ros::Time::now();
      const bool switch_expired = return_home_ && submitted_traj_ &&
          now > submitted_traj_->start_time_ + ros::Duration(handover_margin_);
      if (switch_expired || (now - submitted_at_).toSec() > handover_timeout_ + 0.5) {
        submitted_traj_.reset();
        if (!return_home_) fd_->static_state_ = true;
        scheduleRetry(now, switch_expired ? "scheduled handover expired" : "activation timeout");
        transitState(PLAN_TRAJ, "activation timeout; retrying task");
      }
      break;
    }

    case EXEC_TRAJ: {
      const ros::Time now = ros::Time::now();
      if (last_execution_status_.isZero() || now < last_execution_status_ ||
          (now - last_execution_status_).toSec() >= 0.5) {
        const bool settled = return_home_ && executing_traj_ && executing_traj_->terminal_return_ &&
            !active_trajectory_unsafe_ && (now - executing_traj_->start_time_).toSec() >= executing_traj_->duration_;
        publishPlannerStatus(settled ? uav_mission_msgs::PlannerStatus::GOAL_REACHED
                                     : (return_home_ && executing_traj_ && executing_traj_->terminal_return_
                                        ? uav_mission_msgs::PlannerStatus::SETTLING
                                        : uav_mission_msgs::PlannerStatus::EXECUTING),
                             settled ? "terminal reference settled; verifying actual arrival" :
                             (return_home_ ? "executing return trajectory" : "executing exploration trajectory"));
        last_execution_status_ = now;
      }
      LocalTrajData* info = executing_traj_.get();
      if (!info) {
        fd_->static_state_ = true;
        transitState(PLAN_TRAJ, "no executed trajectory");
        break;
      }
      double t_cur = (now - info->start_time_).toSec();

      if (return_home_ && info->terminal_return_ && !active_trajectory_unsafe_) {
        // Let a safe terminal trajectory settle. Collision checking continues
        // in safetyCallback; large tracking errors request measured-state recovery.
        const double t = std::max(0.0, std::min(t_cur, info->duration_));
        const double tracking_error = (fd_->odom_pos_ - info->position_traj_.evaluateDeBoorT(t)).norm();
        const bool convergence_timeout = t_cur > info->duration_ + 3.0 &&
            ((fd_->odom_pos_ - return_goal_).norm() > home_tolerance_ ||
             fd_->odom_vel_.norm() > home_speed_tolerance_ || std::abs(fd_->odom_yaw_) > 0.08);
        if (tracking_error > 0.6) {
          if (terminal_error_since_.isZero()) terminal_error_since_ = now;
        } else terminal_error_since_ = ros::Time();
        const bool tracking_failure = !terminal_error_since_.isZero() &&
            (now - terminal_error_since_).toSec() >= 0.5;
        if (tracking_failure || convergence_timeout) {
          // Recovery starts from measured state: brake first so a discontinuous
          // replacement cannot jump directly from the old reference to odometry.
          active_trajectory_unsafe_ = true;
          fd_->static_state_ = true;
          publishPlannerStatus(uav_mission_msgs::PlannerStatus::TRAJECTORY_UNSAFE,
                               "terminal tracking recovery; brake and replan");
          transitState(PLAN_TRAJ, "terminal tracking recovery");
        }
        break;
      }
      // Replan if traj is almost fully executed
      double time_to_end = info->duration_ - t_cur;
      if (time_to_end < fp_->replan_thresh1_) {
        transitState(PLAN_TRAJ, "FSM");
        ROS_DEBUG("Replan: traj fully executed=================================");
        return;
      }
      // Replan if next frontier to be visited is covered
      if (!return_home_ && t_cur > fp_->replan_thresh2_ &&
          expl_manager_->frontier_finder_->isFrontierCovered()) {
        transitState(PLAN_TRAJ, "FSM");
        ROS_DEBUG("Replan: cluster covered=====================================");
        return;
      }
      // Replan after some time
      const double periodic_threshold = return_home_ ? return_replan_period_ : fp_->replan_thresh3_;
      if (t_cur > periodic_threshold && !classic_) {
        transitState(PLAN_TRAJ, "FSM");
        ROS_DEBUG("Replan: periodic call=======================================");
      }
      break;
    }
  }
}

bool FastExplorationFSM::hasSafeActiveTrajectory(const ros::Time& now) const {
  if (!executing_traj_ || active_trajectory_unsafe_) return false;
  const ros::Time end = executing_traj_->start_time_ + ros::Duration(executing_traj_->duration_);
  return now <= end;
}

double FastExplorationFSM::activeTrajectoryRemaining(const ros::Time& now) const {
  if (!executing_traj_) return 0.0;
  const ros::Time end = executing_traj_->start_time_ + ros::Duration(executing_traj_->duration_);
  return std::max(0.0, (end - now).toSec());
}

void FastExplorationFSM::enterBrakingRecovery(const string& reason) {
  if (!active_trajectory_unsafe_) {
    active_trajectory_unsafe_ = true;
    fd_->static_state_ = true;
    submitted_traj_.reset();
    publishPlannerStatus(uav_mission_msgs::PlannerStatus::TRAJECTORY_UNSAFE, reason);
  }
  next_plan_attempt_ = ros::Time::now() + ros::Duration(return_urgent_retry_interval_);
}

void FastExplorationFSM::scheduleRetry(const ros::Time& now, const string& reason) {
  if (!return_home_) {
    // Preserve exploration's existing behavior: a failed rolling replan keeps
    // sampling the active trajectory unless no executable reference remains.
    if (active_trajectory_unsafe_ || !executing_traj_) fd_->static_state_ = true;
    next_plan_attempt_ = now + ros::Duration(retry_interval_);
    return;
  }

  if (hasSafeActiveTrajectory(now) && !fd_->static_state_) {
    const double remaining = activeTrajectoryRemaining(now);
    const double lead = fp_->replan_time_ + handover_margin_;
    const double available_wait = remaining - lead;
    if (available_wait <= return_urgent_retry_interval_) {
      ROS_WARN("Return retry cannot meet active trajectory deadline (remaining %.3f s): %s",
               remaining, reason.c_str());
      enterBrakingRecovery("return handover window exhausted; controlled braking recovery");
      return;
    }
    const bool urgent = remaining <= lead + return_retry_interval_;
    const double wait = urgent
        ? std::min(return_urgent_retry_interval_, available_wait)
        : std::min(return_retry_interval_, available_wait);
    fd_->static_state_ = false;
    next_plan_attempt_ = now + ros::Duration(std::max(0.0, wait));
    ROS_WARN("Return retry in %.3f s with %.3f s active trajectory remaining: %s",
             wait, remaining, reason.c_str());
    return;
  }

  fd_->static_state_ = true;
  next_plan_attempt_ = now + ros::Duration(return_urgent_retry_interval_);
}

bool FastExplorationFSM::preparePlanningStart(const ros::Time& now) {
  const double lead = fp_->replan_time_ + (return_home_ ? handover_margin_ : 0.0);
  planning_switch_time_ = now + ros::Duration(lead);
  if (!fd_->static_state_ && executing_traj_ && !active_trajectory_unsafe_ &&
      (!return_home_ || hasSafeActiveTrajectory(now))) {
    LocalTrajData* info = executing_traj_.get();
    const double switch_t = (planning_switch_time_ - info->start_time_).toSec();
    if (return_home_ && switch_t >= info->duration_) {
      enterBrakingRecovery("safe trajectory has no remaining handover window; controlled braking");
      return false;
    }
    const double t = std::max(0.0, std::min(info->duration_, switch_t));
    fd_->start_pt_ = info->position_traj_.evaluateDeBoorT(t);
    fd_->start_vel_ = info->velocity_traj_.evaluateDeBoorT(t);
    fd_->start_acc_ = info->acceleration_traj_.evaluateDeBoorT(t);
    fd_->start_yaw_(0) = info->yaw_traj_.evaluateDeBoorT(t)[0];
    fd_->start_yaw_(1) = info->yawdot_traj_.evaluateDeBoorT(t)[0];
    fd_->start_yaw_(2) = info->yawdotdot_traj_.evaluateDeBoorT(t)[0];
    if (t >= info->duration_) {
      // Exploration can replan from the endpoint hold after a delayed attempt.
      fd_->start_vel_.setZero();
      fd_->start_acc_.setZero();
      fd_->start_yaw_(1) = fd_->start_yaw_(2) = 0.0;
    }
  } else {
    fd_->start_pt_ = fd_->odom_pos_;
    fd_->start_vel_ = fd_->odom_vel_;
    fd_->start_acc_.setZero();
    fd_->start_yaw_(0) = fd_->odom_yaw_;
    fd_->start_yaw_(1) = fd_->start_yaw_(2) = 0.0;
  }
  return true;
}

int FastExplorationFSM::callExplorationPlanner() {

  // Failed optimizations must not overwrite the last accepted trajectory data.
  const LocalTrajData previous = planner_manager_->local_data_;
  const double segment_length = std::max(1.0, 5.0 / (1.0 + std::min(plan_failures_, 4)));
  int res = return_home_
      ? expl_manager_->planToGoal(fd_->start_pt_, fd_->start_vel_, fd_->start_acc_,
                                  fd_->start_yaw_, return_goal_, 0.0,
                                  segment_length, plan_failures_ > 0, true)
      : expl_manager_->planExploreMotion(fd_->start_pt_, fd_->start_vel_, fd_->start_acc_,
                                         fd_->start_yaw_);
  classic_ = false;

  // int res = expl_manager_->classicFrontier(fd_->start_pt_, fd_->start_yaw_[0]);
  // classic_ = true;

  // int res = expl_manager_->rapidFrontier(fd_->start_pt_, fd_->start_vel_, fd_->start_yaw_[0],
  // classic_);

  if (return_home_ && res == SUCCEED &&
      ros::Time::now() + ros::Duration(handover_margin_) >= planning_switch_time_) {
    ROS_WARN("Discard trajectory that missed its fixed handover time by %.3f s",
             (ros::Time::now() - planning_switch_time_).toSec());
    planner_manager_->local_data_ = previous;
    return FAIL;
  }

  if (res == SUCCEED) {
    auto info = &planner_manager_->local_data_;
    info->traj_id_ = ++next_trajectory_id_;
    // Exploration's look-ahead is a nominal planning budget, not a failure
    // deadline. Late successes still undergo executor continuity checks.
    info->start_time_ = return_home_ ? planning_switch_time_
                                   : std::max(planning_switch_time_, ros::Time::now());

    bspline::Bspline bspline;
    bspline.context = context_;
    bspline.order = planner_manager_->pp_.bspline_degree_;
    bspline.start_time = info->start_time_;
    bspline.traj_id = info->traj_id_;
    Eigen::MatrixXd pos_pts = info->position_traj_.getControlPoint();
    for (int i = 0; i < pos_pts.rows(); ++i) {
      geometry_msgs::Point pt;
      pt.x = pos_pts(i, 0);
      pt.y = pos_pts(i, 1);
      pt.z = pos_pts(i, 2);
      bspline.pos_pts.push_back(pt);
    }
    Eigen::VectorXd knots = info->position_traj_.getKnot();
    for (int i = 0; i < knots.rows(); ++i) {
      bspline.knots.push_back(knots(i));
    }
    Eigen::MatrixXd yaw_pts = info->yaw_traj_.getControlPoint();
    for (int i = 0; i < yaw_pts.rows(); ++i) {
      double yaw = yaw_pts(i, 0);
      bspline.yaw_pts.push_back(yaw);
    }
    bspline.yaw_dt = info->yaw_traj_.getKnotSpan();
    fd_->newest_traj_ = bspline;
  }
  if (res != SUCCEED) planner_manager_->local_data_ = previous;
  return res;
}

void FastExplorationFSM::visualize() {
  auto info = &planner_manager_->local_data_;
  auto plan_data = &planner_manager_->plan_data_;
  auto ed_ptr = expl_manager_->ed_;

  // Draw updated box
  // Vector3d bmin, bmax;
  // planner_manager_->edt_environment_->sdf_map_->getUpdatedBox(bmin, bmax);
  // visualization_->drawBox((bmin + bmax) / 2.0, bmax - bmin, Vector4d(0, 1, 0, 0.3), "updated_box", 0,
  // 4);

  // Draw frontier
  static int last_ftr_num = 0;
  for (int i = 0; i < ed_ptr->frontiers_.size(); ++i) {
    visualization_->drawCubes(ed_ptr->frontiers_[i], 0.1,
                              visualization_->getColor(double(i) / ed_ptr->frontiers_.size(), 0.4),
                              "frontier", i, 4);
    // visualization_->drawBox(ed_ptr->frontier_boxes_[i].first, ed_ptr->frontier_boxes_[i].second,
    //                         Vector4d(0.5, 0, 1, 0.3), "frontier_boxes", i, 4);
  }
  for (int i = ed_ptr->frontiers_.size(); i < last_ftr_num; ++i) {
    visualization_->drawCubes({}, 0.1, Vector4d(0, 0, 0, 1), "frontier", i, 4);
    // visualization_->drawBox(Vector3d(0, 0, 0), Vector3d(0, 0, 0), Vector4d(1, 0, 0, 0.3),
    // "frontier_boxes", i, 4);
  }
  last_ftr_num = ed_ptr->frontiers_.size();
  // for (int i = 0; i < ed_ptr->dead_frontiers_.size(); ++i)
  //   visualization_->drawCubes(ed_ptr->dead_frontiers_[i], 0.1, Vector4d(0, 0, 0, 0.5), "dead_frontier",
  //                             i, 4);
  // for (int i = ed_ptr->dead_frontiers_.size(); i < 5; ++i)
  //   visualization_->drawCubes({}, 0.1, Vector4d(0, 0, 0, 0.5), "dead_frontier", i, 4);

  // Draw global top viewpoints info
  // visualization_->drawSpheres(ed_ptr->points_, 0.2, Vector4d(0, 0.5, 0, 1), "points", 0, 6);
  // visualization_->drawLines(ed_ptr->global_tour_, 0.07, Vector4d(0, 0.5, 0, 1), "global_tour", 0, 6);
  // visualization_->drawLines(ed_ptr->points_, ed_ptr->views_, 0.05, Vector4d(0, 1, 0.5, 1), "view", 0, 6);
  // visualization_->drawLines(ed_ptr->points_, ed_ptr->averages_, 0.03, Vector4d(1, 0, 0, 1),
  // "point-average", 0, 6);

  // Draw local refined viewpoints info
  // visualization_->drawSpheres(ed_ptr->refined_points_, 0.2, Vector4d(0, 0, 1, 1), "refined_pts", 0, 6);
  // visualization_->drawLines(ed_ptr->refined_points_, ed_ptr->refined_views_, 0.05,
  //                           Vector4d(0.5, 0, 1, 1), "refined_view", 0, 6);
  // visualization_->drawLines(ed_ptr->refined_tour_, 0.07, Vector4d(0, 0, 1, 1), "refined_tour", 0, 6);
  // visualization_->drawLines(ed_ptr->refined_views1_, ed_ptr->refined_views2_, 0.04, Vector4d(0, 0, 0,
  // 1),
  //                           "refined_view", 0, 6);
  // visualization_->drawLines(ed_ptr->refined_points_, ed_ptr->unrefined_points_, 0.05, Vector4d(1, 1,
  // 0, 1),
  //                           "refine_pair", 0, 6);
  // for (int i = 0; i < ed_ptr->n_points_.size(); ++i)
  //   visualization_->drawSpheres(ed_ptr->n_points_[i], 0.1,
  //                               visualization_->getColor(double(ed_ptr->refined_ids_[i]) /
  //                               ed_ptr->frontiers_.size()),
  //                               "n_points", i, 6);
  // for (int i = ed_ptr->n_points_.size(); i < 15; ++i)
  //   visualization_->drawSpheres({}, 0.1, Vector4d(0, 0, 0, 1), "n_points", i, 6);

  // Draw trajectory
  // visualization_->drawSpheres({ ed_ptr->next_goal_ }, 0.3, Vector4d(0, 1, 1, 1), "next_goal", 0, 6);
  visualization_->drawBspline(info->position_traj_, 0.1, Vector4d(1.0, 0.0, 0.0, 1), false, 0.15,
                              Vector4d(1, 1, 0, 1));
  // visualization_->drawSpheres(plan_data->kino_path_, 0.1, Vector4d(1, 0, 1, 1), "kino_path", 0, 0);
  // visualization_->drawLines(ed_ptr->path_next_goal_, 0.05, Vector4d(0, 1, 1, 1), "next_goal", 1, 6);
}

void FastExplorationFSM::clearVisMarker() {
  // visualization_->drawSpheres({}, 0.2, Vector4d(0, 0.5, 0, 1), "points", 0, 6);
  // visualization_->drawLines({}, 0.07, Vector4d(0, 0.5, 0, 1), "global_tour", 0, 6);
  // visualization_->drawSpheres({}, 0.2, Vector4d(0, 0, 1, 1), "refined_pts", 0, 6);
  // visualization_->drawLines({}, {}, 0.05, Vector4d(0.5, 0, 1, 1), "refined_view", 0, 6);
  // visualization_->drawLines({}, 0.07, Vector4d(0, 0, 1, 1), "refined_tour", 0, 6);
  // visualization_->drawSpheres({}, 0.1, Vector4d(0, 0, 1, 1), "B-Spline", 0, 0);

  // visualization_->drawLines({}, {}, 0.03, Vector4d(1, 0, 0, 1), "current_pose", 0, 6);
}

void FastExplorationFSM::frontierCallback(const ros::TimerEvent& e) {
  static int delay = 0;
  if (++delay < 5) return;

  if (state_ == WAIT_TRIGGER || state_ == FINISH) {
    auto ft = expl_manager_->frontier_finder_;
    auto ed = expl_manager_->ed_;
    ft->searchFrontiers();
    ft->computeFrontiersToVisit();
    ft->updateFrontierCostMatrix();

    ft->getFrontiers(ed->frontiers_);
    ft->getFrontierBoxes(ed->frontier_boxes_);

    // Draw frontier and bounding box
    for (int i = 0; i < ed->frontiers_.size(); ++i) {
      visualization_->drawCubes(ed->frontiers_[i], 0.1,
                                visualization_->getColor(double(i) / ed->frontiers_.size(), 0.4),
                                "frontier", i, 4);
      // visualization_->drawBox(ed->frontier_boxes_[i].first, ed->frontier_boxes_[i].second,
      // Vector4d(0.5, 0, 1, 0.3),
      //                         "frontier_boxes", i, 4);
    }
    for (int i = ed->frontiers_.size(); i < 50; ++i) {
      visualization_->drawCubes({}, 0.1, Vector4d(0, 0, 0, 1), "frontier", i, 4);
      // visualization_->drawBox(Vector3d(0, 0, 0), Vector3d(0, 0, 0), Vector4d(1, 0, 0, 0.3),
      // "frontier_boxes", i, 4);
    }
  }

  // if (!fd_->static_state_)
  // {
  //   static double astar_time = 0.0;
  //   static int astar_num = 0;
  //   auto t1 = ros::Time::now();

  //   planner_manager_->path_finder_->reset();
  //   planner_manager_->path_finder_->setResolution(0.4);
  //   if (planner_manager_->path_finder_->search(fd_->odom_pos_, Vector3d(-5, 0, 1)))
  //   {
  //     auto path = planner_manager_->path_finder_->getPath();
  //     visualization_->drawLines(path, 0.05, Vector4d(1, 0, 0, 1), "astar", 0, 6);
  //     auto visit = planner_manager_->path_finder_->getVisited();
  //     visualization_->drawCubes(visit, 0.3, Vector4d(0, 0, 1, 0.4), "astar-visit", 0, 6);
  //   }
  //   astar_num += 1;
  //   astar_time = (ros::Time::now() - t1).toSec();
  //   ROS_WARN("Average astar time: %lf", astar_time);
  // }
}

bool FastExplorationFSM::validContext(
    const uav_mission_msgs::TaskContext& context) const {
  return context.manager_epoch != 0 && context.mission_id != 0 &&
         context.task_type != uav_mission_msgs::TaskContext::TASK_NONE &&
         context.source_id == source_id_ &&
         context.manager_epoch == authorization_.manager_epoch &&
         context.mission_id == authorization_.mission_id &&
         context.task_type == authorization_.task_type &&
         context.source_id == authorization_.source_id;
}

void FastExplorationFSM::publishPlannerStatus(uint8_t state, const string& detail) {
  uav_mission_msgs::PlannerStatus status;
  status.header.stamp = ros::Time::now();
  status.header.frame_id = world_frame_;
  status.context = context_;
  status.trajectory_id = state == uav_mission_msgs::PlannerStatus::TRAJECTORY_UNSAFE && executing_traj_
      ? executing_traj_->traj_id_ : (planner_manager_->local_data_.traj_id_ > 0
          ? planner_manager_->local_data_.traj_id_ : 0);
  status.state = state;
  status.consecutive_failures = plan_failures_;
  status.detail = detail;
  planner_status_pub_.publish(status);
}

void FastExplorationFSM::authorizationCallback(
    const uav_mission_msgs::TaskContextConstPtr& msg) {
  authorization_ = *msg;
  if (have_mission_command_)
    missionCallback(uav_mission_msgs::MissionCommandConstPtr(
        new uav_mission_msgs::MissionCommand(last_mission_command_)));
}

void FastExplorationFSM::missionCallback(
    const uav_mission_msgs::MissionCommandConstPtr& msg) {
  last_mission_command_ = *msg;
  have_mission_command_ = true;
  if (!validContext(msg->context)) return;
  const bool current = msg->context.manager_epoch == context_.manager_epoch &&
      msg->context.mission_id == context_.mission_id &&
      msg->context.task_type == context_.task_type &&
      msg->context.source_id == context_.source_id;
  if (msg->command == uav_mission_msgs::MissionCommand::STOP ||
      msg->command == uav_mission_msgs::MissionCommand::HOLD_HOME) {
    if (current) {
      closed_context_ = msg->context;
      have_closed_context_ = true;
      fd_->static_state_ = true;
      fd_->trigger_ = false;
      transitState(WAIT_TRIGGER, "mission stop/hold");
      publishPlannerStatus(uav_mission_msgs::PlannerStatus::IDLE, "planner stopped");
    }
    return;
  }
  if (msg->command == uav_mission_msgs::MissionCommand::REPLAN) {
    if (!current || !return_home_ || have_closed_context_) return;
    // Keep the batch and recovery history. A transport recovery does not
    // invalidate a still-safe active trajectory.
    if (state_ == EXEC_TRAJ && executing_traj_ && executing_traj_->terminal_return_ &&
        !active_trajectory_unsafe_ &&
        (ros::Time::now() - executing_traj_->start_time_).toSec() < executing_traj_->duration_ + 3.0)
      return;
    fd_->static_state_ = !hasSafeActiveTrajectory(ros::Time::now());
    if (state_ != PLAN_TRAJ && state_ != WAIT_EXEC && state_ != PUB_TRAJ) {
      next_plan_attempt_ = ros::Time::now();
      transitState(PLAN_TRAJ, "same-task recovery request");
    }
    return;
  }
  if (msg->command != uav_mission_msgs::MissionCommand::START) return;
  if (have_closed_context_ && msg->context.manager_epoch == closed_context_.manager_epoch &&
      msg->context.mission_id == closed_context_.mission_id &&
      msg->context.task_type == closed_context_.task_type &&
      msg->context.source_id == closed_context_.source_id) return;
  if (current) return;
  if (msg->context.task_type != uav_mission_msgs::TaskContext::EXPLORE &&
      msg->context.task_type != uav_mission_msgs::TaskContext::RETURN_HOME) return;
  if (msg->context.task_type == uav_mission_msgs::TaskContext::RETURN_HOME) {
    return_goal_ << msg->goal.x, msg->goal.y, msg->goal.z;
    if (!return_goal_.allFinite()) return;
  }
  context_ = msg->context;
  executing_traj_.reset();
  submitted_traj_.reset();
  next_trajectory_id_ = 0;
  have_closed_context_ = false;
  return_home_ = context_.task_type == uav_mission_msgs::TaskContext::RETURN_HOME;
  no_frontier_cycles_ = 0;
  plan_failures_ = 0;
  active_trajectory_unsafe_ = false;
  next_plan_attempt_ = ros::Time();
  planning_switch_time_ = ros::Time();
  last_execution_status_ = ros::Time();
  fd_->static_state_ = true;
  fd_->trigger_ = true;
  transitState(PLAN_TRAJ, return_home_ ? "return command" : "explore command");
}

void FastExplorationFSM::triggerCallback(const nav_msgs::PathConstPtr& msg) {
  // Kept for launch compatibility only. Mission admission always requires the
  // manager authorization delivered on /fuel/mission_command.
  if (!msg->poses.empty() && context_.manager_epoch == 0)
    ROS_WARN_THROTTLE(2.0, "Ignoring legacy waypoint trigger without mission authorization");
}

void FastExplorationFSM::trajectoryCallback(
    const uav_mission_msgs::TrajectoryStatusConstPtr& msg) {
  if (!validContext(msg->context) || have_closed_context_ ||
      msg->context.manager_epoch != context_.manager_epoch ||
      msg->context.mission_id != context_.mission_id ||
      msg->context.task_type != context_.task_type ||
      msg->context.source_id != context_.source_id) return;
  if (msg->state == uav_mission_msgs::TrajectoryStatus::ACTIVE && submitted_traj_ &&
      msg->trajectory_id == static_cast<uint64_t>(submitted_traj_->traj_id_)) {
    executing_traj_ = submitted_traj_;
    terminal_error_since_ = ros::Time();
    submitted_traj_.reset();
    plan_failures_ = 0;
    active_trajectory_unsafe_ = false;
    fd_->static_state_ = false;
    next_plan_attempt_ = ros::Time();
    transitState(EXEC_TRAJ, "executor activated trajectory");
  } else if ((msg->state == uav_mission_msgs::TrajectoryStatus::REJECTED && submitted_traj_ &&
              msg->trajectory_id == static_cast<uint64_t>(submitted_traj_->traj_id_)) ||
             msg->state == uav_mission_msgs::TrajectoryStatus::INVALID) {
    if (plan_failures_ < std::numeric_limits<int>::max()) ++plan_failures_;
    submitted_traj_.reset();
    const ros::Time now = ros::Time::now();
    if (msg->state == uav_mission_msgs::TrajectoryStatus::INVALID) {
      active_trajectory_unsafe_ = true;
      fd_->static_state_ = true;
    } else {
      fd_->static_state_ = return_home_ ? !hasSafeActiveTrajectory(now) : true;
    }
    scheduleRetry(now, msg->state == uav_mission_msgs::TrajectoryStatus::INVALID
        ? "active trajectory invalid" : "candidate handover rejected");
    publishPlannerStatus(uav_mission_msgs::PlannerStatus::FAILED, "execution rejected; retrying goal");
    transitState(PLAN_TRAJ, "execution recovery");
  }
}

void FastExplorationFSM::safetyCallback(const ros::TimerEvent& e) {
  // Check the trajectory actually activated by traj_server, including while
  // replacement planning fails or waits for handover.
  if (!executing_traj_ || active_trajectory_unsafe_ || have_closed_context_ ||
      state_ == WAIT_TRIGGER || state_ == INIT) return;
  auto& info = *executing_traj_;
  const auto map = planner_manager_->edt_environment_->sdf_map_;
  const double elapsed = (ros::Time::now() - info.start_time_).toSec();
  const double begin = std::max(0.0, std::min(info.duration_, elapsed));
  bool safe = true;
  for (double t = begin; ; t = std::min(info.duration_, t + 0.05)) {
    const Vector3d point = info.position_traj_.evaluateDeBoorT(t);
    if (!point.allFinite() || !map->isInBox(point) ||
        map->getOccupancy(point) == SDFMap::UNKNOWN || map->getInflateOccupancy(point) != 0) {
      safe = false;
      break;
    }
    if (t >= info.duration_) break;
  }
  if (!safe) {
    active_trajectory_unsafe_ = true;
    fd_->static_state_ = true;
    next_plan_attempt_ = ros::Time::now();
    publishPlannerStatus(uav_mission_msgs::PlannerStatus::TRAJECTORY_UNSAFE,
                         "brake unsafe active path immediately; keep replanning home");
    transitState(PLAN_TRAJ, "unsafe execution recovery");
  }
}

void FastExplorationFSM::odometryCallback(const nav_msgs::OdometryConstPtr& msg) {
  fd_->odom_pos_(0) = msg->pose.pose.position.x;
  fd_->odom_pos_(1) = msg->pose.pose.position.y;
  fd_->odom_pos_(2) = msg->pose.pose.position.z;

  fd_->odom_vel_(0) = msg->twist.twist.linear.x;
  fd_->odom_vel_(1) = msg->twist.twist.linear.y;
  fd_->odom_vel_(2) = msg->twist.twist.linear.z;

  fd_->odom_orient_.w() = msg->pose.pose.orientation.w;
  fd_->odom_orient_.x() = msg->pose.pose.orientation.x;
  fd_->odom_orient_.y() = msg->pose.pose.orientation.y;
  fd_->odom_orient_.z() = msg->pose.pose.orientation.z;

  if (!fd_->odom_orient_.coeffs().allFinite() || fd_->odom_orient_.norm() < 1e-6 ||
      !fd_->odom_pos_.allFinite() || !fd_->odom_vel_.allFinite()) return;
  fd_->odom_orient_.normalize();
  if (odom_twist_in_body_) fd_->odom_vel_ = fd_->odom_orient_ * fd_->odom_vel_;
  Eigen::Vector3d rot_x = fd_->odom_orient_.toRotationMatrix().block<3, 1>(0, 0);
  fd_->odom_yaw_ = atan2(rot_x(1), rot_x(0));

  fd_->have_odom_ = true;
}

void FastExplorationFSM::transitState(EXPL_STATE new_state, string pos_call) {
  int pre_s = int(state_);
  state_ = new_state;
  ROS_DEBUG_STREAM("[" + pos_call + "]: from " + fd_->state_str_[pre_s] + " to " + fd_->state_str_[int(new_state)]);
}
}  // namespace fast_planner
