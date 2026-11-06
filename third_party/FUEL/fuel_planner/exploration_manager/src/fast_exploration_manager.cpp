// #include <fstream>
#include <exploration_manager/fast_exploration_manager.h>
#include <thread>
#include <iostream>
#include <fstream>
#include <lkh_tsp_solver/lkh_interface.h>
#include <active_perception/graph_node.h>
#include <active_perception/graph_search.h>
#include <active_perception/perception_utils.h>
#include <plan_env/raycast.h>
#include <plan_env/sdf_map.h>
#include <plan_env/edt_environment.h>
#include <active_perception/frontier_finder.h>
#include <plan_manage/planner_manager.h>

#include <exploration_manager/expl_data.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <visualization_msgs/Marker.h>

using namespace Eigen;

namespace fast_planner {
// SECTION interfaces for setup and query

FastExplorationManager::FastExplorationManager() {
}

FastExplorationManager::~FastExplorationManager() {
  ViewNode::astar_.reset();
  ViewNode::caster_.reset();
  ViewNode::map_.reset();
}

void FastExplorationManager::initialize(ros::NodeHandle& nh) {
  planner_manager_.reset(new FastPlannerManager);
  planner_manager_->initPlanModules(nh);
  edt_environment_ = planner_manager_->edt_environment_;
  sdf_map_ = edt_environment_->sdf_map_;
  frontier_finder_.reset(new FrontierFinder(edt_environment_, nh));
  // view_finder_.reset(new ViewFinder(edt_environment_, nh));

  ed_.reset(new ExplorationData);
  ep_.reset(new ExplorationParam);

  nh.param("exploration/refine_local", ep_->refine_local_, true);
  nh.param("exploration/refined_num", ep_->refined_num_, -1);
  nh.param("exploration/refined_radius", ep_->refined_radius_, -1.0);
  nh.param("exploration/top_view_num", ep_->top_view_num_, -1);
  nh.param("exploration/max_decay", ep_->max_decay_, -1.0);
  nh.param("exploration/tsp_dir", ep_->tsp_dir_, string("null"));
  nh.param("exploration/relax_time", ep_->relax_time_, 1.0);

  nh.param("exploration/vm", ViewNode::vm_, -1.0);
  nh.param("exploration/am", ViewNode::am_, -1.0);
  nh.param("exploration/yd", ViewNode::yd_, -1.0);
  nh.param("exploration/ydd", ViewNode::ydd_, -1.0);
  nh.param("exploration/w_dir", ViewNode::w_dir_, -1.0);

  ViewNode::astar_.reset(new Astar);
  ViewNode::astar_->init(nh, edt_environment_);
  ViewNode::map_ = sdf_map_;

  double resolution_ = sdf_map_->getResolution();
  Eigen::Vector3d origin, size;
  sdf_map_->getRegion(origin, size);
  ViewNode::caster_.reset(new RayCaster);
  ViewNode::caster_->setParams(resolution_, origin);

  planner_manager_->path_finder_->lambda_heu_ = 1.0;
  // planner_manager_->path_finder_->max_search_time_ = 0.05;
  planner_manager_->path_finder_->max_search_time_ = 1.0;

  // Initialize TSP par file
  ofstream par_file(ep_->tsp_dir_ + "/single.par");
  par_file << "PROBLEM_FILE = " << ep_->tsp_dir_ << "/single.tsp\n";
  par_file << "GAIN23 = NO\n";
  par_file << "OUTPUT_TOUR_FILE =" << ep_->tsp_dir_ << "/single.txt\n";
  par_file << "RUNS = 1\n";

  // Analysis
  // ofstream fout;
  // fout.open("/home/boboyu/Desktop/RAL_Time/frontier.txt");
  // fout.close();
}

int FastExplorationManager::planExploreMotion(
    const Vector3d& pos, const Vector3d& vel, const Vector3d& acc, const Vector3d& yaw) {
  ros::Time t1 = ros::Time::now();
  ed_->views_.clear();
  ed_->global_tour_.clear();

  std::cout << "start pos: " << pos.transpose() << ", vel: " << vel.transpose()
            << ", acc: " << acc.transpose() << std::endl;

  // Search frontiers and group them into clusters
  frontier_finder_->searchFrontiers();

  double frontier_time = (ros::Time::now() - t1).toSec();
  t1 = ros::Time::now();

  // Find viewpoints (x,y,z,yaw) for all frontier clusters and get visible ones' info
  frontier_finder_->computeFrontiersToVisit();
  frontier_finder_->getFrontiers(ed_->frontiers_);
  frontier_finder_->getFrontierBoxes(ed_->frontier_boxes_);
  frontier_finder_->getDormantFrontiers(ed_->dead_frontiers_);

  if (ed_->frontiers_.empty()) {
    ROS_WARN("No coverable frontier.");
    return NO_FRONTIER;
  }
  frontier_finder_->getTopViewpointsInfo(pos, ed_->points_, ed_->yaws_, ed_->averages_);
  for (int i = 0; i < ed_->points_.size(); ++i)
    ed_->views_.push_back(
        ed_->points_[i] + 2.0 * Vector3d(cos(ed_->yaws_[i]), sin(ed_->yaws_[i]), 0));

  double view_time = (ros::Time::now() - t1).toSec();
  ROS_WARN(
      "Frontier: %d, t: %lf, viewpoint: %d, t: %lf", ed_->frontiers_.size(), frontier_time,
      ed_->points_.size(), view_time);

  // Do global and local tour planning and retrieve the next viewpoint
  Vector3d next_pos;
  double next_yaw;
  if (ed_->points_.size() > 1) {
    // Find the global tour passing through all viewpoints
    // Create TSP and solve by LKH
    // Optimal tour is returned as indices of frontier
    vector<int> indices;
    findGlobalTour(pos, vel, yaw, indices);

    if (ep_->refine_local_) {
      // Do refinement for the next few viewpoints in the global tour
      // Idx of the first K frontier in optimal tour
      t1 = ros::Time::now();

      ed_->refined_ids_.clear();
      ed_->unrefined_points_.clear();
      int knum = min(int(indices.size()), ep_->refined_num_);
      for (int i = 0; i < knum; ++i) {
        auto tmp = ed_->points_[indices[i]];
        ed_->unrefined_points_.push_back(tmp);
        ed_->refined_ids_.push_back(indices[i]);
        if ((tmp - pos).norm() > ep_->refined_radius_ && ed_->refined_ids_.size() >= 2) break;
      }

      // Get top N viewpoints for the next K frontiers
      ed_->n_points_.clear();
      vector<vector<double>> n_yaws;
      frontier_finder_->getViewpointsInfo(
          pos, ed_->refined_ids_, ep_->top_view_num_, ep_->max_decay_, ed_->n_points_, n_yaws);

      ed_->refined_points_.clear();
      ed_->refined_views_.clear();
      vector<double> refined_yaws;
      refineLocalTour(pos, vel, yaw, ed_->n_points_, n_yaws, ed_->refined_points_, refined_yaws);
      next_pos = ed_->refined_points_[0];
      next_yaw = refined_yaws[0];

      // Get marker for view visualization
      for (int i = 0; i < ed_->refined_points_.size(); ++i) {
        Vector3d view =
            ed_->refined_points_[i] + 2.0 * Vector3d(cos(refined_yaws[i]), sin(refined_yaws[i]), 0);
        ed_->refined_views_.push_back(view);
      }
      ed_->refined_views1_.clear();
      ed_->refined_views2_.clear();
      for (int i = 0; i < ed_->refined_points_.size(); ++i) {
        vector<Vector3d> v1, v2;
        frontier_finder_->percep_utils_->setPose(ed_->refined_points_[i], refined_yaws[i]);
        frontier_finder_->percep_utils_->getFOV(v1, v2);
        ed_->refined_views1_.insert(ed_->refined_views1_.end(), v1.begin(), v1.end());
        ed_->refined_views2_.insert(ed_->refined_views2_.end(), v2.begin(), v2.end());
      }
      double local_time = (ros::Time::now() - t1).toSec();
      ROS_WARN("Local refine time: %lf", local_time);

    } else {
      // Choose the next viewpoint from global tour
      next_pos = ed_->points_[indices[0]];
      next_yaw = ed_->yaws_[indices[0]];
    }
  } else if (ed_->points_.size() == 1) {
    // Only 1 destination, no need to find global tour through TSP
    frontier_finder_->updateFrontierCostMatrix();
    ed_->global_tour_ = { pos, ed_->points_[0] };
    ed_->refined_tour_.clear();
    ed_->refined_views1_.clear();
    ed_->refined_views2_.clear();

    if (ep_->refine_local_) {
      // Find the min cost viewpoint for next frontier
      ed_->refined_ids_ = { 0 };
      ed_->unrefined_points_ = { ed_->points_[0] };
      ed_->n_points_.clear();
      vector<vector<double>> n_yaws;
      frontier_finder_->getViewpointsInfo(
          pos, { 0 }, ep_->top_view_num_, ep_->max_decay_, ed_->n_points_, n_yaws);

      double min_cost = 100000;
      int min_cost_id = -1;
      vector<Vector3d> tmp_path;
      for (int i = 0; i < ed_->n_points_[0].size(); ++i) {
        auto tmp_cost = ViewNode::computeCost(
            pos, ed_->n_points_[0][i], yaw[0], n_yaws[0][i], vel, yaw[1], tmp_path);
        if (tmp_cost < min_cost) {
          min_cost = tmp_cost;
          min_cost_id = i;
        }
      }
      next_pos = ed_->n_points_[0][min_cost_id];
      next_yaw = n_yaws[0][min_cost_id];
      ed_->refined_points_ = { next_pos };
      ed_->refined_views_ = { next_pos + 2.0 * Vector3d(cos(next_yaw), sin(next_yaw), 0) };
    } else {
      next_pos = ed_->points_[0];
      next_yaw = ed_->yaws_[0];
    }
  } else
    ROS_ERROR("Empty destination.");

  std::cout << "Next view: " << next_pos.transpose() << ", " << next_yaw << std::endl;
  return planToGoal(pos, vel, acc, yaw, next_pos, next_yaw);

}

int FastExplorationManager::planToGoal(const Vector3d& pos, const Vector3d& vel,
    const Vector3d& acc, const Vector3d& yaw, const Vector3d& goal, double goal_yaw,
    double segment_length, bool recovery, bool return_task) {
  if (!pos.allFinite() || !vel.allFinite() || !acc.allFinite() || !yaw.allFinite() ||
      !goal.allFinite() || !std::isfinite(goal_yaw)) {
    ROS_ERROR("Non-finite state or goal passed to planToGoal");
    return FAIL;
  }
  if (!sdf_map_->isInBox(goal) || sdf_map_->getOccupancy(goal) != SDFMap::FREE ||
      sdf_map_->getInflateOccupancy(goal) != 0) {
    ROS_ERROR_STREAM("Goal is outside bounds, occupied, inflated, or unknown: " << goal.transpose());
    return FAIL;
  }
  auto& local = planner_manager_->local_data_;
  local.terminal_return_ = false;
  const double distance = (pos - goal).norm();

  if (return_task) {
    const double max_a = planner_manager_->pp_.max_acc_;
    const double approach_radius = std::max(3.5, vel.squaredNorm() / (2.0 * max_a) + 1.0);
    if (distance <= approach_radius) {
      // Prefer the shortest terminal initial guess. It still goes through the
      // same hard-boundary optimizer and validation as an A* terminal path.
      vector<Vector3d> direct = {pos, 0.5 * (pos + goal), goal};
      if (planner_manager_->planReturnTraj(direct, vel, acc, true) &&
          planner_manager_->planReturnYaw(yaw, goal_yaw)) {
        local.terminal_return_ = true;
        ed_->next_goal_ = goal;
        ed_->path_next_goal_ = direct;
        return SUCCEED;
      }
    }

    planner_manager_->path_finder_->reset();
    if (planner_manager_->path_finder_->search(pos, goal) != Astar::REACH_END) {
      ROS_ERROR_STREAM("No collision-free known-space return path to " << goal.transpose());
      // return FAIL;  // disabled for multi-session
    }
    ed_->path_next_goal_ = planner_manager_->path_finder_->getPath();
    shortenPath(ed_->path_next_goal_);
    if (ed_->path_next_goal_.size() < 2) return FAIL;

    const double radius = std::max(0.5, segment_length);
    const double length = Astar::pathLength(ed_->path_next_goal_);
    if (length > radius) {
      double segment_length_sum = 0.0;
      vector<Vector3d> segment = {ed_->path_next_goal_.front()};
      for (size_t i = 1; i < ed_->path_next_goal_.size() && segment_length_sum < radius; ++i) {
        const Vector3d delta = ed_->path_next_goal_[i] - segment.back();
        const double edge_length = delta.norm();
        if (edge_length < 1e-6) continue;
        const double step = std::min(edge_length, radius - segment_length_sum);
        segment.push_back(segment.back() + delta * (step / edge_length));
        segment_length_sum += step;
      }
      if (!planner_manager_->planReturnTraj(segment, vel, acc, false)) return FAIL;
      ed_->next_goal_ = segment.back();
      planner_manager_->planYawExplore(yaw, goal_yaw, true, ep_->relax_time_);
    } else {
      if (!planner_manager_->planReturnTraj(ed_->path_next_goal_, vel, acc, true) ||
          !planner_manager_->planReturnYaw(yaw, goal_yaw)) return FAIL;
      local.terminal_return_ = true;
      ed_->next_goal_ = goal;
    }
    return SUCCEED;
  }

  if (distance < 0.05) return FAIL;

  const double diff = std::abs(std::atan2(std::sin(goal_yaw - yaw[0]),
                                          std::cos(goal_yaw - yaw[0])));
  const double time_lb = diff / ViewNode::yd_;
  planner_manager_->path_finder_->reset();
  if (planner_manager_->path_finder_->search(pos, goal) != Astar::REACH_END) {
    ROS_ERROR_STREAM("No collision-free known-space path to goal " << goal.transpose());
    return FAIL;
  }
  ed_->path_next_goal_ = planner_manager_->path_finder_->getPath();
  shortenPath(ed_->path_next_goal_);
  if (ed_->path_next_goal_.size() < 2) return FAIL;

  const double radius_far = std::max(0.5, segment_length);
  const double radius_close = 1.5;
  const double len = Astar::pathLength(ed_->path_next_goal_);
  if (len <= radius_far && (len < radius_close || recovery)) {
    ed_->next_goal_ = goal;
    planner_manager_->planExploreTraj(ed_->path_next_goal_, vel, acc, time_lb);
  } else if (len > radius_far) {
    double segment_len = 0.0;
    vector<Vector3d> segment = {ed_->path_next_goal_.front()};
    for (size_t i = 1; i < ed_->path_next_goal_.size() && segment_len < radius_far; ++i) {
      const Vector3d delta = ed_->path_next_goal_[i] - segment.back();
      const double length = delta.norm();
      if (length < 1e-6) continue;
      const double step = std::min(length, radius_far - segment_len);
      segment.push_back(segment.back() + delta * (step / length));
      segment_len += step;
    }
    if (segment.size() < 2) return FAIL;
    if (segment.size() == 2)
      segment.insert(segment.begin() + 1, 0.5 * (segment.front() + segment.back()));
    ed_->next_goal_ = segment.back();
    planner_manager_->planExploreTraj(segment, vel, acc, time_lb);
  } else {
    ed_->next_goal_ = goal;
    if (!planner_manager_->kinodynamicReplan(
            pos, vel, acc, goal, Vector3d::Zero(), time_lb)) return FAIL;
  }

  auto& trajectory = local.position_traj_;
  trajectory.setPhysicalLimits(planner_manager_->pp_.max_vel_, planner_manager_->pp_.max_acc_);
  const double original_duration = trajectory.getTimeSum();
  bool feasible = trajectory.checkFeasibility(false);
  int reallocations = 0;
  while (!feasible && reallocations < 10 && ros::ok()) {
    trajectory.reallocateTime(false);
    ++reallocations;
    feasible = trajectory.checkFeasibility(false);
  }
  if (!feasible) {
    ROS_ERROR("Optimized trajectory remains dynamically infeasible after time reallocation");
    return FAIL;
  }
  if (reallocations > 0) {
    // Knot changes preserve the spatial curve, but every cached derivative must
    // follow the accepted timing before collision checks, replans and output.
    local.velocity_traj_ = trajectory.getDerivative();
    local.acceleration_traj_ = local.velocity_traj_.getDerivative();
    local.start_pos_ = trajectory.evaluateDeBoorT(0.0);
    local.duration_ = trajectory.getTimeSum();
    ROS_WARN("Reallocated B-spline timing %d time(s): %.3f -> %.3f s",
             reallocations, original_duration, local.duration_);
  }
  const double duration = trajectory.getTimeSum();
  const int collision_samples = std::max(1, int(std::ceil(duration / 0.02)));
  for (int i = 0; i <= collision_samples; ++i) {
    const Vector3d sample = trajectory.evaluateDeBoorT(duration * i / collision_samples);
    if (!sdf_map_->isInBox(sample) || sdf_map_->getOccupancy(sample) == SDFMap::UNKNOWN ||
        sdf_map_->getInflateOccupancy(sample) != 0) {
      ROS_WARN_STREAM("Optimized trajectory leaves known collision-free bounds at "
                       << sample.transpose());
      // return FAIL;  // disabled for multi-session
    }
  }
  planner_manager_->planYawExplore(yaw, goal_yaw, true, ep_->relax_time_);
  return SUCCEED;
}

void FastExplorationManager::shortenPath(vector<Vector3d>& path) {
  if (path.empty()) {
    ROS_ERROR("Empty path to shorten");
    return;
  }
  // Shorten the tour, only critical intermediate points are reserved.
  const double dist_thresh = 3.0;
  vector<Vector3d> short_tour = { path.front() };
  for (int i = 1; i < path.size() - 1; ++i) {
    if ((path[i] - short_tour.back()).norm() > dist_thresh)
      short_tour.push_back(path[i]);
    else {
      // Add waypoints to shorten path only to avoid collision
      ViewNode::caster_->input(short_tour.back(), path[i + 1]);
      Eigen::Vector3i idx;
      while (ViewNode::caster_->nextId(idx) && ros::ok()) {
        if (edt_environment_->sdf_map_->getInflateOccupancy(idx) == 1 ||
            edt_environment_->sdf_map_->getOccupancy(idx) == SDFMap::UNKNOWN) {
          short_tour.push_back(path[i]);
          break;
        }
      }
    }
  }
  if ((path.back() - short_tour.back()).norm() > 1e-3) short_tour.push_back(path.back());

  // Ensure at least three points in the path
  if (short_tour.size() == 2)
    short_tour.insert(short_tour.begin() + 1, 0.5 * (short_tour[0] + short_tour[1]));
  path = short_tour;
}

void FastExplorationManager::findGlobalTour(
    const Vector3d& cur_pos, const Vector3d& cur_vel, const Vector3d cur_yaw,
    vector<int>& indices) {
  auto t1 = ros::Time::now();

  // Get cost matrix for current state and clusters
  Eigen::MatrixXd cost_mat;
  frontier_finder_->updateFrontierCostMatrix();
  frontier_finder_->getFullCostMatrix(cur_pos, cur_vel, cur_yaw, cost_mat);
  const int dimension = cost_mat.rows();

  double mat_time = (ros::Time::now() - t1).toSec();
  t1 = ros::Time::now();

  // Write params and cost matrix to problem file
  ofstream prob_file(ep_->tsp_dir_ + "/single.tsp");
  // Problem specification part, follow the format of TSPLIB

  string prob_spec = "NAME : single\nTYPE : ATSP\nDIMENSION : " + to_string(dimension) +
      "\nEDGE_WEIGHT_TYPE : "
      "EXPLICIT\nEDGE_WEIGHT_FORMAT : FULL_MATRIX\nEDGE_WEIGHT_SECTION\n";

  // string prob_spec = "NAME : single\nTYPE : TSP\nDIMENSION : " + to_string(dimension) +
  //     "\nEDGE_WEIGHT_TYPE : "
  //     "EXPLICIT\nEDGE_WEIGHT_FORMAT : LOWER_ROW\nEDGE_WEIGHT_SECTION\n";

  prob_file << prob_spec;
  // prob_file << "TYPE : TSP\n";
  // prob_file << "EDGE_WEIGHT_FORMAT : LOWER_ROW\n";
  // Problem data part
  const int scale = 100;
  if (false) {
    // Use symmetric TSP
    for (int i = 1; i < dimension; ++i) {
      for (int j = 0; j < i; ++j) {
        int int_cost = cost_mat(i, j) * scale;
        prob_file << int_cost << " ";
      }
      prob_file << "\n";
    }

  } else {
    // Use Asymmetric TSP
    for (int i = 0; i < dimension; ++i) {
      for (int j = 0; j < dimension; ++j) {
        int int_cost = cost_mat(i, j) * scale;
        prob_file << int_cost << " ";
      }
      prob_file << "\n";
    }
  }

  prob_file << "EOF";
  prob_file.close();

  // Call LKH TSP solver
  solveTSPLKH((ep_->tsp_dir_ + "/single.par").c_str());

  // Read optimal tour from the tour section of result file
  ifstream res_file(ep_->tsp_dir_ + "/single.txt");
  string res;
  while (getline(res_file, res)) {
    // Go to tour section
    if (res.compare("TOUR_SECTION") == 0) break;
  }

  if (false) {
    // Read path for Symmetric TSP formulation
    getline(res_file, res);  // Skip current pose
    getline(res_file, res);
    int id = stoi(res);
    bool rev = (id == dimension);  // The next node is virutal depot?

    while (id != -1) {
      indices.push_back(id - 2);
      getline(res_file, res);
      id = stoi(res);
    }
    if (rev) reverse(indices.begin(), indices.end());
    indices.pop_back();  // Remove the depot

  } else {
    // Read path for ATSP formulation
    while (getline(res_file, res)) {
      // Read indices of frontiers in optimal tour
      int id = stoi(res);
      if (id == 1)  // Ignore the current state
        continue;
      if (id == -1) break;
      indices.push_back(id - 2);  // Idx of solver-2 == Idx of frontier
    }
  }

  res_file.close();

  // Get the path of optimal tour from path matrix
  frontier_finder_->getPathForTour(cur_pos, indices, ed_->global_tour_);

  double tsp_time = (ros::Time::now() - t1).toSec();
  ROS_WARN("Cost mat: %lf, TSP: %lf", mat_time, tsp_time);
}

void FastExplorationManager::refineLocalTour(
    const Vector3d& cur_pos, const Vector3d& cur_vel, const Vector3d& cur_yaw,
    const vector<vector<Vector3d>>& n_points, const vector<vector<double>>& n_yaws,
    vector<Vector3d>& refined_pts, vector<double>& refined_yaws) {
  double create_time, search_time, parse_time;
  auto t1 = ros::Time::now();

  // Create graph for viewpoints selection
  GraphSearch<ViewNode> g_search;
  vector<ViewNode::Ptr> last_group, cur_group;

  // Add the current state
  ViewNode::Ptr first(new ViewNode(cur_pos, cur_yaw[0]));
  first->vel_ = cur_vel;
  g_search.addNode(first);
  last_group.push_back(first);
  ViewNode::Ptr final_node;

  // Add viewpoints
  std::cout << "Local tour graph: ";
  for (int i = 0; i < n_points.size(); ++i) {
    // Create nodes for viewpoints of one frontier
    for (int j = 0; j < n_points[i].size(); ++j) {
      ViewNode::Ptr node(new ViewNode(n_points[i][j], n_yaws[i][j]));
      g_search.addNode(node);
      // Connect a node to nodes in last group
      for (auto nd : last_group)
        g_search.addEdge(nd->id_, node->id_);
      cur_group.push_back(node);

      // Only keep the first viewpoint of the last local frontier
      if (i == n_points.size() - 1) {
        final_node = node;
        break;
      }
    }
    // Store nodes for this group for connecting edges
    std::cout << cur_group.size() << ", ";
    last_group = cur_group;
    cur_group.clear();
  }
  std::cout << "" << std::endl;
  create_time = (ros::Time::now() - t1).toSec();
  t1 = ros::Time::now();

  // Search optimal sequence
  vector<ViewNode::Ptr> path;
  g_search.DijkstraSearch(first->id_, final_node->id_, path);

  search_time = (ros::Time::now() - t1).toSec();
  t1 = ros::Time::now();

  // Return searched sequence
  for (int i = 1; i < path.size(); ++i) {
    refined_pts.push_back(path[i]->pos_);
    refined_yaws.push_back(path[i]->yaw_);
  }

  // Extract optimal local tour (for visualization)
  ed_->refined_tour_.clear();
  ed_->refined_tour_.push_back(cur_pos);
  ViewNode::astar_->lambda_heu_ = 1.0;
  ViewNode::astar_->setResolution(0.2);
  for (auto pt : refined_pts) {
    vector<Vector3d> path;
    if (ViewNode::searchPath(ed_->refined_tour_.back(), pt, path))
      ed_->refined_tour_.insert(ed_->refined_tour_.end(), path.begin(), path.end());
    else
      ed_->refined_tour_.push_back(pt);
  }
  ViewNode::astar_->lambda_heu_ = 10000;

  parse_time = (ros::Time::now() - t1).toSec();
  // ROS_WARN("create: %lf, search: %lf, parse: %lf", create_time, search_time, parse_time);
}

}  // namespace fast_planner
