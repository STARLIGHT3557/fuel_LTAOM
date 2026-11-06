#include <algorithm>
#include <cmath>
#include <cstdint>
#include <stdexcept>
#include <string>

#include <Eigen/Eigen>
#include <bspline/Bspline.h>
#include <bspline/non_uniform_bspline.h>
#include <nav_msgs/Odometry.h>
#include <quadrotor_msgs/PositionCommand.h>
#include <ros/ros.h>
#include <std_msgs/Empty.h>
#include <uav_mission_msgs/ControllerAck.h>
#include <uav_mission_msgs/MissionCommand.h>
#include <uav_mission_msgs/PlannerStatus.h>
#include <uav_mission_msgs/TaskContext.h>
#include <uav_mission_msgs/TrajectoryStatus.h>

using fast_planner::NonUniformBspline;

namespace {
struct TrajectorySlot {
  bool valid{false};
  bool controller_accepted{false};
  bool completion_reported{false};
  uav_mission_msgs::TaskContext context;
  uint64_t id{0};
  ros::Time start_time;
  ros::Time received_at;
  double duration{0.0};
  NonUniformBspline position, velocity, acceleration, yaw, yaw_rate;
};

ros::Publisher command_pub, status_pub;
nav_msgs::Odometry odom;
bool odom_twist_in_body{false};
uav_mission_msgs::TaskContext authorization;
TrajectorySlot active, pending;
quadrotor_msgs::PositionCommand command;
std::string world_frame{"world"};
std::string source_id{"fuel_planner"};
double position_switch_tolerance{0.75};
double velocity_switch_tolerance{1.0};
double acceleration_switch_tolerance{2.0};
double handover_timeout{2.0};
double handover_margin{0.1};
double braking_acceleration{1.0};

bool braking{false};
ros::Time braking_started;
Eigen::Vector3d braking_position{Eigen::Vector3d::Zero()};
Eigen::Vector3d braking_velocity{Eigen::Vector3d::Zero()};
uav_mission_msgs::TaskContext braking_context, closed_context;
uint64_t braking_trajectory_id{0};
double braking_yaw{0.0};
bool have_closed_context{false};

bool sameContext(const uav_mission_msgs::TaskContext& a,
                 const uav_mission_msgs::TaskContext& b) {
  return a.manager_epoch == b.manager_epoch && a.mission_id == b.mission_id &&
         a.task_type == b.task_type && a.source_id == b.source_id;
}

bool validContext(const uav_mission_msgs::TaskContext& context) {
  return context.manager_epoch != 0 && context.mission_id != 0 &&
         context.task_type != uav_mission_msgs::TaskContext::TASK_NONE &&
         context.source_id == source_id && sameContext(context, authorization);
}

void publishStatus(const TrajectorySlot& slot, uint8_t state, const std::string& detail) {
  uav_mission_msgs::TrajectoryStatus status;
  status.header.stamp = ros::Time::now();
  status.header.frame_id = world_frame;
  status.context = slot.context;
  status.trajectory_id = slot.id;
  status.state = state;
  status.detail = detail;
  status_pub.publish(status);
}

void evaluate(TrajectorySlot& slot, const ros::Time& stamp, Eigen::Vector3d& position,
              Eigen::Vector3d& velocity, Eigen::Vector3d& acceleration,
              double& yaw, double& yaw_rate) {
  const double t = std::max(0.0, std::min(slot.duration, (stamp - slot.start_time).toSec()));
  position = slot.position.evaluateDeBoorT(t);
  yaw = slot.yaw.evaluateDeBoorT(t)[0];
  if (t >= slot.duration) {
    velocity.setZero();
    acceleration.setZero();
    yaw_rate = 0.0;
  } else {
    velocity = slot.velocity.evaluateDeBoorT(t);
    acceleration = slot.acceleration.evaluateDeBoorT(t);
    yaw_rate = slot.yaw_rate.evaluateDeBoorT(t)[0];
  }
}

void evaluateBraking(const ros::Time& stamp, Eigen::Vector3d& position,
                     Eigen::Vector3d& velocity, Eigen::Vector3d& acceleration) {
  const double stop_time = braking_velocity.norm() / braking_acceleration;
  const double t = std::max(0.0, std::min(stop_time, (stamp - braking_started).toSec()));
  position = braking_position;
  velocity.setZero();
  acceleration.setZero();
  if (stop_time > 1e-6) {
    velocity = braking_velocity * (1.0 - t / stop_time);
    if (t < stop_time) acceleration = -braking_velocity / stop_time;
    position += braking_velocity * (t - 0.5 * t * t / stop_time);
  }
}

bool continuousWithExecution(TrajectorySlot& candidate, const ros::Time& switch_time) {
  Eigen::Vector3d reference_p, reference_v, reference_a, new_p, new_v, new_a;
  double reference_yaw = 0.0, reference_yaw_rate = 0.0, new_yaw, new_yaw_rate;
  if (braking && sameContext(braking_context, candidate.context)) {
    evaluateBraking(switch_time, reference_p, reference_v, reference_a);
  } else if (active.valid && sameContext(active.context, candidate.context)) {
    evaluate(active, switch_time, reference_p, reference_v, reference_a,
             reference_yaw, reference_yaw_rate);
  } else {
    const ros::Time now = ros::Time::now();
    const double age = (now - odom.header.stamp).toSec();
    if (odom.header.stamp.isZero() || age < -0.05 || age >= 0.5) {
      ROS_ERROR("Reject trajectory without fresh odometry for task handover");
      return false;
    }
    reference_p << odom.pose.pose.position.x, odom.pose.pose.position.y,
        odom.pose.pose.position.z;
    reference_v << odom.twist.twist.linear.x, odom.twist.twist.linear.y,
        odom.twist.twist.linear.z;
    // Odometry has no linear acceleration. A new task starts from the
    // controller's current hold/flight state, so require a bounded starting
    // acceleration relative to the zero-acceleration handover reference.
    reference_a.setZero();
  }
  evaluate(candidate, switch_time, new_p, new_v, new_a, new_yaw, new_yaw_rate);
  const double dp = (reference_p - new_p).norm();
  const double dv = (reference_v - new_v).norm();
  const double da = (reference_a - new_a).norm();
  const bool continuous = reference_p.allFinite() && reference_v.allFinite() &&
                          reference_a.allFinite() && dp <= position_switch_tolerance &&
                          dv <= velocity_switch_tolerance &&
                          da <= acceleration_switch_tolerance;
  if (!continuous)
    ROS_ERROR("Reject discontinuous trajectory: dp=%.3f dv=%.3f da=%.3f",
              dp, dv, da);
  return continuous;
}

bool parseSpline(const bspline::Bspline& msg, TrajectorySlot& slot) {
  if (!validContext(msg.context) || msg.traj_id <= 0 || msg.order < 1 ||
      msg.pos_pts.size() <= static_cast<size_t>(msg.order) || msg.knots.empty() ||
      msg.yaw_pts.size() < 4 || !std::isfinite(msg.yaw_dt) || msg.yaw_dt <= 0.0 ||
      msg.start_time.isZero()) return false;
  Eigen::MatrixXd position_points(msg.pos_pts.size(), 3);
  for (size_t i = 0; i < msg.pos_pts.size(); ++i) {
    position_points.row(i) << msg.pos_pts[i].x, msg.pos_pts[i].y, msg.pos_pts[i].z;
    if (!position_points.row(i).allFinite()) return false;
  }
  Eigen::VectorXd knots(msg.knots.size());
  for (size_t i = 0; i < msg.knots.size(); ++i) {
    if (!std::isfinite(msg.knots[i])) return false;
    knots(i) = msg.knots[i];
  }
  Eigen::MatrixXd yaw_points(msg.yaw_pts.size(), 1);
  for (size_t i = 0; i < msg.yaw_pts.size(); ++i) {
    if (!std::isfinite(msg.yaw_pts[i])) return false;
    yaw_points(i, 0) = msg.yaw_pts[i];
  }

  slot = TrajectorySlot{};
  slot.context = msg.context;
  slot.id = static_cast<uint64_t>(msg.traj_id);
  slot.start_time = msg.start_time;
  slot.received_at = ros::Time::now();
  slot.position = NonUniformBspline(position_points, msg.order, 0.1);
  slot.position.setKnot(knots);
  slot.velocity = slot.position.getDerivative();
  slot.acceleration = slot.velocity.getDerivative();
  slot.yaw = NonUniformBspline(yaw_points, 3, msg.yaw_dt);
  slot.yaw_rate = slot.yaw.getDerivative();
  slot.duration = slot.position.getTimeSum();
  slot.valid = std::isfinite(slot.duration) && slot.duration > 0.0;
  return slot.valid;
}

void splineCallback(const bspline::BsplineConstPtr& msg) {
  if (have_closed_context && sameContext(msg->context, closed_context)) {
    TrajectorySlot rejected;
    rejected.context = msg->context;
    rejected.id = msg->traj_id > 0 ? static_cast<uint64_t>(msg->traj_id) : 0;
    publishStatus(rejected, uav_mission_msgs::TrajectoryStatus::REJECTED,
                  "trajectory belongs to a closed mission batch");
    return;
  }
  // Trajectory ids are monotonic only within one task batch. A new task may
  // restart numbering without being compared with the previous task.
  const uint64_t greatest_id = std::max(
      active.valid && sameContext(active.context, msg->context) ? active.id : 0,
      pending.valid && sameContext(pending.context, msg->context) ? pending.id : 0);
  TrajectorySlot candidate;
  if (msg->traj_id <= 0 || static_cast<uint64_t>(msg->traj_id) <= greatest_id ||
      !parseSpline(*msg, candidate)) {
    candidate.context = msg->context;
    candidate.id = msg->traj_id > 0 ? static_cast<uint64_t>(msg->traj_id) : 0;
    publishStatus(candidate, uav_mission_msgs::TrajectoryStatus::REJECTED,
                  "invalid, unauthorized, or non-increasing B-spline");
    return;
  }
  const bool return_task = candidate.context.task_type == uav_mission_msgs::TaskContext::RETURN_HOME;
  const ros::Time now = ros::Time::now();
  if (return_task && candidate.start_time <= now) {
    publishStatus(candidate, uav_mission_msgs::TrajectoryStatus::REJECTED,
                  "trajectory arrived after its scheduled handover time");
    return;
  }
  if (!continuousWithExecution(candidate, std::max(candidate.start_time, now))) {
    publishStatus(candidate, uav_mission_msgs::TrajectoryStatus::REJECTED,
                  "trajectory does not match active position/velocity/acceleration at switch");
    return;
  }
  pending = candidate;
  publishStatus(pending, uav_mission_msgs::TrajectoryStatus::PENDING,
                "trajectory validated; waiting for controller takeover confirmation");
}

void authorizationCallback(const uav_mission_msgs::TaskContextConstPtr& msg) {
  authorization = *msg;
  if (have_closed_context && !sameContext(closed_context, authorization))
    have_closed_context = false;
  if (pending.valid && !sameContext(pending.context, authorization)) pending = TrajectorySlot{};
  if (active.valid && !sameContext(active.context, authorization)) active = TrajectorySlot{};
  if (braking && !sameContext(braking_context, authorization)) braking = false;
}

void ackCallback(const uav_mission_msgs::ControllerAckConstPtr& msg) {
  if (pending.valid && sameContext(msg->context, pending.context) &&
      msg->trajectory_id == pending.id && msg->accepted)
    pending.controller_accepted = true;
}

void startBraking(const uav_mission_msgs::PlannerStatus& status) {
  if (braking) return;
  braking = true;
  braking_started = ros::Time::now();
  braking_position << odom.pose.pose.position.x, odom.pose.pose.position.y, odom.pose.pose.position.z;
  braking_velocity << odom.twist.twist.linear.x, odom.twist.twist.linear.y,
      odom.twist.twist.linear.z;
  braking_context = status.context;
  braking_trajectory_id = status.trajectory_id != 0
      ? status.trajectory_id : (active.valid ? active.id : 1);
  const auto& q = odom.pose.pose.orientation;
  braking_yaw = std::atan2(2.0 * (q.w * q.z + q.x * q.y),
                           1.0 - 2.0 * (q.y * q.y + q.z * q.z));
  pending = TrajectorySlot{};
  active = TrajectorySlot{};
  TrajectorySlot invalid;
  invalid.context = braking_context;
  invalid.id = braking_trajectory_id;
  publishStatus(invalid, uav_mission_msgs::TrajectoryStatus::INVALID,
                "active path unsafe and no replacement exists; braking");
}

void plannerCallback(const uav_mission_msgs::PlannerStatusConstPtr& msg) {
  if (msg->state == uav_mission_msgs::PlannerStatus::TRAJECTORY_UNSAFE &&
      validContext(msg->context)) startBraking(*msg);
}

void clearForClosedContext(const uav_mission_msgs::TaskContext& context) {
  closed_context = context;
  have_closed_context = true;
  active = TrajectorySlot{};
  pending = TrajectorySlot{};
  braking = false;
}

void missionCallback(const uav_mission_msgs::MissionCommandConstPtr& msg) {
  if (!sameContext(msg->context, authorization)) return;
  if (msg->command == uav_mission_msgs::MissionCommand::STOP ||
      msg->command == uav_mission_msgs::MissionCommand::HOLD_HOME)
    clearForClosedContext(msg->context);
}

void stopCallback(const std_msgs::EmptyConstPtr&) {
  if (authorization.manager_epoch != 0) clearForClosedContext(authorization);
}

void replanCallback(const std_msgs::EmptyConstPtr&) {
  ROS_DEBUG_THROTTLE(1.0, "Replan requested; retaining active trajectory until safe handover");
}

void odomCallback(const nav_msgs::OdometryConstPtr& msg) {
  odom = *msg;
  if (!odom_twist_in_body) return;
  const auto& q = msg->pose.pose.orientation;
  Eigen::Quaterniond rotation(q.w, q.x, q.y, q.z);
  const auto& v = msg->twist.twist.linear;
  if (!rotation.coeffs().allFinite() || rotation.norm() < 1e-6) {
    odom.header.stamp = ros::Time();
    return;
  }
  const Eigen::Vector3d world_velocity = rotation.normalized() * Eigen::Vector3d(v.x, v.y, v.z);
  odom.twist.twist.linear.x = world_velocity.x();
  odom.twist.twist.linear.y = world_velocity.y();
  odom.twist.twist.linear.z = world_velocity.z();
}

void fillCommand(const uav_mission_msgs::TaskContext& context, uint64_t id,
                 const Eigen::Vector3d& position, const Eigen::Vector3d& velocity,
                 const Eigen::Vector3d& acceleration, double yaw, double yaw_rate,
                 const ros::Time& now) {
  command.header.stamp = now;
  command.header.frame_id = world_frame;
  command.context = context;
  command.trajectory_id = static_cast<uint32_t>(id);
  command.trajectory_flag = quadrotor_msgs::PositionCommand::TRAJECTORY_STATUS_READY;
  command.position.x = position.x();
  command.position.y = position.y();
  command.position.z = position.z();
  command.velocity.x = velocity.x();
  command.velocity.y = velocity.y();
  command.velocity.z = velocity.z();
  command.acceleration.x = acceleration.x();
  command.acceleration.y = acceleration.y();
  command.acceleration.z = acceleration.z();
  command.yaw = yaw;
  command.yaw_dot = yaw_rate;
  command_pub.publish(command);
}

void commandCallback(const ros::TimerEvent&) {
  const ros::Time now = ros::Time::now();
  if (pending.valid) {
    const bool return_task = pending.context.task_type == uav_mission_msgs::TaskContext::RETURN_HOME;
    publishStatus(pending, uav_mission_msgs::TrajectoryStatus::PENDING,
                  "waiting for controller takeover confirmation");
    if (return_task && !pending.controller_accepted && now >= pending.start_time) {
      publishStatus(pending, uav_mission_msgs::TrajectoryStatus::REJECTED,
                    "controller did not confirm before scheduled handover");
      pending = TrajectorySlot{};
    } else if (!pending.controller_accepted &&
        (now - pending.received_at).toSec() > handover_timeout) {
      publishStatus(pending, uav_mission_msgs::TrajectoryStatus::REJECTED,
                    "controller handover confirmation timed out");
      pending = TrajectorySlot{};
    } else if (pending.controller_accepted && now >= pending.start_time) {
      if (return_task && (now - pending.start_time).toSec() > handover_margin) {
        publishStatus(pending, uav_mission_msgs::TrajectoryStatus::REJECTED,
                      "controller confirmation arrived after handover expiry");
        pending = TrajectorySlot{};
      } else if (!continuousWithExecution(pending, now)) {
        publishStatus(pending, uav_mission_msgs::TrajectoryStatus::REJECTED,
                      "trajectory is discontinuous at actual activation time");
        pending = TrajectorySlot{};
      } else {
        active = pending;
        pending = TrajectorySlot{};
        braking = false;  // A validated replacement resumes return after braking.
        publishStatus(active, uav_mission_msgs::TrajectoryStatus::ACTIVE,
                      "controller confirmed; continuity rechecked at activation");
      }
    }
  }
  if (braking) {
    Eigen::Vector3d position, velocity, acceleration;
    evaluateBraking(now, position, velocity, acceleration);
    fillCommand(braking_context, braking_trajectory_id, position, velocity, acceleration,
                braking_yaw, 0.0, now);
    return;
  }
  if (!active.valid || now < active.start_time) return;

  Eigen::Vector3d position, velocity, acceleration;
  double yaw = 0.0, yaw_rate = 0.0;
  evaluate(active, now, position, velocity, acceleration, yaw, yaw_rate);
  if (!position.allFinite() || !velocity.allFinite() || !acceleration.allFinite() ||
      !std::isfinite(yaw) || !std::isfinite(yaw_rate)) {
    publishStatus(active, uav_mission_msgs::TrajectoryStatus::INVALID,
                  "non-finite active trajectory state");
    active = TrajectorySlot{};
    return;
  }
  fillCommand(active.context, active.id, position, velocity, acceleration, yaw, yaw_rate, now);
  if ((now - active.start_time).toSec() >= active.duration && !active.completion_reported) {
    active.completion_reported = true;
    publishStatus(active, uav_mission_msgs::TrajectoryStatus::COMPLETED,
                  "trajectory segment completed; endpoint hold remains active");
  }
}
}  // namespace

int main(int argc, char** argv) {
  ros::init(argc, argv, "traj_server");
  ros::NodeHandle node;
  ros::NodeHandle nh("~");

  nh.param("odom_twist_in_body", odom_twist_in_body, false);
  nh.param<std::string>("world_frame", world_frame, "world");
  nh.param<std::string>("source_id", source_id, "fuel_planner");
  nh.param("trajectory_handover/position_tolerance", position_switch_tolerance, 0.75);
  nh.param("trajectory_handover/velocity_tolerance", velocity_switch_tolerance, 1.0);
  nh.param("trajectory_handover/acceleration_tolerance", acceleration_switch_tolerance, 2.0);
  nh.param("trajectory_handover/timeout", handover_timeout, 2.0);
  nh.param("return_home/handover_margin", handover_margin, 0.1);
  nh.param("manager/max_acc", braking_acceleration, 1.0);
  if (position_switch_tolerance <= 0.0 || velocity_switch_tolerance <= 0.0 ||
      acceleration_switch_tolerance <= 0.0 || handover_timeout <= 0.0 ||
      handover_margin < 0.0 || !std::isfinite(handover_margin) ||
      braking_acceleration <= 0.0 || source_id.empty())
    throw std::invalid_argument("invalid trajectory handover/braking parameters");

  auto spline_sub = node.subscribe("planning/bspline", 10, splineCallback);
  auto replan_sub = node.subscribe<std_msgs::Empty>("planning/replan", 10, replanCallback);
  auto stop_sub = node.subscribe<std_msgs::Empty>("/fuel/stop_planner", 10, stopCallback);
  auto odom_sub = node.subscribe<nav_msgs::Odometry>("/odom_world", 50, odomCallback);
  auto authorization_sub = node.subscribe<uav_mission_msgs::TaskContext>(
      "/fuel/authorized_context", 10, authorizationCallback);
  auto ack_sub = node.subscribe<uav_mission_msgs::ControllerAck>(
      "/fuel/controller_ack", 20, ackCallback);
  auto planner_sub = node.subscribe<uav_mission_msgs::PlannerStatus>(
      "/fuel/planner_status", 20, plannerCallback);
  auto mission_sub = node.subscribe<uav_mission_msgs::MissionCommand>(
      "/fuel/mission_command", 20, missionCallback);

  command_pub = node.advertise<quadrotor_msgs::PositionCommand>("/position_cmd", 50);
  status_pub = node.advertise<uav_mission_msgs::TrajectoryStatus>("/fuel/trajectory_status", 20);
  auto command_timer = node.createTimer(ros::Duration(0.01), commandCallback);

  command.kx = {5.7, 5.7, 6.2};
  command.kv = {3.4, 3.4, 4.0};
  ROS_WARN("[Traj server]: active/pending handover ready; normal replans retain active trajectories");
  ros::spin();
  return 0;
}
