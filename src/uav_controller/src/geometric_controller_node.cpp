// ROS adapter for the reusable flight controller. All flight decisions and
// reference selection live in FlightController; this file owns ROS I/O only.
#include <cmath>
#include <memory>
#include <stdexcept>
#include <string>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <mavros_msgs/AttitudeTarget.h>
#include <mavros_msgs/ExtendedState.h>
#include <mavros_msgs/PositionTarget.h>
#include <mavros_msgs/State.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/String.h>
#include <uav_mission_msgs/ControllerAck.h>
#include <uav_mission_msgs/TaskContext.h>
#include <uav_mission_msgs/TrajectoryStatus.h>

#include "uav_controller/TrajectoryCommand.h"
#include "uav_controller/TrajectorySetpoint.h"
#include "uav_controller/FlightStatus.h"
#include "uav_controller/flight_controller.hpp"
#include "uav_controller/feedback_time.hpp"

namespace uc = uav_controller;

class GeometricControllerRos1Node {
public:
  GeometricControllerRos1Node() : pnh_("~") {
    uc::ControllerParams control;
    loadVector("k_pos", Eigen::Vector3d(3.0, 3.0, 4.0), control.k_pos);
    loadVector("k_vel", Eigen::Vector3d(3.0, 3.0, 3.5), control.k_vel);
    pnh_.param("hover_percentage", control.hover_percentage, 0.65);
    pnh_.param("max_a_fb", control.max_a_fb, 8.0);
    pnh_.param("gravity", control.gravity, 9.81);
    uc::FlightParams flight;
    pnh_.param("fsm/position_step", flight.position_step, flight.position_step);
    pnh_.param("fsm/direct_trajectory", flight.direct_trajectory, flight.direct_trajectory);
    direct_trajectory_ = flight.direct_trajectory;
    loadVector("fsm/step_target", flight.step_target, flight.step_target);
    pnh_.param("fsm/takeoff_height", flight.takeoff_height, flight.takeoff_height);
    pnh_.param("fsm/takeoff_speed", flight.takeoff_speed, flight.takeoff_speed);
    pnh_.param("fsm/hover_before", flight.hover_before, flight.hover_before);
    pnh_.param("fsm/position_tolerance", flight.position_tolerance, flight.position_tolerance);
    pnh_.param("fsm/speed_tolerance", flight.speed_tolerance, flight.speed_tolerance);
    pnh_.param("fsm/stable_time", flight.stable_time, flight.stable_time);
    pnh_.param("fsm/trajectory_timeout", flight.trajectory_timeout, flight.trajectory_timeout);
    pnh_.param("fsm/takeoff_timeout", flight.takeoff_timeout, flight.takeoff_timeout);
    pnh_.param("fsm/hover_timeout", flight.hover_timeout, flight.hover_timeout);
    pnh_.param("fsm/feedback_recovery_time", flight.feedback_recovery_time, flight.feedback_recovery_time);
    pnh_.param("timestamp_future_tolerance", future_tolerance_, 0.05);
    flight.timestamp_future_tolerance = future_tolerance_;
    pnh_.param("estimate_thrust", flight.estimate_thrust, false);
    pnh_.param("control_rate_hz", control_rate_, 100.0);
    pnh_.param("feedback_timeout", feedback_timeout_, 0.5);
    pnh_.param("state_timeout", state_timeout_, 2.0);
    pnh_.param("print_tracking", print_tracking_, true);
    pnh_.param("print_period_ms", print_period_ms_, 500);
    pnh_.param("feedback_guard/max_speed", pose_jump_max_speed_, 8.0);
    pnh_.param("feedback_guard/margin", pose_jump_margin_, 0.5);
    pnh_.param("feedback_guard/accept_after", pose_jump_accept_after_, 2.0);
    if (!positive(control_rate_) || !positive(feedback_timeout_) ||
        !positive(state_timeout_) || print_period_ms_ <= 0 ||
        pose_jump_max_speed_ < 0.0 || pose_jump_margin_ < 0.0 ||
        pose_jump_accept_after_ < 0.0)
      throw std::invalid_argument("invalid ROS rate/timeout parameters");
    controller_ = std::make_unique<uc::FlightController>(control, flight,
                                                        ros::WallTime::now().toNSec());
    input_.state.az_specific_up = control.gravity;

    sub_pose_ = nh_.subscribe(topic("pose_topic", "/mavros/local_position/pose"), 5,
                             &GeometricControllerRos1Node::onPose, this);
    sub_velocity_ = nh_.subscribe(topic("velocity_topic", "/mavros/local_position/velocity_local"), 5,
                                 &GeometricControllerRos1Node::onVelocity, this);
    sub_imu_ = nh_.subscribe(topic("imu_topic", "/mavros/imu/data"), 5,
                            &GeometricControllerRos1Node::onImu, this);
    sub_state_ = nh_.subscribe(topic("state_topic", "/mavros/state"), 5,
                              &GeometricControllerRos1Node::onState, this);
    sub_extended_ = nh_.subscribe(topic("extended_state_topic", "/mavros/extended_state"), 5,
                                 &GeometricControllerRos1Node::onExtendedState, this);
    sub_trajectory_ = nh_.subscribe(topic("setpoint_topic", "/uav_controller/trajectory_setpoint"), 5,
                                   &GeometricControllerRos1Node::onTrajectory, this);
    sub_authorization_ = nh_.subscribe(
        topic("authorization_topic", "/fuel/authorized_context"), 5,
        &GeometricControllerRos1Node::onAuthorization, this);
    sub_trajectory_status_ = nh_.subscribe(
        topic("trajectory_status_topic", "/fuel/trajectory_status"), 10,
        &GeometricControllerRos1Node::onTrajectoryStatus, this);
    pub_attitude_ = nh_.advertise<mavros_msgs::AttitudeTarget>(
        topic("attitude_setpoint_topic", "/mavros/setpoint_raw/attitude"), 10);
    pub_reference_ = nh_.advertise<mavros_msgs::PositionTarget>(
        topic("active_setpoint_topic", "/uav_controller/active_setpoint"), 10);
    pub_command_ = nh_.advertise<uc::TrajectoryCommand>(
        topic("trajectory_command_topic", "/uav_controller/trajectory_command"), 1, true);
    pub_phase_ = nh_.advertise<std_msgs::String>(
        topic("fsm_state_topic", "/uav_controller/fsm_state"), 1, true);
    pub_status_ = nh_.advertise<uc::FlightStatus>(
        topic("fsm_status_topic", "/uav_controller/fsm_status"), 1, true);
    pub_controller_ack_ = nh_.advertise<uav_mission_msgs::ControllerAck>(
        topic("controller_ack_topic", "/fuel/controller_ack"), 10);

    timer_ = nh_.createTimer(ros::Duration(1.0/control_rate_),
                             &GeometricControllerRos1Node::controlLoop, this);
    if (flight.direct_trajectory) {
      ROS_INFO("Direct trajectory mode: hold on airborne OFFBOARD entry and accept fresh external setpoints");
    } else if (flight.position_step) {
      ROS_INFO("Position step: target=[%.2f %.2f %.2f] m ENU, yaw=0 rad, rate %.0f Hz; "
               "manual takeoff, then switch to OFFBOARD to apply the target immediately",
               flight.step_target.x(), flight.step_target.y(), flight.step_target.z(), control_rate_);
      ROS_INFO("Waiting for armed + OFFBOARD and fresh MAVROS IN_AIR confirmation; "
               "exit/re-enter OFFBOARD to repeat the step");
    } else {
      ROS_INFO("FSM controller: absolute takeoff z=%.2f m, hover %.1f s, rate %.0f Hz; "
               "trajectory completion holds the endpoint for operator takeover",
               flight.takeoff_height, flight.hover_before, control_rate_);
      ROS_INFO("Waiting for manual ARM + OFFBOARD and fresh MAVROS ground confirmation");
    }
  }

private:
  static bool positive(double v) { return std::isfinite(v) && v > 0.0; }
  bool fresh(const ros::Time &stamp, const ros::Time &now, double timeout) const {
    return uc::freshTimestamp(stamp.toSec(), now.toSec(), timeout, future_tolerance_);
  }
  std::string topic(const std::string &name, const std::string &fallback) {
    std::string result;
    pnh_.param(name, result, fallback);
    return result;
  }
  void loadVector(const std::string &name, const Eigen::Vector3d &defaults, Eigen::Vector3d &out) {
    pnh_.param(name + "/x", out.x(), defaults.x());
    pnh_.param(name + "/y", out.y(), defaults.y());
    pnh_.param(name + "/z", out.z(), defaults.z());
  }
  void onPose(const geometry_msgs::PoseStamped::ConstPtr &msg) {
    const auto &p = msg->pose.position;
    const auto &q = msg->pose.orientation;
    Eigen::Vector3d pos(p.x, p.y, p.z);
    Eigen::Quaterniond quat(q.w, q.x, q.y, q.z);
    if (!pos.allFinite() || !quat.coeffs().allFinite() || quat.norm() < 1e-6) {
      pose_stamp_ = ros::Time();
      return;
    }
    // A SLAM estimate can jump or diverge. Feeding such a step straight into a
    // 100 Hz attitude loop saturates the command and flies the vehicle away, so
    // implausible steps hold the previous pose until the source recovers. A
    // jump that persists is a real re-anchor (e.g. multi-session relocalization)
    // and is accepted after pose_jump_accept_after.
    if (have_pose_ && pose_jump_max_speed_ > 0.0) {
      const double dt = (msg->header.stamp - last_pose_stamp_).toSec();
      const double step = (pos - last_pose_pos_).norm();
      if (dt > 0.0 && dt < 1.0 && step > pose_jump_max_speed_ * dt + pose_jump_margin_) {
        if (pose_jump_since_.isZero()) pose_jump_since_ = ros::Time::now();
        const double held = (ros::Time::now() - pose_jump_since_).toSec();
        if (held < pose_jump_accept_after_) {
          ROS_WARN_THROTTLE(1.0,
              "Rejecting implausible pose step %.2f m in %.3f s (limit %.1f m/s); "
              "holding previous feedback for %.1f s",
              step, dt, pose_jump_max_speed_, held);
          return;
        }
        ROS_WARN("Accepting persistent pose re-anchor of %.2f m; feedback frame changed", step);
        pose_jump_since_ = ros::Time();
      } else if (!pose_jump_since_.isZero() &&
                 (ros::Time::now() - pose_jump_since_).toSec() > 0.1) {
        pose_jump_since_ = ros::Time();
      }
    }
    have_pose_ = true;
    last_pose_pos_ = pos;
    last_pose_stamp_ = msg->header.stamp;
    input_.state.pos = pos;
    input_.state.quat = quat.normalized();
    pose_stamp_ = msg->header.stamp;
  }
  void onVelocity(const geometry_msgs::TwistStamped::ConstPtr &msg) {
    const auto &v = msg->twist.linear;
    input_.state.vel << v.x, v.y, v.z;
    velocity_stamp_ = input_.state.vel.allFinite() ? msg->header.stamp : ros::Time();
  }
  void onImu(const sensor_msgs::Imu::ConstPtr &msg) {
    const auto &a = msg->linear_acceleration;
    const Eigen::Vector3d accel(a.x, a.y, a.z);
    input_.state.az_specific_up = (input_.state.quat * accel).z();
    imu_stamp_ = accel.allFinite() ? msg->header.stamp : ros::Time();
  }
  void onState(const mavros_msgs::State::ConstPtr &msg) {
    if (input_.armed && !msg->armed)
      ROS_INFO("FCU reports disarmed (armed=false), mode=%s", msg->mode.c_str());
    input_.armed = msg->armed;
    input_.offboard = msg->mode == "OFFBOARD";
    connected_ = msg->connected;
    state_stamp_ = msg->header.stamp;
  }
  void onExtendedState(const mavros_msgs::ExtendedState::ConstPtr &msg) {
    landed_state_ = msg->landed_state;
    switch (msg->landed_state) {
      case mavros_msgs::ExtendedState::LANDED_STATE_ON_GROUND:
        observed_landing_state_ = uc::LandingState::ON_GROUND; break;
      case mavros_msgs::ExtendedState::LANDED_STATE_IN_AIR:
        observed_landing_state_ = uc::LandingState::IN_AIR; break;
      case mavros_msgs::ExtendedState::LANDED_STATE_TAKEOFF:
        observed_landing_state_ = uc::LandingState::TAKING_OFF; break;
      case mavros_msgs::ExtendedState::LANDED_STATE_LANDING:
        observed_landing_state_ = uc::LandingState::LANDING; break;
      default: observed_landing_state_ = uc::LandingState::UNKNOWN; break;
    }
    extended_stamp_ = msg->header.stamp;
  }
  bool authorized(const uav_mission_msgs::TaskContext& context) const {
    const auto& auth = input_.authorization;
    return auth.valid && context.manager_epoch == auth.manager_epoch &&
           context.mission_id == auth.mission_id && context.task_type == auth.task_type &&
           context.source_id == auth.source_id;
  }
  void onAuthorization(const uav_mission_msgs::TaskContext::ConstPtr& msg) {
    auto& auth = input_.authorization;
    auth.manager_epoch = msg->manager_epoch;
    auth.mission_id = msg->mission_id;
    auth.task_type = msg->task_type;
    auth.source_id = msg->source_id;
    auth.valid = msg->manager_epoch != 0 && msg->mission_id != 0 &&
                 msg->task_type != uav_mission_msgs::TaskContext::TASK_NONE &&
                 !msg->source_id.empty();
  }
  void onTrajectoryStatus(const uav_mission_msgs::TrajectoryStatus::ConstPtr& msg) {
    if (msg->state != uav_mission_msgs::TrajectoryStatus::PENDING) return;
    uav_mission_msgs::ControllerAck ack;
    ack.header.stamp = ros::Time::now();
    ack.header.frame_id = msg->header.frame_id;
    ack.context = msg->context;
    ack.trajectory_id = msg->trajectory_id;
    ack.accepted = controller_ready_ && authorized(msg->context) && msg->trajectory_id != 0;
    ack.detail = ack.accepted ? "authorized trajectory handover accepted"
                              : "controller not ready or task is unauthorized";
    pub_controller_ack_.publish(ack);
  }
  void onTrajectory(const uc::TrajectorySetpoint::ConstPtr &msg) {
    // FUEL/direct mode is a manager-owned execution path and therefore requires
    // the complete authorization tuple.  The legacy takeoff/hover pipeline owns
    // its mission id inside FlightController and must remain usable without a
    // mission manager (for example with fixed_point_pub).
    const bool admissible = direct_trajectory_
        ? authorized(msg->context) && msg->trajectory_id != 0
        : msg->context.mission_id != 0;
    if (!admissible) {
      ROS_WARN_THROTTLE(1.0, "Rejected unauthorized trajectory setpoint");
      return;
    }
    auto &point = input_.trajectory;
    point.manager_epoch = msg->context.manager_epoch;
    point.mission_id = msg->context.mission_id;
    point.trajectory_id = msg->trajectory_id;
    point.task_type = msg->context.task_type;
    point.source_id = msg->context.source_id;
    point.reference.pos << msg->position.x, msg->position.y, msg->position.z;
    point.reference.vel << msg->velocity.x, msg->velocity.y, msg->velocity.z;
    point.reference.acc << msg->acceleration.x, msg->acceleration.y, msg->acceleration.z;
    point.reference.yaw = msg->yaw;
    point.stamp = msg->header.stamp.toSec();
    point.finished = msg->finished;
    point.valid = !msg->header.stamp.isZero();
  }
  void publishSetpoints(const uc::FlightOutput &out, const ros::Time &now) {
    mavros_msgs::AttitudeTarget attitude;
    attitude.header.stamp = now;
    attitude.type_mask = mavros_msgs::AttitudeTarget::IGNORE_ROLL_RATE |
                          mavros_msgs::AttitudeTarget::IGNORE_PITCH_RATE |
                          mavros_msgs::AttitudeTarget::IGNORE_YAW_RATE;
    attitude.orientation.w = out.command.q.w();
    attitude.orientation.x = out.command.q.x();
    attitude.orientation.y = out.command.q.y();
    attitude.orientation.z = out.command.q.z();
    attitude.thrust = out.command.thrust;
    pub_attitude_.publish(attitude);

    mavros_msgs::PositionTarget reference;
    reference.header.stamp = now;
    reference.header.frame_id = "map";
    reference.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
    reference.type_mask = mavros_msgs::PositionTarget::IGNORE_YAW_RATE;
    reference.position.x = out.reference.pos.x();
    reference.position.y = out.reference.pos.y();
    reference.position.z = out.reference.pos.z();
    reference.velocity.x = out.reference.vel.x();
    reference.velocity.y = out.reference.vel.y();
    reference.velocity.z = out.reference.vel.z();
    reference.acceleration_or_force.x = out.reference.acc.x();
    reference.acceleration_or_force.y = out.reference.acc.y();
    reference.acceleration_or_force.z = out.reference.acc.z();
    reference.yaw = out.reference.yaw;
    pub_reference_.publish(reference);
  }
  void controlLoop(const ros::TimerEvent &) {
    const ros::Time now = ros::Time::now();
    const bool pose_ok = fresh(pose_stamp_, now, feedback_timeout_);
    const bool velocity_ok = fresh(velocity_stamp_, now, feedback_timeout_);
    const bool state_ok = fresh(state_stamp_, now, state_timeout_);
    input_.feedback_valid = connected_ && pose_ok && velocity_ok && state_ok;
    input_.landing_state = fresh(extended_stamp_, now, state_timeout_)
                               ? observed_landing_state_ : uc::LandingState::UNKNOWN;
    input_.imu_valid = fresh(imu_stamp_, now, feedback_timeout_);
    if (!input_.feedback_valid) {
      ROS_WARN_THROTTLE(1.0,
          "Feedback invalid: connected=%d pose_ok=%d age=%.4fs velocity_ok=%d age=%.4fs "
          "state_ok=%d age=%.4fs (age<0: future stamp; stamp=0: missing/invalid; "
          "limits pose/velocity=%.2fs state=%.2fs future=%.3fs)",
          connected_, pose_ok, (now-pose_stamp_).toSec(), velocity_ok,
          (now-velocity_stamp_).toSec(), state_ok, (now-state_stamp_).toSec(),
          feedback_timeout_, state_timeout_, future_tolerance_);
    }
    const uc::FlightOutput out = controller_->update(input_, now.toSec());
    controller_ready_ = out.feedback_ready && out.output_policy == uc::OutputPolicy::CLOSED_LOOP &&
                        (out.phase == uc::FlightPhase::HOLD || out.phase == uc::FlightPhase::TRAJECTORY);
    const bool changed = !have_phase_ || out.transition.sequence != last_transition_;
    if (changed) {
      ROS_INFO("FSM: %s -> %s reason=%s mission=%llu transition=%llu output=%s",
               have_phase_ ? uc::phaseName(out.transition.from) : "INIT", uc::phaseName(out.phase),
               uc::reasonName(out.transition.reason), static_cast<unsigned long long>(out.mission_id),
               static_cast<unsigned long long>(out.transition.sequence), uc::policyName(out.output_policy));
      std_msgs::String status;
      status.data = uc::phaseName(out.phase);
      pub_phase_.publish(status);
      last_transition_ = out.transition.sequence;
      have_phase_ = true;
    }
    const bool policy_changed = out.output_policy != last_policy_;
    if (changed || policy_changed || now < command_stamp_ || (now - command_stamp_).toSec() >= 0.1) {
      uc::FlightStatus status;
      status.header.stamp = now;
      status.mode = uc::modeName(out.mode);
      status.mission_stage = uc::stageName(out.mission_stage);
      status.phase = uc::phaseName(out.phase);
      status.previous_phase = uc::phaseName(out.transition.from);
      status.transition_reason = uc::reasonName(out.transition.reason);
      status.transition_id = out.transition.sequence;
      status.context.manager_epoch = out.manager_epoch;
      status.context.mission_id = out.mission_id;
      status.context.task_type = out.task_type;
      status.context.source_id = out.source_id;
      status.trajectory_id = out.trajectory_id;
      status.output_policy = uc::policyName(out.output_policy);
      status.feedback_ready = out.feedback_ready;
      status.landing_state = uc::landingStateName(input_.landing_state);
      status.phase_elapsed = out.phase_elapsed;
      pub_status_.publish(status);
      uc::TrajectoryCommand command;
      command.header.stamp = now;
      command.header.frame_id = "map";
      command.context.manager_epoch = out.manager_epoch;
      command.context.mission_id = out.mission_id;
      command.context.task_type = out.task_type;
      command.context.source_id = out.source_id;
      command.trajectory_id = out.trajectory_id;
      command.active = out.trajectory_active;
      command.origin.x = out.trajectory_origin.pos.x();
      command.origin.y = out.trajectory_origin.pos.y();
      command.origin.z = out.trajectory_origin.pos.z();
      command.yaw = out.trajectory_origin.yaw;
      pub_command_.publish(command);
      command_stamp_ = now;
    }
    last_policy_ = out.output_policy;
    if (!out.publish) {
      ROS_WARN_THROTTLE(2.0, "No attitude output: phase=%s feedback_ready=%d landing_state=%s reason=%s",
                        uc::phaseName(out.phase), out.feedback_ready,
                        uc::landingStateName(input_.landing_state), uc::reasonName(out.transition.reason));
      return;
    }
    publishSetpoints(out, now);
    if (print_tracking_) {
      const Eigen::Vector3d error = out.reference.pos - input_.state.pos;
      ROS_INFO_THROTTLE(print_period_ms_/1000.0,
                        "[%s] error=[%+.2f %+.2f %+.2f] thrust=%.3f raw=%.3f "
                        "armed=%d landed_state=%u landing_state=%s output=%s",
                        uc::phaseName(out.phase), error.x(), error.y(), error.z(),
                        out.command.thrust, out.command.thrust_raw, input_.armed,
                        static_cast<unsigned>(landed_state_), uc::landingStateName(input_.landing_state),
                        uc::policyName(out.output_policy));
    }
  }

  ros::NodeHandle nh_, pnh_;
  ros::Subscriber sub_pose_, sub_velocity_, sub_imu_, sub_state_, sub_extended_, sub_trajectory_,
      sub_authorization_, sub_trajectory_status_;
  ros::Publisher pub_attitude_, pub_reference_, pub_command_, pub_phase_, pub_status_, pub_controller_ack_;
  ros::Timer timer_;
  std::unique_ptr<uc::FlightController> controller_;
  uc::FlightInput input_;
  ros::Time pose_stamp_, velocity_stamp_, imu_stamp_, state_stamp_, extended_stamp_, command_stamp_;
  std::uint64_t last_transition_{0};
  uc::OutputPolicy last_policy_{uc::OutputPolicy::NO_OUTPUT};
  uc::LandingState observed_landing_state_{uc::LandingState::UNKNOWN};
  double control_rate_{100.0}, feedback_timeout_{0.5}, state_timeout_{2.0};
  double future_tolerance_{0.05};
  Eigen::Vector3d last_pose_pos_{Eigen::Vector3d::Zero()};
  ros::Time last_pose_stamp_, pose_jump_since_;
  bool have_pose_{false};
  double pose_jump_max_speed_{8.0}, pose_jump_margin_{0.5}, pose_jump_accept_after_{2.0};
  uint8_t landed_state_{mavros_msgs::ExtendedState::LANDED_STATE_UNDEFINED};
  int print_period_ms_{500};
  bool print_tracking_{true}, connected_{false}, have_phase_{false}, controller_ready_{false};
  bool direct_trajectory_{false};
};

int main(int argc, char **argv) {
  ros::init(argc, argv, "geometric_controller_node");
  try {
    GeometricControllerRos1Node node;
    ros::spin();
  } catch (const std::exception &error) {
    ROS_FATAL("Controller startup failed: %s", error.what());
    return 1;
  }
  return 0;
}
