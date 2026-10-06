// 接收控制器任务指令，平滑移动到固定目标，并持续发布定点参考。
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <stdexcept>
#include <string>

#include <Eigen/Core>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <ros/ros.h>

#include "uav_controller/TrajectoryCommand.h"
#include "uav_controller/TrajectorySetpoint.h"
#include "uav_controller/feedback_time.hpp"

namespace uc = uav_controller;

class FixedPointRos1Pub {
public:
  FixedPointRos1Pub() : pnh_("~") {
    pnh_.param("target_x", target_.x(), 0.0);
    pnh_.param("target_y", target_.y(), 0.0);
    pnh_.param("target_z", target_.z(), 1.0);
    pnh_.param("move_speed", move_speed_, 0.3);
    pnh_.param("pub_rate_hz", rate_, 50.0);
    pnh_.param("feedback_timeout", feedback_timeout_, 0.5);
    pnh_.param("command_timeout", command_timeout_, 0.5);
    pnh_.param("timestamp_future_tolerance", future_tolerance_, 0.05);
    if (!target_.allFinite() || !positive(move_speed_) || !positive(rate_) ||
        !positive(feedback_timeout_) || !positive(command_timeout_) ||
        !std::isfinite(future_tolerance_) || future_tolerance_ < 0.0)
      throw std::invalid_argument("invalid fixed-point target/rate/timeout parameters");

    sub_command_ = nh_.subscribe(topic("command_topic", "/uav_controller/trajectory_command"), 5,
                                &FixedPointRos1Pub::onCommand, this);
    sub_pose_ = nh_.subscribe(topic("pose_topic", "/mavros/local_position/pose"), 5,
                             &FixedPointRos1Pub::onPose, this);
    sub_velocity_ = nh_.subscribe(topic("velocity_topic", "/mavros/local_position/velocity_local"), 5,
                                 &FixedPointRos1Pub::onVelocity, this);
    pub_ = nh_.advertise<uc::TrajectorySetpoint>(
        topic("setpoint_topic", "/uav_controller/trajectory_setpoint"), 10);
    timer_ = nh_.createTimer(ros::Duration(1.0 / rate_), &FixedPointRos1Pub::tick, this);
    ROS_INFO("Fixed point: target=[%.2f %.2f %.2f] m, speed<=%.2f m/s; waiting for controller mission",
             target_.x(), target_.y(), target_.z(), move_speed_);
  }

private:
  static bool positive(double value) { return std::isfinite(value) && value > 0.0; }
  bool fresh(const ros::Time &stamp, const ros::Time &now, double timeout) const {
    return uc::freshTimestamp(stamp.toSec(), now.toSec(), timeout, future_tolerance_);
  }
  std::string topic(const std::string &name, const std::string &fallback) {
    std::string result;
    pnh_.param(name, result, fallback);
    return result;
  }
  void onPose(const geometry_msgs::PoseStamped::ConstPtr &msg) {
    const auto &p = msg->pose.position;
    pose_stamp_ = std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z)
                      ? msg->header.stamp : ros::Time();
  }
  void onVelocity(const geometry_msgs::TwistStamped::ConstPtr &msg) {
    const auto &v = msg->twist.linear;
    velocity_stamp_ = std::isfinite(v.x) && std::isfinite(v.y) && std::isfinite(v.z)
                          ? msg->header.stamp : ros::Time();
  }
  void onCommand(const uc::TrajectoryCommand::ConstPtr &msg) {
    const ros::Time now = ros::Time::now();
    if (!fresh(msg->header.stamp, now, command_timeout_) ||
        msg->header.stamp < command_stamp_) return;
    command_stamp_ = msg->header.stamp;
    if (!msg->active) {
      active_ = false;
      return;
    }
    // 同一任务的重复指令不能重启轨迹，中断后必须等待新任务。
    if (have_mission_ && msg->context.manager_epoch == context_.manager_epoch &&
        msg->context.mission_id == context_.mission_id &&
        msg->context.task_type == context_.task_type &&
        msg->context.source_id == context_.source_id) return;
    if (!fresh(pose_stamp_, now, feedback_timeout_) ||
        !fresh(velocity_stamp_, now, feedback_timeout_)) return;

    const Eigen::Vector3d origin(msg->origin.x, msg->origin.y, msg->origin.z);
    if (!origin.allFinite() || !std::isfinite(msg->yaw)) return;
    const double duration = std::max(1.0, 1.875 * (target_ - origin).norm() / move_speed_);
    if (!std::isfinite(duration)) return;
    origin_ = origin;
    yaw_ = msg->yaw;
    duration_ = duration;
    started_at_ = now;
    last_tick_ = now;
    context_ = msg->context;
    trajectory_id_ = msg->trajectory_id == 0 ? 1 : msg->trajectory_id;
    have_mission_ = true;
    active_ = true;
    ROS_INFO("Fixed-point mission %llu started; target=[%.2f %.2f %.2f] m",
             static_cast<unsigned long long>(context_.mission_id), target_.x(), target_.y(), target_.z());
  }
  void tick(const ros::TimerEvent &) {
    if (!active_) return;
    const ros::Time now = ros::Time::now();
    if (now < last_tick_ || (now - last_tick_).toSec() > 0.5 ||
        !fresh(command_stamp_, now, command_timeout_) ||
        !fresh(pose_stamp_, now, feedback_timeout_) ||
        !fresh(velocity_stamp_, now, feedback_timeout_)) {
      active_ = false;
      ROS_WARN("Fixed-point mission interrupted: stale command/feedback or clock discontinuity");
      return;
    }
    last_tick_ = now;

    Eigen::Vector3d position = target_;
    Eigen::Vector3d velocity = Eigen::Vector3d::Zero();
    Eigen::Vector3d acceleration = Eigen::Vector3d::Zero();
    const double elapsed = (now - started_at_).toSec();
    if (elapsed < duration_) {
      // 五次多项式保证起止速度、加速度为零，峰值速度不超过 move_speed。
      const double s = std::clamp(elapsed / duration_, 0.0, 1.0);
      const double s2 = s * s, s3 = s2 * s;
      const Eigen::Vector3d delta = target_ - origin_;
      position = origin_ + delta * s3 * (10.0 - 15.0 * s + 6.0 * s2);
      velocity = delta / duration_ * 30.0 * s2 * (1.0 - s) * (1.0 - s);
      acceleration = delta / (duration_ * duration_) *
                     60.0 * s * (1.0 - s) * (1.0 - 2.0 * s);
    }

    uc::TrajectorySetpoint msg;
    msg.header.stamp = now;
    msg.header.frame_id = "map";
    msg.context = context_;
    msg.trajectory_id = trajectory_id_;
    msg.position.x = position.x();
    msg.position.y = position.y();
    msg.position.z = position.z();
    msg.velocity.x = velocity.x();
    msg.velocity.y = velocity.y();
    msg.velocity.z = velocity.z();
    msg.acceleration.x = acceleration.x();
    msg.acceleration.y = acceleration.y();
    msg.acceleration.z = acceleration.z();
    msg.yaw = yaw_;
    // 持续发布目标，保留轨迹超时保护，直到操作者中断任务。
    msg.finished = false;
    pub_.publish(msg);
  }

  ros::NodeHandle nh_, pnh_;
  ros::Subscriber sub_command_, sub_pose_, sub_velocity_;
  ros::Publisher pub_;
  ros::Timer timer_;
  Eigen::Vector3d target_{0.0, 0.0, 1.0};
  Eigen::Vector3d origin_{Eigen::Vector3d::Zero()};
  ros::Time pose_stamp_, velocity_stamp_, command_stamp_, last_tick_, started_at_;
  uav_mission_msgs::TaskContext context_;
  std::uint64_t trajectory_id_{0};
  double move_speed_{0.3}, rate_{50.0}, feedback_timeout_{0.5}, command_timeout_{0.5};
  double future_tolerance_{0.05}, yaw_{0.0}, duration_{1.0};
  bool active_{false}, have_mission_{false};
};

int main(int argc, char **argv) {
  ros::init(argc, argv, "fixed_point_pub");
  try {
    FixedPointRos1Pub node;
    ros::spin();
  } catch (const std::exception &error) {
    ROS_FATAL("Fixed-point startup failed: %s", error.what());
    return 1;
  }
  return 0;
}
