#pragma once
#include <ros/ros.h>
#include <Eigen/Eigen>
#include <queue>
#include <mavros_msgs/PositionTarget.h>
#include <mavros_msgs/AttitudeTarget.h>
#include <mavros_msgs/SetMode.h>
#include <prometheus_msgs/UAVCommand.h>
#include <prometheus_msgs/UAVState.h>
#include <prometheus_msgs/UAVControlState.h>

// P600 simulation controller: XYZ position takeoff/hold and FUEL mode 9 only.
class UAV_controller {
 public:
  UAV_controller(ros::NodeHandle& nh, const Eigen::Vector3d& kp, const Eigen::Vector3d& kv);
  void mainloop();
 private:
  void commandCallback(const prometheus_msgs::UAVCommand::ConstPtr& msg);
  void stateCallback(const prometheus_msgs::UAVState::ConstPtr& msg);
  void hold();
  void send_pos_setpoint(const Eigen::Vector3d& pos_sp, float yaw_sp);
  void send_att_thrust_setpoint(const Eigen::Vector3d& pos_sp, const Eigen::Vector3d& vel_sp,
      const Eigen::Vector3d& acc_des, float yaw_sp, const Eigen::Vector3d& uav_pos,
      const Eigen::Vector3d& uav_vel, const float& uav_yaw, const Eigen::Quaterniond& uav_quat);
  Eigen::Matrix3d quat2RotMatrix(const Eigen::Vector4f& q);
  Eigen::Vector4f rot2Quaternion(const Eigen::Matrix3d& rot);
  bool estimateThrustModel(const Eigen::Vector3d& acceleration);
  void resetThrustMapping();
  ros::Subscriber command_sub_, state_sub_;
  ros::Publisher px4_setpoint_raw_local_pub, px4_setpoint_raw_attitude_pub, control_pub_;
  ros::ServiceClient mode_client_;
  prometheus_msgs::UAVState uav_state;
  prometheus_msgs::UAVCommand command_;
  Eigen::Vector3d k_pos_, k_vel_, position_ = Eigen::Vector3d::Zero(), velocity_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d hold_position_ = Eigen::Vector3d::Zero(), fence_min_, fence_max_;
  Eigen::Quaterniond attitude_ = Eigen::Quaterniond::Identity();
  double yaw_ = 0, hold_yaw_ = 0, state_timeout_ = 0.5, command_timeout_ = 0.5;
  bool have_state_ = false, have_hold_ = false, landing_ = false, failsafe_ = false;
  bool was_offboard_ = false;
  int uav_id_ = 1;
  ros::Time state_time_, command_time_, mode_request_time_;
  // Original controller thrust mapping and RLS tuning.
  std::queue<std::pair<ros::Time, double>> timed_thrust_;
  const double rho2_ = 0.998;
  double hover_percentage = 0.258, thr2acc_ = 9.81 / 0.258, P_ = 1e6;
  bool estimateThrustStopFlag = true;
};
