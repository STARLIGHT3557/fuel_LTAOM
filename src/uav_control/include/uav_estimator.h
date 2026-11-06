#pragma once
#include <ros/ros.h>
#include <prometheus_msgs/UAVState.h>
#include <nav_msgs/Path.h>
#include <tf2_ros/transform_broadcaster.h>
#include <vector>

// GPS/PX4-only state adapter for the P600 simulation.
class UAV_estimator {
 public:
  explicit UAV_estimator(ros::NodeHandle& nh);
 private:
  void publishState(const ros::TimerEvent&);
  void publishVisuals(const ros::TimerEvent&);
  prometheus_msgs::UAVState state_;
  std::string body_frame_, lidar_frame_;
  std::vector<ros::Subscriber> subscribers_;
  ros::Publisher state_pub_, odom_pub_, mesh_pub_, path_pub_;
  ros::Timer state_timer_, visual_timer_;
  ros::Time pose_time_, velocity_time_, imu_time_, gps_time_, fcu_time_;
  bool fcu_connected_ = false;
  double max_xy_ = 5, max_z_ = 4;
  nav_msgs::Path path_;
  tf2_ros::TransformBroadcaster tf_;
};
