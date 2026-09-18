#include "uav_estimator.h"
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/GPSRAW.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/BatteryState.h>
#include <sensor_msgs/NavSatFix.h>
#include <std_msgs/Float64.h>
#include <nav_msgs/Odometry.h>
#include <visualization_msgs/Marker.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <cmath>

UAV_estimator::UAV_estimator(ros::NodeHandle& nh) {
  int id;
  nh.param("uav_id", id, 1);
  nh.param("control/maximum_safe_vel_xy", max_xy_, 5.0);
  nh.param("control/maximum_safe_vel_z", max_z_, 4.0);
  state_.uav_id = id;
  state_.location_source = prometheus_msgs::UAVState::GPS;
  state_.attitude_q.w = 1;
  const std::string ns = "/uav" + std::to_string(id);
  body_frame_ = "uav" + std::to_string(id) + "/base_link";
  lidar_frame_ = "uav" + std::to_string(id) + "/lidar_link";
  subscribers_.push_back(nh.subscribe<mavros_msgs::State>(ns + "/mavros/state", 1,
      [this](const mavros_msgs::State::ConstPtr& msg) {
        fcu_connected_ = msg->connected; state_.armed = msg->armed; state_.mode = msg->mode;
        fcu_time_ = ros::Time::now();
      }));
  subscribers_.push_back(nh.subscribe<geometry_msgs::PoseStamped>(ns + "/mavros/local_position/pose", 1,
      [this](const geometry_msgs::PoseStamped::ConstPtr& msg) {
        const auto& p = msg->pose.position;
        if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) return;
        state_.position[0] = p.x; state_.position[1] = p.y; state_.position[2] = p.z;
        pose_time_ = ros::Time::now();
      }));
  subscribers_.push_back(nh.subscribe<geometry_msgs::TwistStamped>(ns + "/mavros/local_position/velocity_local", 1,
      [this](const geometry_msgs::TwistStamped::ConstPtr& msg) {
        const auto& v = msg->twist.linear;
        if (!std::isfinite(v.x) || !std::isfinite(v.y) || !std::isfinite(v.z)) return;
        state_.velocity[0] = v.x; state_.velocity[1] = v.y; state_.velocity[2] = v.z;
        velocity_time_ = ros::Time::now();
      }));
  subscribers_.push_back(nh.subscribe<sensor_msgs::Imu>(ns + "/mavros/imu/data", 1,
      [this](const sensor_msgs::Imu::ConstPtr& msg) {
        const auto& q = msg->orientation;
        const auto& a = msg->linear_acceleration;
        const double norm = q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w;
        if (!std::isfinite(norm) || std::abs(norm - 1) > 0.02 ||
            !std::isfinite(a.x) || !std::isfinite(a.y) || !std::isfinite(a.z)) return;
        state_.attitude_q = q;
        double roll, pitch, yaw;
        tf2::Matrix3x3(tf2::Quaternion(q.x, q.y, q.z, q.w)).getRPY(roll, pitch, yaw);
        state_.attitude[0] = roll; state_.attitude[1] = pitch; state_.attitude[2] = yaw;
        state_.attitude_rate[0] = msg->angular_velocity.x;
        state_.attitude_rate[1] = msg->angular_velocity.y;
        state_.attitude_rate[2] = msg->angular_velocity.z;
        state_.a[0] = a.x; state_.a[1] = a.y; state_.a[2] = a.z;
        imu_time_ = ros::Time::now();
      }));
  subscribers_.push_back(nh.subscribe<mavros_msgs::GPSRAW>(ns + "/mavros/gpsstatus/gps1/raw", 1,
      [this](const mavros_msgs::GPSRAW::ConstPtr& msg) {
        state_.gps_status = msg->fix_type; state_.gps_num = msg->satellites_visible;
        gps_time_ = ros::Time::now();
      }));
  subscribers_.push_back(nh.subscribe<sensor_msgs::NavSatFix>(ns + "/mavros/global_position/global", 1,
      [this](const sensor_msgs::NavSatFix::ConstPtr& msg) {
        state_.latitude = msg->latitude; state_.longitude = msg->longitude; state_.altitude = msg->altitude;
      }));
  subscribers_.push_back(nh.subscribe<std_msgs::Float64>(ns + "/mavros/global_position/rel_alt", 1,
      [this](const std_msgs::Float64::ConstPtr& msg) { state_.rel_alt = msg->data; }));
  subscribers_.push_back(nh.subscribe<sensor_msgs::BatteryState>(ns + "/mavros/battery", 1,
      [this](const sensor_msgs::BatteryState::ConstPtr& msg) {
        state_.battery_state = msg->voltage; state_.battery_percetage = msg->percentage;
      }));
  state_pub_ = nh.advertise<prometheus_msgs::UAVState>(ns + "/prometheus/state", 1);
  odom_pub_ = nh.advertise<nav_msgs::Odometry>(ns + "/prometheus/odom", 1);
  mesh_pub_ = nh.advertise<visualization_msgs::Marker>(ns + "/prometheus/uav_mesh", 1);
  path_pub_ = nh.advertise<nav_msgs::Path>(ns + "/prometheus/trajectory", 1);
  state_timer_ = nh.createTimer(ros::Duration(0.02), &UAV_estimator::publishState, this);
  visual_timer_ = nh.createTimer(ros::Duration(0.05), &UAV_estimator::publishVisuals, this);
}

void UAV_estimator::publishState(const ros::TimerEvent&) {
  const auto now = ros::Time::now();
  const auto fresh = [&now](const ros::Time& stamp, double limit) {
    return !stamp.isZero() && now >= stamp && (now - stamp).toSec() <= limit;
  };
  state_.connected = fcu_connected_ && fresh(fcu_time_, 3.0);
  state_.odom_valid = state_.connected && fresh(pose_time_, 0.5) && fresh(velocity_time_, 0.5) &&
      fresh(imu_time_, 0.5) && fresh(gps_time_, 1.0) && state_.gps_status >= 3 &&
      std::hypot(state_.velocity[0], state_.velocity[1]) <= max_xy_ && std::abs(state_.velocity[2]) <= max_z_;
  state_.header.stamp = now;
  state_.header.frame_id = "world";
  state_pub_.publish(state_);
  if (!state_.odom_valid) return;
  nav_msgs::Odometry odom;
  odom.header = state_.header;
  odom.child_frame_id = body_frame_;
  odom.pose.pose.position.x = state_.position[0];
  odom.pose.pose.position.y = state_.position[1];
  odom.pose.pose.position.z = std::max(0.01f, state_.position[2]);
  odom.pose.pose.orientation = state_.attitude_q;
  odom.twist.twist.linear.x = state_.velocity[0];
  odom.twist.twist.linear.y = state_.velocity[1];
  odom.twist.twist.linear.z = state_.velocity[2];
  odom_pub_.publish(odom);
  geometry_msgs::TransformStamped body;
  body.header = odom.header; body.child_frame_id = body_frame_;
  body.transform.translation.x = odom.pose.pose.position.x;
  body.transform.translation.y = odom.pose.pose.position.y;
  body.transform.translation.z = odom.pose.pose.position.z;
  body.transform.rotation = state_.attitude_q;
  tf_.sendTransform(body);
  // Ray origin = body mounting (0.13, 0, 0.23) + pitched sensor offset (0, 0, 0.05).
  geometry_msgs::TransformStamped lidar;
  lidar.header.stamp = now; lidar.header.frame_id = body_frame_; lidar.child_frame_id = lidar_frame_;
  lidar.transform.translation.x = 0.13 + 0.05 * std::sin(0.35);
  lidar.transform.translation.z = 0.23 + 0.05 * std::cos(0.35);
  lidar.transform.rotation.y = std::sin(0.35 / 2);
  lidar.transform.rotation.w = std::cos(0.35 / 2);
  tf_.sendTransform(lidar);
}

void UAV_estimator::publishVisuals(const ros::TimerEvent&) {
  if (!state_.odom_valid) return;
  visualization_msgs::Marker mesh;
  mesh.header = state_.header;
  mesh.ns = "mesh"; mesh.id = 0;
  mesh.type = visualization_msgs::Marker::MESH_RESOURCE;
  mesh.action = visualization_msgs::Marker::ADD;
  mesh.pose.position.x = state_.position[0]; mesh.pose.position.y = state_.position[1]; mesh.pose.position.z = state_.position[2];
  mesh.pose.orientation = state_.attitude_q;
  mesh.scale.x = mesh.scale.y = mesh.scale.z = 1;
  mesh.color.a = 1; mesh.color.b = 1;
  mesh.mesh_resource = "package://prometheus_uav_control/meshes/hummingbird.mesh";
  mesh_pub_.publish(mesh);
  geometry_msgs::PoseStamped pose;
  pose.header = mesh.header; pose.pose = mesh.pose;
  path_.header = pose.header;
  path_.poses.push_back(pose);
  if (path_.poses.size() > 1000) path_.poses.erase(path_.poses.begin());
  path_pub_.publish(path_);
}
