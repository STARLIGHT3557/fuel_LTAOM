#include <cmath>
#include <memory>
#include <stdexcept>
#include <Eigen/Geometry>
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <uav_controller/FlightStatus.h>
#include <uav_controller/TrajectorySetpoint.h>
#include <quadrotor_msgs/PositionCommand.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Empty.h>
#include <std_srvs/Trigger.h>
#include <visualization_msgs/Marker.h>

class FuelBridge {
  using SyncPolicy = message_filters::sync_policies::ApproximateTime<
      sensor_msgs::PointCloud2, nav_msgs::Odometry>;
  ros::NodeHandle nh_, pnh_{"~"};
  message_filters::Subscriber<sensor_msgs::PointCloud2> cloud_sub_;
  message_filters::Subscriber<nav_msgs::Odometry> sync_odom_sub_;
  std::unique_ptr<message_filters::Synchronizer<SyncPolicy>> sync_;
  ros::Subscriber odom_sub_, state_sub_, command_sub_, goal_sub_;
  ros::Publisher cloud_pub_, hits_pub_, pose_pub_, command_pub_, trigger_pub_, stop_pub_, bounds_pub_;
  ros::ServiceServer start_srv_, stop_srv_;
  ros::Timer watchdog_;
  nav_msgs::Odometry odom_;
  uav_controller::FlightStatus state_;
  ros::Time cloud_stamp_, state_stamp_, last_command_, started_;
  Eigen::Vector3d lidar_translation_;
  Eigen::Matrix3d lidar_rotation_;
  Eigen::Vector3d bounds_min_, bounds_max_;
  bool have_bounds_ = false;
  std::string world_frame_, lidar_frame_;
  double min_range_, data_timeout_, command_timeout_, planning_timeout_;
  double no_return_range_;
  bool active_ = false;

  bool fresh(const ros::Time& stamp, double timeout) const {
    const double age = (ros::Time::now() - stamp).toSec();
    return !stamp.isZero() && age >= -0.05 && age < timeout;
  }

  bool ready() const {
    return fresh(odom_.header.stamp, data_timeout_) && fresh(cloud_stamp_, data_timeout_) &&
           fresh(state_stamp_, data_timeout_) && state_.feedback_ready && state_.output_policy == "CLOSED_LOOP" &&
           (state_.phase == "HOLD" || state_.phase == "TRAJECTORY");
  }

  bool inBounds(const geometry_msgs::Point& p) const {
    return !have_bounds_ || (p.x > bounds_min_.x() && p.x < bounds_max_.x() &&
        p.y > bounds_min_.y() && p.y < bounds_max_.y() &&
        p.z > bounds_min_.z() && p.z < bounds_max_.z());
  }

  void configureBounds() {
    int count = 0;
    for (int i = 0; i < 3; ++i) {
      const std::string axis(1, "xyz"[i]);
      count += pnh_.getParam("box_min_" + axis, bounds_min_[i]);
      count += pnh_.getParam("box_max_" + axis, bounds_max_[i]);
    }
    if (count == 0) return;
    if (count != 6 || !bounds_min_.allFinite() || !bounds_max_.allFinite() ||
        (bounds_min_.array() >= bounds_max_.array()).any())
      throw std::runtime_error("All six finite exploration bounds are required, with min < max");
    have_bounds_ = true;
    bounds_pub_ = nh_.advertise<visualization_msgs::Marker>("exploration_bounds", 1, true);
    visualization_msgs::Marker marker;
    marker.header.frame_id = world_frame_;
    marker.ns = "exploration_bounds";
    marker.type = visualization_msgs::Marker::LINE_LIST;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.05;
    marker.color.r = 1.0;
    marker.color.g = 0.85;
    marker.color.b = 0.1;
    marker.color.a = 1.0;
    for (int corner = 0; corner < 8; ++corner) {
      for (int axis = 0; axis < 3; ++axis) {
        if (corner & (1 << axis)) continue;
        for (int index : {corner, corner | (1 << axis)}) {
          geometry_msgs::Point p;
          p.x = index & 1 ? bounds_max_.x() : bounds_min_.x();
          p.y = index & 2 ? bounds_max_.y() : bounds_min_.y();
          p.z = index & 4 ? bounds_max_.z() : bounds_min_.z();
          marker.points.push_back(p);
        }
      }
    }
    bounds_pub_.publish(marker);
  }

  void cloudCallback(const sensor_msgs::PointCloud2ConstPtr& cloud,
                     const nav_msgs::OdometryConstPtr& odom) {
    if (cloud->header.frame_id != lidar_frame_ || odom->header.frame_id != world_frame_) {
      ROS_ERROR_THROTTLE(2.0, "FUEL frame mismatch: cloud='%s' (expected '%s'), odometry='%s' (expected '%s'). Check lidar_frame and odom_topic.",
                         cloud->header.frame_id.c_str(), lidar_frame_.c_str(),
                         odom->header.frame_id.c_str(), world_frame_.c_str());
      return;
    }
    if (!fresh(cloud->header.stamp, data_timeout_) ||
        std::abs((cloud->header.stamp - odom->header.stamp).toSec()) > 0.05) return;
    const auto& o = odom->pose.pose;
    Eigen::Quaterniond q(o.orientation.w, o.orientation.x, o.orientation.y, o.orientation.z);
    Eigen::Vector3d position(o.position.x, o.position.y, o.position.z);
    if (!q.coeffs().allFinite() || q.norm() < 1e-6 || !position.allFinite()) return;
    const Eigen::Matrix3d rotation = q.normalized().toRotationMatrix();
    const Eigen::Vector3d origin = position + rotation * lidar_translation_;
    const Eigen::Matrix3d sensor_rotation = rotation * lidar_rotation_;
    pcl::PointCloud<pcl::PointXYZ> input, output, hits;
    pcl::fromROSMsg(*cloud, input);
    output.reserve(input.size());
    for (const auto& point : input) {
      Eigen::Vector3d p(point.x, point.y, point.z);
      // Legacy zero/invalid returns cannot certify free space. The simulation
      // mapping topic instead preserves no-hit rays at the sensor range limit.
      if (!p.allFinite() || p.norm() < min_range_) continue;
      const Eigen::Vector3d w = origin + sensor_rotation * p;
      output.push_back(pcl::PointXYZ(w.x(), w.y(), w.z()));
      if (no_return_range_ <= 0.0 || p.norm() < no_return_range_ - 1e-3)
        hits.push_back(output.back());
    }
    if (output.empty()) return;
    sensor_msgs::PointCloud2 result;
    pcl::toROSMsg(output, result);
    result.header = cloud->header;
    result.header.frame_id = world_frame_;
    geometry_msgs::PoseStamped pose;
    pose.header = result.header;
    pose.pose.position.x = origin.x();
    pose.pose.position.y = origin.y();
    pose.pose.position.z = origin.z();
    Eigen::Quaterniond sensor_q(sensor_rotation);
    pose.pose.orientation.w = sensor_q.w();
    pose.pose.orientation.x = sensor_q.x();
    pose.pose.orientation.y = sensor_q.y();
    pose.pose.orientation.z = sensor_q.z();
    // FUEL consumes world-frame endpoints and the sensor ray origin separately.
    pose_pub_.publish(pose);
    cloud_pub_.publish(result);
    sensor_msgs::PointCloud2 hits_msg;
    pcl::toROSMsg(hits, hits_msg);
    hits_msg.header = result.header;
    hits_pub_.publish(hits_msg);
    cloud_stamp_ = cloud->header.stamp;
  }

  bool start(std_srvs::Trigger::Request&, std_srvs::Trigger::Response& response) {
    if (active_) {
      response.message = "Exploration is already active; stop before restarting.";
      return true;
    }
    if (!inBounds(odom_.pose.pose.position)) {
      response.message = "Current position is outside the exploration box shown in RViz. Check local coordinates and takeoff position.";
      return true;
    }
    if (!ready() || trigger_pub_.getNumSubscribers() == 0 || cloud_pub_.getNumSubscribers() == 0) {
      response.message = "Wait for fresh odometry/cloud, FUEL, and CLOSED_LOOP control.";
      return true;
    }
    nav_msgs::Path trigger;
    trigger.header = odom_.header;
    geometry_msgs::PoseStamped pose;
    pose.header = trigger.header;
    pose.pose = odom_.pose.pose;
    trigger.poses.push_back(pose);
    started_ = ros::Time::now();
    last_command_ = ros::Time();
    active_ = true;
    trigger_pub_.publish(trigger);
    response.success = true;
    response.message = "FUEL exploration triggered.";
    return true;
  }

  void halt() {
    active_ = false;
    stop_pub_.publish(std_msgs::Empty());
    // Finish the direct trajectory at the measured pose so the controller
    // enters HOLD without following a stale planner command.
    if (fresh(state_stamp_, data_timeout_) && state_.feedback_ready &&
        state_.output_policy == "CLOSED_LOOP") {
      uav_controller::TrajectorySetpoint cmd;
      cmd.header.stamp = ros::Time::now();
      cmd.header.frame_id = world_frame_;
      cmd.mission_id = state_.mission_id;
      cmd.position = odom_.pose.pose.position;
      const auto &q = odom_.pose.pose.orientation;
      cmd.yaw = std::atan2(2.0 * (q.w*q.z + q.x*q.y),
                           1.0 - 2.0 * (q.y*q.y + q.z*q.z));
      cmd.finished = true;
      command_pub_.publish(cmd);
    }
  }

  void commandCallback(const quadrotor_msgs::PositionCommandConstPtr& msg) {
    if (!active_ || !ready() || !fresh(msg->header.stamp, command_timeout_) ||
        msg->header.stamp < started_ || msg->header.frame_id != world_frame_ ||
        msg->trajectory_flag != quadrotor_msgs::PositionCommand::TRAJECTORY_STATUS_READY) return;
    for (double value : {msg->position.x, msg->position.y, msg->position.z,
                        msg->velocity.x, msg->velocity.y, msg->velocity.z,
                        msg->acceleration.x, msg->acceleration.y, msg->acceleration.z,
                        msg->yaw, msg->yaw_dot}) {
      if (!std::isfinite(value)) return;
    }
    uav_controller::TrajectorySetpoint cmd;
    cmd.header = msg->header;
    cmd.mission_id = state_.mission_id;
    cmd.position = msg->position;
    cmd.velocity = msg->velocity;
    cmd.acceleration = msg->acceleration;
    cmd.yaw = std::atan2(std::sin(msg->yaw), std::cos(msg->yaw));
    cmd.finished = false;
    command_pub_.publish(cmd);
    last_command_ = msg->header.stamp;
  }

public:
  FuelBridge() {
    pnh_.param<std::string>("world_frame", world_frame_, "world");
    pnh_.param<std::string>("lidar_frame", lidar_frame_, "lidar_link");
    pnh_.param("min_range", min_range_, 0.6);
    pnh_.param("no_return_range", no_return_range_, 40.0);
    pnh_.param("data_timeout", data_timeout_, 0.5);
    pnh_.param("command_timeout", command_timeout_, 0.5);
    pnh_.param("planning_timeout", planning_timeout_, 10.0);
    double roll, pitch, yaw;
    pnh_.param("lidar_x", lidar_translation_.x(), 0.13);
    pnh_.param("lidar_y", lidar_translation_.y(), 0.0);
    pnh_.param("lidar_z", lidar_translation_.z(), 0.28);
    pnh_.param("lidar_roll", roll, 0.0);
    pnh_.param("lidar_pitch", pitch, 0.0);
    pnh_.param("lidar_yaw", yaw, 0.0);
    lidar_rotation_ = (Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) *
                       Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
                       Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX())).toRotationMatrix();
    configureBounds();
    cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("cloud_world", 2);
    hits_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("cloud_hits_world", 2);
    pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("sensor_pose", 2);
    command_pub_ = nh_.advertise<uav_controller::TrajectorySetpoint>("command", 10);
    trigger_pub_ = nh_.advertise<nav_msgs::Path>("trigger", 1);
    stop_pub_ = nh_.advertise<std_msgs::Empty>("stop_planner", 1);
    cloud_sub_.subscribe(nh_, "cloud", 5);
    sync_odom_sub_.subscribe(nh_, "odom", 30);
    sync_.reset(new message_filters::Synchronizer<SyncPolicy>(SyncPolicy(30), cloud_sub_, sync_odom_sub_));
    sync_->setMaxIntervalDuration(ros::Duration(0.05));
    sync_->registerCallback(boost::bind(&FuelBridge::cloudCallback, this, _1, _2));
    odom_sub_ = nh_.subscribe<nav_msgs::Odometry>("odom", 10,
        [this](const nav_msgs::OdometryConstPtr& msg) {
          if (msg->header.frame_id == world_frame_) odom_ = *msg;
        });
    state_sub_ = nh_.subscribe<uav_controller::FlightStatus>("control_state", 10,
        [this](const uav_controller::FlightStatusConstPtr& msg) {
          // Receipt time is authoritative; tests and relays may leave the header unset.
          state_ = *msg; state_stamp_ = ros::Time::now();
        });
    command_sub_ = nh_.subscribe("position_cmd", 10, &FuelBridge::commandCallback, this);
    start_srv_ = nh_.advertiseService("start", &FuelBridge::start, this);
    stop_srv_ = nh_.advertiseService<std_srvs::Trigger::Request, std_srvs::Trigger::Response>(
        "stop", [this](std_srvs::Trigger::Request&, std_srvs::Trigger::Response& response) {
          halt(); response.success = true; response.message = "FUEL stopped; hover requested."; return true;
        });
    goal_sub_ = nh_.subscribe<geometry_msgs::PoseStamped>("goal", 1,
        [this](const geometry_msgs::PoseStampedConstPtr&) {
          std_srvs::Trigger request;
          start(request.request, request.response);
          if (request.response.success) {
            ROS_INFO("RViz 2D Nav Goal: FUEL exploration started.");
          } else {
            ROS_WARN_STREAM("RViz 2D Nav Goal did not start exploration: " << request.response.message);
          }
        });
    watchdog_ = nh_.createTimer(ros::Duration(0.05), [this](const ros::TimerEvent&) {
      if (active_ && (!ready() || (last_command_.isZero() ?
          !fresh(started_, planning_timeout_) : !fresh(last_command_, command_timeout_)))) {
        ROS_ERROR("FUEL input/command timed out or command control lost; exploration stopped.");
        halt();
      }
    });
    ROS_INFO("FUEL bridge ready. Take off and hover in CLOSED_LOOP, then click and drag RViz 2D Nav Goal to start exploration.");
  }
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "fuel_bridge");
  FuelBridge bridge;
  ros::spin();
  return 0;
}
