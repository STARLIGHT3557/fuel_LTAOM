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
#include <uav_mission_msgs/MissionCommand.h>
#include <uav_mission_msgs/PlannerStatus.h>
#include <uav_mission_msgs/TaskContext.h>
#include <quadrotor_msgs/PositionCommand.h>
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

namespace {
bool sameContext(const uav_mission_msgs::TaskContext& a,
                 const uav_mission_msgs::TaskContext& b) {
  return a.manager_epoch == b.manager_epoch && a.mission_id == b.mission_id &&
         a.task_type == b.task_type && a.source_id == b.source_id;
}
}  // namespace

class FuelBridge {
  using SyncPolicy = message_filters::sync_policies::ApproximateTime<
      sensor_msgs::PointCloud2, nav_msgs::Odometry>;
  ros::NodeHandle nh_, pnh_{"~"};
  message_filters::Subscriber<sensor_msgs::PointCloud2> cloud_sub_;
  message_filters::Subscriber<nav_msgs::Odometry> sync_odom_sub_;
  std::unique_ptr<message_filters::Synchronizer<SyncPolicy>> sync_;
  ros::Subscriber odom_sub_, state_sub_, command_sub_, mission_sub_, authorization_sub_;
  ros::Publisher cloud_pub_, hits_pub_, pose_pub_, command_pub_, trigger_pub_, bounds_pub_, planner_status_pub_;
  ros::Publisher model_pub_;
  ros::Timer watchdog_;
  nav_msgs::Odometry odom_;
  uav_controller::FlightStatus state_;
  ros::Time cloud_stamp_, state_stamp_, last_command_, started_;
  Eigen::Vector3d lidar_translation_;
  Eigen::Matrix3d lidar_rotation_;
  Eigen::Vector3d cloud_translation_;
  Eigen::Matrix3d cloud_rotation_;
  Eigen::Vector3d bounds_min_, bounds_max_;
  bool have_bounds_ = false;
  std::string world_frame_, lidar_frame_, cloud_frame_, source_id_;
  double min_range_, data_timeout_, command_timeout_, planning_timeout_;
  bool active_ = false;
  bool timeout_reported_ = false;
  uint64_t trajectory_id_ = 0;
  uav_mission_msgs::TaskContext authorization_, context_, closed_context_;
  uav_mission_msgs::MissionCommand last_mission_command_;
  bool have_mission_command_ = false, have_closed_context_ = false;

  bool fresh(const ros::Time& stamp, double timeout) const {
    const double age = (ros::Time::now() - stamp).toSec();
    return !stamp.isZero() && age >= -0.05 && age < timeout;
  }

  void publishModel(const nav_msgs::Odometry& odom) const {
    if (model_pub_.getNumSubscribers() == 0 || !fresh(odom.header.stamp, 0.5)) return;
    const auto& pose = odom.pose.pose;
    const Eigen::Vector3d position(pose.position.x, pose.position.y, pose.position.z);
    Eigen::Quaterniond attitude(pose.orientation.w, pose.orientation.x,
                                pose.orientation.y, pose.orientation.z);
    if (!position.allFinite() || !attitude.coeffs().allFinite() ||
        !std::isfinite(attitude.norm()) || attitude.norm() < 1e-6) return;
    attitude.normalize();

    using Marker = visualization_msgs::Marker;
    visualization_msgs::MarkerArray model;
    model.markers.reserve(12);
    // Body coordinates: X forward, Y left, Z up. Dimensions are in metres.
    const double motor_offset = 0.17;
    const double diagonal = std::atan2(1.0, 1.0);
    const Eigen::Vector3d grey(0.32, 0.36, 0.42);
    const Eigen::Vector3d orange(1.0, 0.45, 0.05);
    const Eigen::Vector3d blue(0.12, 0.55, 1.0);
    auto part = [&](int type, const Eigen::Vector3d& offset,
                    const Eigen::Vector3d& size, double yaw,
                    const Eigen::Vector3d& colour, double alpha) {
      Marker marker;
      marker.header = odom.header;
      marker.ns = "uav_model";
      marker.id = static_cast<int>(model.markers.size());
      marker.type = type;
      marker.action = Marker::ADD;
      const Eigen::Vector3d centre = position + attitude * offset;
      const Eigen::Quaterniond rotation =
          attitude * Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ());
      marker.pose.position.x = centre.x();
      marker.pose.position.y = centre.y();
      marker.pose.position.z = centre.z();
      marker.pose.orientation.w = rotation.w();
      marker.pose.orientation.x = rotation.x();
      marker.pose.orientation.y = rotation.y();
      marker.pose.orientation.z = rotation.z();
      marker.scale.x = size.x();
      marker.scale.y = size.y();
      marker.scale.z = size.z();
      marker.color.r = colour.x();
      marker.color.g = colour.y();
      marker.color.b = colour.z();
      marker.color.a = alpha;
      marker.lifetime = ros::Duration(0.5);
      model.markers.push_back(marker);
    };

    part(Marker::CUBE, {0, 0, 0}, {0.22, 0.14, 0.06}, 0, grey, 1);
    part(Marker::CUBE, {0.10, 0, 0}, {0.055, 0.14, 0.062}, 0, orange, 1);
    const double arm_length = 2 * std::sqrt(2.0) * motor_offset;
    for (double yaw : {diagonal, -diagonal})
      part(Marker::CUBE, {0, 0, 0}, {arm_length, 0.025, 0.025}, yaw, grey, 1);
    for (double x : {motor_offset, -motor_offset}) {
      for (double y : {motor_offset, -motor_offset}) {
        part(Marker::CYLINDER, {x, y, 0.015}, {0.045, 0.045, 0.035}, 0, grey, 1);
        part(Marker::CYLINDER, {x, y, 0.038}, {0.18, 0.18, 0.006}, 0,
             x > 0 ? orange : blue, 0.65);
      }
    }
    model_pub_.publish(model);
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
    if (cloud->header.frame_id != cloud_frame_ || odom->header.frame_id != world_frame_) {
      ROS_ERROR_THROTTLE(2.0, "FUEL frame mismatch: cloud='%s' (expected '%s'), odometry='%s' (expected '%s'). Check cloud_frame and odom_topic.",
                         cloud->header.frame_id.c_str(), cloud_frame_.c_str(),
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
      if (!p.allFinite()) continue;
      // FAST-LIO's deskewed cloud is in its IMU frame. Its coordinate
      // origin differs from the LiDAR ray origin used for range filtering.
      const Eigen::Vector3d body_point = cloud_translation_ + cloud_rotation_ * p;
      if ((body_point - lidar_translation_).norm() < min_range_) continue;
      const Eigen::Vector3d w = position + rotation * body_point;
      output.push_back(pcl::PointXYZ(w.x(), w.y(), w.z()));
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

  bool start(const uav_mission_msgs::MissionCommand& command) {
    if (have_closed_context_ && sameContext(closed_context_, command.context)) {
      ROS_WARN("Ignoring delayed START for a closed mission batch");
      return true;
    }
    if (active_ && sameContext(context_, command.context)) return true;
    if (!inBounds(odom_.pose.pose.position)) {
      ROS_ERROR("Mission rejected: current position is outside the exploration box.");
      return true;
    }
    if (!ready() || trigger_pub_.getNumSubscribers() == 0 || cloud_pub_.getNumSubscribers() == 0) {
      ROS_WARN_THROTTLE(1.0, "Authorized mission is waiting for fresh odometry/cloud, FUEL, and CLOSED_LOOP control.");
      return false;
    }
    nav_msgs::Path trigger;
    trigger.header = odom_.header;
    geometry_msgs::PoseStamped pose;
    pose.header = trigger.header;
    pose.pose = odom_.pose.pose;
    trigger.poses.push_back(pose);
    context_ = command.context;
    have_closed_context_ = false;
    started_ = ros::Time::now();
    last_command_ = ros::Time();
    trajectory_id_ = 0;
    timeout_reported_ = false;
    active_ = true;
    trigger_pub_.publish(trigger);
    return true;
  }

  void halt(const geometry_msgs::Point* hold = nullptr) {
    active_ = false;
    if (fresh(state_stamp_, data_timeout_) && state_.feedback_ready &&
        state_.output_policy == "CLOSED_LOOP") {
      uav_controller::TrajectorySetpoint cmd;
      cmd.header.stamp = ros::Time::now();
      cmd.header.frame_id = world_frame_;
      cmd.context = context_;
      cmd.trajectory_id = trajectory_id_;
      cmd.position = hold ? *hold : odom_.pose.pose.position;
      const auto &q = odom_.pose.pose.orientation;
      // Home holds local ENU yaw zero; other stops retain measured heading.
      cmd.yaw = hold ? 0.0 : std::atan2(2.0 * (q.w*q.z + q.x*q.y),
                           1.0 - 2.0 * (q.y*q.y + q.z*q.z));
      cmd.finished = true;
      command_pub_.publish(cmd);
    }
  }

  void publishPlannerStatus(uint8_t state, const std::string& detail) {
    uav_mission_msgs::PlannerStatus status;
    status.header.stamp = ros::Time::now();
    status.header.frame_id = world_frame_;
    status.context = context_;
    status.trajectory_id = trajectory_id_;
    status.state = state;
    status.detail = detail;
    planner_status_pub_.publish(status);
  }

  bool handleMissionCommand(const uav_mission_msgs::MissionCommand& command) {
    if (command.context.manager_epoch == 0 || command.context.mission_id == 0 ||
        command.context.task_type == uav_mission_msgs::TaskContext::TASK_NONE ||
        command.context.source_id != source_id_) {
      ROS_WARN("Ignoring unauthorized mission command");
      return true;
    }
    // Authorization and command are separate ROS connections, so their
    // delivery order is intentionally not assumed.
    if (!sameContext(command.context, authorization_)) return false;
    if (command.command == uav_mission_msgs::MissionCommand::STOP) {
      if (sameContext(command.context, context_)) {
        closed_context_ = command.context;
        have_closed_context_ = true;
        halt();
      }
      return true;
    }
    if (command.command == uav_mission_msgs::MissionCommand::HOLD_HOME) {
      if (sameContext(command.context, context_)) {
        closed_context_ = command.context;
        have_closed_context_ = true;
        halt(&command.goal);
      }
      return true;
    }
    if (command.command == uav_mission_msgs::MissionCommand::REPLAN) {
      if (!sameContext(command.context, context_) ||
          (have_closed_context_ && sameContext(command.context, closed_context_))) return true;
      if (!ready()) return false;
      // Reopen after a watchdog timeout without rolling back the accepted
      // trajectory id, changing authorization or re-triggering exploration.
      if (!active_) {
        active_ = true;
        started_ = ros::Time::now();
        last_command_ = ros::Time();
        timeout_reported_ = false;
      }
      return true;
    }
    if (command.command != uav_mission_msgs::MissionCommand::START) return true;
    return start(command);
  }

  void missionCallback(const uav_mission_msgs::MissionCommandConstPtr& msg) {
    last_mission_command_ = *msg;
    have_mission_command_ = true;
    if (handleMissionCommand(*msg)) have_mission_command_ = false;
  }

  void authorizationCallback(const uav_mission_msgs::TaskContextConstPtr& msg) {
    authorization_ = *msg;
    if (have_mission_command_ && handleMissionCommand(last_mission_command_))
      have_mission_command_ = false;
  }

  void commandCallback(const quadrotor_msgs::PositionCommandConstPtr& msg) {
    if (!active_ || !ready() || !fresh(msg->header.stamp, command_timeout_) ||
        msg->header.stamp < started_ || msg->header.frame_id != world_frame_ ||
        msg->trajectory_flag != quadrotor_msgs::PositionCommand::TRAJECTORY_STATUS_READY ||
        !sameContext(msg->context, context_) || msg->trajectory_id == 0 ||
        msg->trajectory_id < trajectory_id_) return;
    for (double value : {msg->position.x, msg->position.y, msg->position.z,
                        msg->velocity.x, msg->velocity.y, msg->velocity.z,
                        msg->acceleration.x, msg->acceleration.y, msg->acceleration.z,
                        msg->yaw, msg->yaw_dot}) {
      if (!std::isfinite(value)) return;
    }
    uav_controller::TrajectorySetpoint cmd;
    cmd.header = msg->header;
    cmd.context = msg->context;
    cmd.trajectory_id = msg->trajectory_id;
    cmd.position = msg->position;
    cmd.velocity = msg->velocity;
    cmd.acceleration = msg->acceleration;
    cmd.yaw = std::atan2(std::sin(msg->yaw), std::cos(msg->yaw));
    cmd.finished = false;
    command_pub_.publish(cmd);
    trajectory_id_ = msg->trajectory_id;
    last_command_ = msg->header.stamp;
    timeout_reported_ = false;
  }

public:
  FuelBridge() {
    pnh_.param<std::string>("world_frame", world_frame_, "world");
    pnh_.param<std::string>("lidar_frame", lidar_frame_, "lidar_link");
    pnh_.param<std::string>("cloud_frame", cloud_frame_, lidar_frame_);
    pnh_.param<std::string>("source_id", source_id_, "fuel_planner");
    if (source_id_.empty()) throw std::invalid_argument("source_id must not be empty");
    pnh_.param("min_range", min_range_, 0.6);
    pnh_.param("data_timeout", data_timeout_, 0.5);
    pnh_.param("command_timeout", command_timeout_, 0.5);
    pnh_.param("planning_timeout", planning_timeout_, 10.0);
    double roll, pitch, yaw;
    pnh_.param("lidar_x", lidar_translation_.x(), 0.0);
    pnh_.param("lidar_y", lidar_translation_.y(), 0.0);
    pnh_.param("lidar_z", lidar_translation_.z(), 0.0);
    pnh_.param("lidar_roll", roll, 0.0);
    pnh_.param("lidar_pitch", pitch, 0.0);
    pnh_.param("lidar_yaw", yaw, 0.0);
    lidar_rotation_ = (Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) *
                       Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
                       Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX())).toRotationMatrix();
    cloud_translation_ = lidar_translation_;
    cloud_rotation_ = lidar_rotation_;
    std::vector<double> cloud_translation, cloud_rotation;
    const bool have_translation = pnh_.getParam("cloud_translation", cloud_translation);
    const bool have_rotation = pnh_.getParam("cloud_rotation", cloud_rotation);
    if (have_translation != have_rotation ||
        (have_translation && (cloud_translation.size() != 3 || cloud_rotation.size() != 9)))
      throw std::invalid_argument("cloud_translation and cloud_rotation require 3 and 9 values");
    if (have_translation) {
      for (int i = 0; i < 3; ++i) {
        cloud_translation_[i] = cloud_translation[i];
        for (int j = 0; j < 3; ++j) cloud_rotation_(i, j) = cloud_rotation[3*i + j];
      }
    }
    if (!cloud_translation_.allFinite() || !cloud_rotation_.allFinite() ||
        !lidar_translation_.allFinite() || !lidar_rotation_.allFinite() ||
        !(cloud_rotation_.transpose() * cloud_rotation_).isApprox(Eigen::Matrix3d::Identity(), 1e-5) ||
        std::abs(cloud_rotation_.determinant() - 1.0) > 1e-5 ||
        !std::isfinite(min_range_) || min_range_ < 0.0 || cloud_frame_.empty() || world_frame_.empty())
      throw std::invalid_argument("Invalid cloud/LiDAR extrinsics or frame configuration");
    configureBounds();
    cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("cloud_world", 2);
    hits_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("cloud_hits_world", 2);
    pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("sensor_pose", 2);
    model_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("uav_model", 1);
    command_pub_ = nh_.advertise<uav_controller::TrajectorySetpoint>("command", 10);
    trigger_pub_ = nh_.advertise<nav_msgs::Path>("trigger", 1);
    planner_status_pub_ = nh_.advertise<uav_mission_msgs::PlannerStatus>("planner_status", 10);
    cloud_sub_.subscribe(nh_, "cloud", 5);
    sync_odom_sub_.subscribe(nh_, "odom", 30);
    sync_.reset(new message_filters::Synchronizer<SyncPolicy>(SyncPolicy(30), cloud_sub_, sync_odom_sub_));
    sync_->setMaxIntervalDuration(ros::Duration(0.05));
    sync_->registerCallback(boost::bind(&FuelBridge::cloudCallback, this, _1, _2));
    odom_sub_ = nh_.subscribe<nav_msgs::Odometry>("odom", 10,
        [this](const nav_msgs::OdometryConstPtr& msg) {
          if (msg->header.frame_id == world_frame_) {
            odom_ = *msg;
            publishModel(*msg);
          }
        });
    state_sub_ = nh_.subscribe<uav_controller::FlightStatus>("control_state", 10,
        [this](const uav_controller::FlightStatusConstPtr& msg) {
          // Receipt time is authoritative; tests and relays may leave the header unset.
          state_ = *msg; state_stamp_ = ros::Time::now();
        });
    command_sub_ = nh_.subscribe("position_cmd", 10, &FuelBridge::commandCallback, this);
    mission_sub_ = nh_.subscribe("mission_command", 10, &FuelBridge::missionCallback, this);
    authorization_sub_ = nh_.subscribe("authorized_context", 10, &FuelBridge::authorizationCallback, this);
    watchdog_ = nh_.createTimer(ros::Duration(0.05), [this](const ros::TimerEvent&) {
      if (have_mission_command_ && handleMissionCommand(last_mission_command_))
        have_mission_command_ = false;
      if (active_ && (!ready() || (last_command_.isZero() ?
          !fresh(started_, planning_timeout_) : !fresh(last_command_, command_timeout_)))) {
        if (!timeout_reported_) {
          publishPlannerStatus(uav_mission_msgs::PlannerStatus::OUTPUT_TIMEOUT,
                               "FUEL input or PositionCommand output timed out");
          timeout_reported_ = true;
        }
        ROS_ERROR("FUEL input/command timed out or command control lost; hover requested.");
        halt();
      }
    });
    ROS_INFO("FUEL bridge ready; mission_manager owns start/stop/return decisions.");
  }
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "fuel_bridge");
  FuelBridge bridge;
  ros::spin();
  return 0;
}
