// -----------------------------------------------------------------------------
//  trajectory_rviz_node (ROS1)
//
//  Publishes RViz-friendly desired/actual trajectory paths and markers for the
//  MAVROS trajectory/OFFBOARD tests.
// -----------------------------------------------------------------------------

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <string>

#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <mavros_msgs/PositionTarget.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <std_msgs/ColorRGBA.h>
#include <std_msgs/Header.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

class TrajectoryRvizNode {
public:
  TrajectoryRvizNode() : pnh_("~") {
    pnh_.param<std::string>("frame_id", frame_id_, "map");
    pnh_.param<std::string>("setpoint_topic", setpoint_topic_,
                            "/uav_controller/active_setpoint");
    pnh_.param<std::string>("pose_topic", pose_topic_,
                            "/mavros/local_position/pose");
    pnh_.param<std::string>("velocity_topic", velocity_topic_,
                            "/mavros/local_position/velocity_local");
    pnh_.param<std::string>("desired_path_topic", desired_path_topic_,
                            "/uav_controller/rviz/desired_path");
    pnh_.param<std::string>("actual_path_topic", actual_path_topic_,
                            "/uav_controller/rviz/actual_path");
    pnh_.param<std::string>("marker_topic", marker_topic_,
                            "/uav_controller/rviz/markers");
    pnh_.param("publish_rate_hz", publish_rate_hz_, 10.0);
    pnh_.param("max_path_points", max_path_points_, 3000);
    pnh_.param("min_path_step", min_path_step_, 0.02);
    pnh_.param("line_width", line_width_, 0.035);
    pnh_.param("text_scale", text_scale_, 0.25);
    pnh_.param("text_offset_z", text_offset_z_, 0.45);
    pnh_.param("show_velocity_arrow", show_velocity_arrow_, true);
    pnh_.param("velocity_arrow_scale", velocity_arrow_scale_, 0.35);

    if (publish_rate_hz_ <= 0.0) {
      ROS_WARN("publish_rate_hz <= 0, using 10 Hz");
      publish_rate_hz_ = 10.0;
    }
    if (max_path_points_ < 2) {
      ROS_WARN("max_path_points < 2, using 2");
      max_path_points_ = 2;
    }
    if (min_path_step_ < 0.0) {
      ROS_WARN("min_path_step < 0, using 0");
      min_path_step_ = 0.0;
    }

    desired_path_.header.frame_id = frame_id_;
    actual_path_.header.frame_id = frame_id_;

    sub_setpoint_ =
        nh_.subscribe(setpoint_topic_, 20, &TrajectoryRvizNode::onSetpoint, this);
    sub_pose_ = nh_.subscribe(pose_topic_, 20, &TrajectoryRvizNode::onPose, this);
    sub_velocity_ =
        nh_.subscribe(velocity_topic_, 20, &TrajectoryRvizNode::onVelocity, this);

    pub_desired_path_ = nh_.advertise<nav_msgs::Path>(desired_path_topic_, 1, true);
    pub_actual_path_ = nh_.advertise<nav_msgs::Path>(actual_path_topic_, 1, true);
    pub_markers_ = nh_.advertise<visualization_msgs::MarkerArray>(marker_topic_, 1);

    timer_ = nh_.createTimer(ros::Duration(1.0 / publish_rate_hz_),
                             &TrajectoryRvizNode::publishVisualization, this);

    ROS_INFO("trajectory_rviz_node: desired='%s' actual='%s' velocity='%s' "
             "frame='%s'",
             setpoint_topic_.c_str(), pose_topic_.c_str(), velocity_topic_.c_str(),
             frame_id_.c_str());
    ROS_INFO("RViz topics: desired_path='%s' actual_path='%s' markers='%s'",
             desired_path_topic_.c_str(), actual_path_topic_.c_str(),
             marker_topic_.c_str());
  }

private:
  static std_msgs::ColorRGBA color(float r, float g, float b, float a) {
    std_msgs::ColorRGBA c;
    c.r = r;
    c.g = g;
    c.b = b;
    c.a = a;
    return c;
  }

  std_msgs::Header header(const ros::Time &stamp) const {
    std_msgs::Header h;
    h.stamp = stamp;
    h.frame_id = frame_id_;
    return h;
  }

  static geometry_msgs::Point makePoint(double x, double y, double z) {
    geometry_msgs::Point p;
    p.x = x;
    p.y = y;
    p.z = z;
    return p;
  }

  static double dist(const geometry_msgs::Point &a, const geometry_msgs::Point &b) {
    const double dx = a.x - b.x;
    const double dy = a.y - b.y;
    const double dz = a.z - b.z;
    return std::sqrt(dx * dx + dy * dy + dz * dz);
  }

  bool appendPathPoint(nav_msgs::Path &path, const geometry_msgs::Point &point,
                       const ros::Time &stamp) {
    if (!path.poses.empty()) {
      const geometry_msgs::Point &last = path.poses.back().pose.position;
      if (dist(last, point) < min_path_step_) {
        return false;
      }
    }

    path.header = header(stamp);
    geometry_msgs::PoseStamped pose;
    pose.header = path.header;
    pose.pose.position = point;
    pose.pose.orientation.w = 1.0;
    path.poses.push_back(pose);

    if (static_cast<int>(path.poses.size()) > max_path_points_) {
      const int extra = static_cast<int>(path.poses.size()) - max_path_points_;
      path.poses.erase(path.poses.begin(), path.poses.begin() + extra);
    }
    return true;
  }

  void onSetpoint(const mavros_msgs::PositionTarget::ConstPtr &msg) {
    const uint16_t ignore_pos = mavros_msgs::PositionTarget::IGNORE_PX |
                                mavros_msgs::PositionTarget::IGNORE_PY |
                                mavros_msgs::PositionTarget::IGNORE_PZ;
    if ((msg->type_mask & ignore_pos) != 0) {
      return;
    }

    desired_pos_ = makePoint(msg->position.x, msg->position.y, msg->position.z);
    have_desired_ = true;
    const ros::Time stamp = msg->header.stamp.isZero() ? ros::Time::now()
                                                       : msg->header.stamp;
    appendPathPoint(desired_path_, desired_pos_, stamp);
  }

  void onPose(const geometry_msgs::PoseStamped::ConstPtr &msg) {
    actual_pos_ = msg->pose.position;
    have_actual_ = true;
    const ros::Time stamp = msg->header.stamp.isZero() ? ros::Time::now()
                                                       : msg->header.stamp;
    appendPathPoint(actual_path_, actual_pos_, stamp);
  }

  void onVelocity(const geometry_msgs::TwistStamped::ConstPtr &msg) {
    velocity_ = msg->twist.linear;
    speed_ = std::sqrt(velocity_.x * velocity_.x + velocity_.y * velocity_.y +
                       velocity_.z * velocity_.z);
    have_velocity_ = true;
  }

  visualization_msgs::Marker baseMarker(int id, const std::string &ns, int type,
                                         const ros::Time &stamp) const {
    visualization_msgs::Marker marker;
    marker.header = header(stamp);
    marker.ns = ns;
    marker.id = id;
    marker.type = type;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.orientation.w = 1.0;
    marker.lifetime = ros::Duration(0.0);
    return marker;
  }

  visualization_msgs::Marker lineMarker(int id, const std::string &ns,
                                         const nav_msgs::Path &path,
                                         const std_msgs::ColorRGBA &c,
                                         const ros::Time &stamp) const {
    visualization_msgs::Marker marker =
        baseMarker(id, ns, visualization_msgs::Marker::LINE_STRIP, stamp);
    marker.scale.x = line_width_;
    marker.color = c;
    marker.points.reserve(path.poses.size());
    for (const auto &pose : path.poses) {
      marker.points.push_back(pose.pose.position);
    }
    return marker;
  }

  visualization_msgs::Marker sphereMarker(int id, const std::string &ns,
                                           const geometry_msgs::Point &point,
                                           const std_msgs::ColorRGBA &c,
                                           const ros::Time &stamp) const {
    visualization_msgs::Marker marker =
        baseMarker(id, ns, visualization_msgs::Marker::SPHERE, stamp);
    marker.pose.position = point;
    marker.scale.x = 0.14;
    marker.scale.y = 0.14;
    marker.scale.z = 0.14;
    marker.color = c;
    return marker;
  }

  visualization_msgs::Marker speedTextMarker(const ros::Time &stamp) const {
    visualization_msgs::Marker marker =
        baseMarker(4, "uav_speed", visualization_msgs::Marker::TEXT_VIEW_FACING,
                   stamp);
    marker.pose.position = actual_pos_;
    marker.pose.position.z += text_offset_z_;
    marker.scale.z = text_scale_;
    marker.color = color(1.0f, 1.0f, 1.0f, 1.0f);

    char text[160];
    if (have_velocity_) {
      std::snprintf(text, sizeof(text),
                    "speed: %.2f m/s\nvx %.2f  vy %.2f  vz %.2f", speed_,
                    velocity_.x, velocity_.y, velocity_.z);
    } else {
      std::snprintf(text, sizeof(text), "speed: -- m/s");
    }
    marker.text = text;
    return marker;
  }

  visualization_msgs::Marker velocityArrowMarker(const ros::Time &stamp) const {
    visualization_msgs::Marker marker =
        baseMarker(5, "uav_speed", visualization_msgs::Marker::ARROW, stamp);
    marker.points.push_back(actual_pos_);
    marker.points.push_back(makePoint(actual_pos_.x + velocity_arrow_scale_ * velocity_.x,
                                      actual_pos_.y + velocity_arrow_scale_ * velocity_.y,
                                      actual_pos_.z + velocity_arrow_scale_ * velocity_.z));
    marker.scale.x = 0.04;
    marker.scale.y = 0.10;
    marker.scale.z = 0.16;
    marker.color = color(0.25f, 1.0f, 0.25f, 0.9f);
    return marker;
  }

  visualization_msgs::Marker deleteMarker(int id, const std::string &ns,
                                           const ros::Time &stamp) const {
    visualization_msgs::Marker marker = baseMarker(id, ns, 0, stamp);
    marker.action = visualization_msgs::Marker::DELETE;
    return marker;
  }

  void publishVisualization(const ros::TimerEvent &) {
    const ros::Time now = ros::Time::now();

    desired_path_.header = header(now);
    actual_path_.header = header(now);
    pub_desired_path_.publish(desired_path_);
    pub_actual_path_.publish(actual_path_);

    visualization_msgs::MarkerArray markers;
    markers.markers.push_back(lineMarker(0, "uav_trajectory", desired_path_,
                                         color(0.0f, 0.85f, 1.0f, 0.95f), now));
    markers.markers.push_back(lineMarker(1, "uav_trajectory", actual_path_,
                                         color(1.0f, 0.45f, 0.0f, 0.95f), now));

    if (have_desired_) {
      markers.markers.push_back(sphereMarker(2, "uav_trajectory", desired_pos_,
                                             color(0.0f, 0.85f, 1.0f, 0.95f),
                                             now));
    }
    if (have_actual_) {
      markers.markers.push_back(sphereMarker(3, "uav_trajectory", actual_pos_,
                                             color(1.0f, 0.45f, 0.0f, 0.95f),
                                             now));
      markers.markers.push_back(speedTextMarker(now));
      if (show_velocity_arrow_ && have_velocity_ && speed_ > 1e-3) {
        markers.markers.push_back(velocityArrowMarker(now));
      } else {
        markers.markers.push_back(deleteMarker(5, "uav_speed", now));
      }
    }

    pub_markers_.publish(markers);
  }

  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  ros::Subscriber sub_setpoint_;
  ros::Subscriber sub_pose_;
  ros::Subscriber sub_velocity_;
  ros::Publisher pub_desired_path_;
  ros::Publisher pub_actual_path_;
  ros::Publisher pub_markers_;
  ros::Timer timer_;

  nav_msgs::Path desired_path_;
  nav_msgs::Path actual_path_;
  geometry_msgs::Point desired_pos_;
  geometry_msgs::Point actual_pos_;
  geometry_msgs::Vector3 velocity_;

  std::string frame_id_;
  std::string setpoint_topic_;
  std::string pose_topic_;
  std::string velocity_topic_;
  std::string desired_path_topic_;
  std::string actual_path_topic_;
  std::string marker_topic_;
  double publish_rate_hz_{10.0};
  int max_path_points_{3000};
  double min_path_step_{0.02};
  double line_width_{0.035};
  double text_scale_{0.25};
  double text_offset_z_{0.45};
  bool show_velocity_arrow_{true};
  double velocity_arrow_scale_{0.35};
  double speed_{0.0};
  bool have_desired_{false};
  bool have_actual_{false};
  bool have_velocity_{false};
};

int main(int argc, char **argv) {
  ros::init(argc, argv, "trajectory_rviz_node");
  TrajectoryRvizNode node;
  ros::spin();
  return 0;
}
