#include <ros/ros.h>
#include <cmath>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>

class LtaomOdomAdapter {
public:
  LtaomOdomAdapter() {
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");
    // FAST-LIO output is already aligned to the first flight's frame when it
    // runs with multisession_mode=1 (it relocalizes against the loaded prior
    // map), so this is the correct real-time pose source for FUEL in both
    // flights. Override via `odom_topic` if a different source is used.
    std::string odom_topic;
    pnh.param<std::string>("odom_topic", odom_topic, "/aft_mapped_to_init");
    pnh.param("guard/max_speed", guard_max_speed_, 8.0);
    pnh.param("guard/enabled", guard_enabled_, true);
    sub_ = nh.subscribe(odom_topic, 100, &LtaomOdomAdapter::cb, this);
    // FUEL 用：twist 清零
    pub_odom_ = nh.advertise<nav_msgs::Odometry>("/ltaom/odom_aligned", 100);
    // 几何控制器用：真实位姿和速度
    pub_pose_ = nh.advertise<geometry_msgs::PoseStamped>("/ltaom/pose", 100);
    pub_vel_  = nh.advertise<geometry_msgs::TwistStamped>("/ltaom/velocity", 100);
    ROS_INFO("LTAOM adapter: sub %s", odom_topic.c_str());
    ROS_INFO("  pub /ltaom/odom_aligned (FUEL, twist=0)");
    ROS_INFO("  pub /ltaom/pose (controller feedback)");
    ROS_INFO("  pub /ltaom/velocity (controller velocity damping)");
  }

  // A LIO/SLAM estimate can jump or diverge. Publishing such a step would send
  // the planner a teleport and the attitude loop a max-thrust command, so
  // implausible steps are dropped: downstream then sees a stale input and holds.
  bool implausible(const nav_msgs::Odometry& msg) {
    if (!guard_enabled_) return false;
    const auto& p = msg.pose.pose.position;
    if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) return true;
    if (!have_prev_) {
      have_prev_ = true;
      prev_ = msg;
      return false;
    }
    const double dt = (msg.header.stamp - prev_.header.stamp).toSec();
    const auto& q = prev_.pose.pose.position;
    const double dx = p.x - q.x, dy = p.y - q.y, dz = p.z - q.z;
    const double step = std::sqrt(dx * dx + dy * dy + dz * dz);
    if (dt > 0.0 && dt < 1.0 && step > guard_max_speed_ * dt + 0.5) {
      // Accept anyway once the same new pose region has persisted for a while:
      // that is a real frame re-anchor (multi-session relocalization).
      if (reject_since_ == ros::Time()) reject_since_ = msg.header.stamp;
      if ((msg.header.stamp - reject_since_).toSec() < 2.0) {
        ROS_WARN_THROTTLE(1.0,
            "LTAOM pose step rejected: %.2f m in %.3f s (limit %.1f m/s); holding",
            step, dt, guard_max_speed_);
        return true;
      }
      ROS_WARN("LTAOM pose re-anchored after %.1f s of persistent offset",
               (msg.header.stamp - reject_since_).toSec());
    }
    reject_since_ = ros::Time();
    prev_ = msg;
    return false;
  }

  void cb(const nav_msgs::OdometryConstPtr& msg) {
    if (implausible(*msg)) return;
    // 1. FUEL：frame_id 重映射。线速度保持 FAST-LIO 的世界系速度：FUEL 用
    // odom_vel_ 给每次重规划做速度连续初始化，清零会让轨迹每次从静止起步。
    // 消费端需设 odom_twist_in_body:=false（世界系，不再做一次姿态旋转）。
    nav_msgs::Odometry out = *msg;
    out.header.frame_id = "world";
    out.child_frame_id = "base_link";
    pub_odom_.publish(out);

    // 2. 控制器：PoseStamped（真实姿态，不清零）
    geometry_msgs::PoseStamped pose;
    pose.header = msg->header;
    pose.header.frame_id = "world";
    pose.pose = msg->pose.pose;
    pub_pose_.publish(pose);

    // 3. 控制器：TwistStamped（FAST-LIO 速度，用于阻尼）
    geometry_msgs::TwistStamped vel;
    vel.header = msg->header;
    vel.header.frame_id = "world";
    vel.twist = msg->twist.twist;
    pub_vel_.publish(vel);
  }

private:
  ros::Subscriber sub_;
  ros::Publisher pub_odom_, pub_pose_, pub_vel_;
  nav_msgs::Odometry prev_;
  ros::Time reject_since_;
  bool have_prev_{false}, guard_enabled_{true};
  double guard_max_speed_{8.0};
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "ltaom_odom_adapter");
  LtaomOdomAdapter adapter;
  ros::spin();
  return 0;
}
