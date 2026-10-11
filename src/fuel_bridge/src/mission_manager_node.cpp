#include <cmath>
#include <cstdint>
#include <limits>
#include <stdexcept>
#include <string>

#include <vector>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <sensor_msgs/BatteryState.h>
#include <std_srvs/Trigger.h>
#include <uav_controller/FlightStatus.h>
#include <uav_mission_msgs/MissionCommand.h>
#include <uav_mission_msgs/MissionStatus.h>
#include <uav_mission_msgs/PlannerStatus.h>
#include <uav_mission_msgs/TaskContext.h>
#include <uav_mission_msgs/TrajectoryStatus.h>

namespace {
bool sameContext(const uav_mission_msgs::TaskContext& a,
                 const uav_mission_msgs::TaskContext& b) {
  return a.manager_epoch == b.manager_epoch && a.mission_id == b.mission_id &&
         a.task_type == b.task_type && a.source_id == b.source_id;
}

bool finitePoint(const geometry_msgs::Point& p) {
  return std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z);
}
}  // namespace

class MissionManager {
 public:
  MissionManager() : pnh_("~") {
    std::vector<double> home;
    if (!pnh_.getParam("home_position", home) || home.size() != 3) {
      home = {0.0, 0.0, 1.0};
    }
    home_.x = home[0];
    home_.y = home[1];
    home_.z = home[2];
    if (!finitePoint(home_)) throw std::invalid_argument("home_position must be finite");

    pnh_.param("battery/enabled", battery_enabled_, true);
    pnh_.param("battery/return_threshold", battery_threshold_, 0.30);
    pnh_.param("battery/data_timeout", battery_timeout_, 2.0);
    pnh_.param("exploration/initial_plan_timeout", initial_plan_timeout_, 10.0);
    pnh_.param("exploration/planner_status_timeout", planner_status_timeout_, 4.0);
    pnh_.param("return_home/retry_interval", return_retry_interval_, 1.0);
    pnh_.param("return_home/planning_timeout", return_planning_timeout_, 10.0);
    pnh_.param("return_home/position_tolerance", position_tolerance_, 0.20);
    pnh_.param("return_home/speed_tolerance", speed_tolerance_, 0.15);
    pnh_.param("return_home/stable_time", stable_time_, 1.5);
    pnh_.param("data_timeout", data_timeout_, 0.5);
    pnh_.param<std::string>("world_frame", world_frame_, "world");
    pnh_.param<std::string>("source_id", source_id_, "fuel_planner");
    if (!(battery_threshold_ >= 0.0 && battery_threshold_ <= 1.0) ||
        battery_timeout_ <= 0.0 || initial_plan_timeout_ <= 0.0 ||
        planner_status_timeout_ <= 0.0 ||
        !std::isfinite(return_retry_interval_) ||
        !std::isfinite(return_planning_timeout_) || return_retry_interval_ <= 0.0 ||
        return_planning_timeout_ <= 0.0 ||
        position_tolerance_ <= 0.0 || speed_tolerance_ <= 0.0 || stable_time_ <= 0.0 ||
        data_timeout_ <= 0.0 || source_id_.empty()) {
      throw std::invalid_argument("invalid mission manager parameters");
    }

    manager_epoch_ = ros::WallTime::now().toNSec();
    if (manager_epoch_ == 0) manager_epoch_ = 1;
    context_.manager_epoch = manager_epoch_;
    context_.task_type = uav_mission_msgs::TaskContext::TASK_NONE;
    context_.source_id = source_id_;

    command_pub_ = nh_.advertise<uav_mission_msgs::MissionCommand>("mission_command", 2, true);
    authorization_pub_ = nh_.advertise<uav_mission_msgs::TaskContext>("authorized_context", 2, true);
    status_pub_ = nh_.advertise<uav_mission_msgs::MissionStatus>("mission_status", 2, true);
    odom_sub_ = nh_.subscribe("odom", 10, &MissionManager::odomCallback, this);
    control_sub_ = nh_.subscribe("control_state", 10, &MissionManager::controlCallback, this);
    battery_sub_ = nh_.subscribe("battery", 10, &MissionManager::batteryCallback, this);
    planner_sub_ = nh_.subscribe("planner_status", 20, &MissionManager::plannerCallback, this);
    trajectory_sub_ = nh_.subscribe("trajectory_status", 20, &MissionManager::trajectoryCallback, this);
    goal_sub_ = nh_.subscribe("goal", 1, &MissionManager::goalCallback, this);

    start_srv_ = nh_.advertiseService("start", &MissionManager::start, this);
    stop_srv_ = nh_.advertiseService("stop", &MissionManager::stop, this);
    return_srv_ = nh_.advertiseService("return_home", &MissionManager::returnHome, this);
    timer_ = nh_.createTimer(ros::Duration(0.05), &MissionManager::timerCallback, this);
    publishAuthorization();
    publishStatus("mission manager ready");
    ROS_INFO("Mission manager epoch=%llu, home=[%.2f %.2f %.2f] ENU, battery=%s threshold=%.2f",
             static_cast<unsigned long long>(manager_epoch_), home_.x, home_.y, home_.z,
             battery_enabled_ ? "enabled" : "disabled", battery_threshold_);
  }

 private:
  bool fresh(const ros::Time& stamp, double timeout) const {
    if (stamp.isZero()) return false;
    const double age = (ros::Time::now() - stamp).toSec();
    return age >= -0.05 && age < timeout;
  }

  bool batteryValid() const {
    return battery_sample_valid_ && fresh(battery_stamp_, battery_timeout_);
  }

  bool flightReady() const {
    return fresh(odom_.header.stamp, data_timeout_) && fresh(control_stamp_, data_timeout_) &&
           control_.feedback_ready && control_.output_policy == "CLOSED_LOOP" &&
           (control_.phase == "HOLD" || control_.phase == "TRAJECTORY");
  }

  void allocateTask(uint8_t type) {
    context_.manager_epoch = manager_epoch_;
    context_.mission_id = ++last_mission_id_;
    context_.task_type = type;
    context_.source_id = source_id_;
    task_started_ = ros::Time::now();
    planner_stamp_ = ros::Time();
    explore_trajectory_ready_ = false;
    recovery_attempts_ = 0;
    recovery_reason_ = uav_mission_msgs::MissionStatus::REASON_NONE;
    replan_requested_ = false;
    last_replan_request_ = ros::Time();
    reference_settled_ = false;
    settled_stamp_ = ros::Time();
    last_active_trajectory_ = 0;
    stable_since_ = ros::Time();
    publishAuthorization();
  }

  void publishAuthorization() { authorization_pub_.publish(context_); }

  void publishCommand(uint8_t command) {
    uav_mission_msgs::MissionCommand msg;
    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = world_frame_;
    msg.context = context_;
    msg.command = command;
    msg.goal = home_;
    command_pub_.publish(msg);
  }

  void publishStatus(const std::string& detail) {
    detail_ = detail;
    uav_mission_msgs::MissionStatus msg;
    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = world_frame_;
    msg.context = context_;
    msg.state = state_;
    msg.return_reason = return_reason_;
    msg.recovery_reason = recovery_reason_;
    msg.recovery_attempts = recovery_attempts_;
    msg.battery_valid = batteryValid();
    msg.battery_percentage = msg.battery_valid
        ? static_cast<float>(battery_percentage_)
        : std::numeric_limits<float>::quiet_NaN();
    msg.mission_done = state_ == uav_mission_msgs::MissionStatus::HOME_HOLD;
    msg.detail = detail_;
    status_pub_.publish(msg);
  }

  void beginReturn(uint8_t reason, const std::string& detail) {
    if (state_ == uav_mission_msgs::MissionStatus::PREPARE_RETURN ||
        state_ == uav_mission_msgs::MissionStatus::RETURNING ||
        state_ == uav_mission_msgs::MissionStatus::HOME_HOLD ||
        state_ == uav_mission_msgs::MissionStatus::RETURN_RECOVERY) {
      return;
    }
    return_reason_ = reason;
    allocateTask(uav_mission_msgs::TaskContext::RETURN_HOME);
    state_ = uav_mission_msgs::MissionStatus::PREPARE_RETURN;
    publishCommand(uav_mission_msgs::MissionCommand::START);
    publishStatus(detail);
    ROS_WARN("Return-home latched: reason=%u mission=%llu (%s)", reason,
             static_cast<unsigned long long>(context_.mission_id), detail.c_str());
  }

  bool returning() const {
    return state_ == uav_mission_msgs::MissionStatus::PREPARE_RETURN ||
           state_ == uav_mission_msgs::MissionStatus::RETURNING ||
           state_ == uav_mission_msgs::MissionStatus::RETURN_RECOVERY;
  }

  void recoverReturn(uint8_t reason, const std::string& detail, bool request_replan) {
    state_ = uav_mission_msgs::MissionStatus::RETURN_RECOVERY;
    recovery_reason_ = reason;
    replan_requested_ = replan_requested_ || request_replan;
    publishStatus(detail);
    ROS_WARN_THROTTLE(1.0, "Return recovery (same batch, retrying): %s", detail.c_str());
  }

  bool start(std_srvs::Trigger::Request&, std_srvs::Trigger::Response& response) {
    if (state_ != uav_mission_msgs::MissionStatus::READY) {
      response.message = "Mission is not READY; duplicate start cannot replace the active task.";
      return true;
    }
    if (!flightReady()) {
      response.message = "Wait for fresh odometry and CLOSED_LOOP controller feedback.";
      return true;
    }
    if (battery_enabled_ && !batteryValid()) {
      response.message = "Exploration requires a fresh valid battery percentage.";
      return true;
    }
    if (battery_enabled_ && battery_percentage_ < battery_threshold_) {
      response.message = "Battery is below the exploration threshold.";
      return true;
    }
    allocateTask(uav_mission_msgs::TaskContext::EXPLORE);
    return_reason_ = uav_mission_msgs::MissionStatus::REASON_NONE;
    state_ = uav_mission_msgs::MissionStatus::EXPLORING;
    publishCommand(uav_mission_msgs::MissionCommand::START);
    publishStatus("exploration started");
    response.success = true;
    response.message = "FUEL exploration task authorized.";
    return true;
  }

  bool stop(std_srvs::Trigger::Request&, std_srvs::Trigger::Response& response) {
    if (state_ != uav_mission_msgs::MissionStatus::READY) {
      publishCommand(uav_mission_msgs::MissionCommand::STOP);
    }
    state_ = uav_mission_msgs::MissionStatus::READY;
    return_reason_ = uav_mission_msgs::MissionStatus::STOP_REQUESTED;
    stable_since_ = ros::Time();
    publishStatus("mission explicitly stopped; hover requested");
    response.success = true;
    response.message = "Mission stopped; this request was not interpreted as return-home.";
    return true;
  }

  bool returnHome(std_srvs::Trigger::Request&, std_srvs::Trigger::Response& response) {
    if (returning() || state_ == uav_mission_msgs::MissionStatus::HOME_HOLD) {
      response.success = true;
      response.message = "Return-home is already active or complete; request is idempotent.";
      return true;
    }
    if (!flightReady()) {
      response.message = "Wait for fresh odometry and CLOSED_LOOP controller feedback.";
      return true;
    }
    beginReturn(uav_mission_msgs::MissionStatus::MANUAL_RETURN, "manual return requested");
    response.success = true;
    response.message = "Return-home task authorized.";
    return true;
  }

  void goalCallback(const geometry_msgs::PoseStampedConstPtr&) {
    std_srvs::Trigger::Request request;
    std_srvs::Trigger::Response response;
    start(request, response);
    if (!response.success) ROS_WARN("RViz exploration start rejected: %s", response.message.c_str());
  }

  void odomCallback(const nav_msgs::OdometryConstPtr& msg) {
    if (msg->header.frame_id == world_frame_) odom_ = *msg;
  }

  void controlCallback(const uav_controller::FlightStatusConstPtr& msg) {
    control_ = *msg;
    control_stamp_ = ros::Time::now();
  }

  void batteryCallback(const sensor_msgs::BatteryStateConstPtr& msg) {
    const double value = msg->percentage;
    battery_sample_valid_ = std::isfinite(value) && value >= 0.0 && value <= 1.0 &&
                            !msg->header.stamp.isZero();
    if (!battery_sample_valid_) {
      ROS_WARN_THROTTLE(2.0, "Ignoring invalid battery percentage/stamp");
      return;
    }
    battery_percentage_ = value;
    battery_stamp_ = msg->header.stamp;
  }

  void plannerCallback(const uav_mission_msgs::PlannerStatusConstPtr& msg) {
    if (!sameContext(msg->context, context_)) return;
    planner_stamp_ = ros::Time::now();
    if (returning()) {
      reference_settled_ = last_active_trajectory_ != 0 &&
                           msg->state == uav_mission_msgs::PlannerStatus::GOAL_REACHED &&
                           msg->trajectory_id == last_active_trajectory_;
      if (reference_settled_) settled_stamp_ = planner_stamp_;
    }
    if (msg->state == uav_mission_msgs::PlannerStatus::TRAJECTORY_READY) {
      explore_trajectory_ready_ = true;
    }
    if (state_ == uav_mission_msgs::MissionStatus::EXPLORING) {
      if (msg->state == uav_mission_msgs::PlannerStatus::EXPLORATION_DONE) {
        beginReturn(uav_mission_msgs::MissionStatus::EXPLORATION_DONE,
                    "exploration completed; preparing return");
      } else if (msg->state == uav_mission_msgs::PlannerStatus::OUTPUT_TIMEOUT) {
        beginReturn(uav_mission_msgs::MissionStatus::TRAJECTORY_TIMEOUT,
                    "exploration trajectory output timed out");
      }
    } else if (returning()) {
      if (msg->state == uav_mission_msgs::PlannerStatus::FAILED) {
        // FUEL already schedules retries. Never stop it or reset its failure
        // history just because a local attempt failed.
        recoverReturn(uav_mission_msgs::MissionStatus::RETURN_PLAN_FAILED,
                      "return planner is retrying with the same task", false);
      } else if (msg->state == uav_mission_msgs::PlannerStatus::TRAJECTORY_UNSAFE) {
        recoverReturn(uav_mission_msgs::MissionStatus::UNSAFE_TRAJECTORY,
                      "braking unsafe execution while replanning return", false);
      } else if (msg->state == uav_mission_msgs::PlannerStatus::OUTPUT_TIMEOUT) {
        recoverReturn(uav_mission_msgs::MissionStatus::TRAJECTORY_TIMEOUT,
                      "return output timed out; reopening planner/bridge", true);
      }
    }
  }

  void trajectoryCallback(const uav_mission_msgs::TrajectoryStatusConstPtr& msg) {
    if (!sameContext(msg->context, context_)) return;
    if (!returning()) return;
    if (msg->state == uav_mission_msgs::TrajectoryStatus::INVALID ||
        msg->state == uav_mission_msgs::TrajectoryStatus::REJECTED) {
      recoverReturn(uav_mission_msgs::MissionStatus::UNSAFE_TRAJECTORY,
                    msg->detail, true);
    } else if (msg->state == uav_mission_msgs::TrajectoryStatus::ACTIVE &&
               msg->trajectory_id > last_active_trajectory_) {
      // Only the executor's actual activation advances the mission. Controller
      // ACKs still belong to the executor/controller handover protocol.
      last_active_trajectory_ = msg->trajectory_id;
      reference_settled_ = false;
      settled_stamp_ = ros::Time();
      stable_since_ = ros::Time();
      state_ = uav_mission_msgs::MissionStatus::RETURNING;
      replan_requested_ = false;
      recovery_reason_ = uav_mission_msgs::MissionStatus::REASON_NONE;
      publishStatus("return trajectory is executing");
    }
  }

  bool reference_settled_{false};
  ros::Time settled_stamp_;
  bool atHome() const {
    if (!reference_settled_ || !fresh(settled_stamp_, 1.0) ||
        !fresh(odom_.header.stamp, data_timeout_)) return false;
    const auto& q = odom_.pose.pose.orientation;
    const double yaw = std::atan2(2.0 * (q.w*q.z + q.x*q.y),
                                  1.0 - 2.0 * (q.y*q.y + q.z*q.z));
    const double yaw_rate = odom_.twist.twist.angular.z;
    if (!std::isfinite(yaw) || !std::isfinite(yaw_rate) ||
        std::abs(yaw) > 0.08 || std::abs(yaw_rate) > 0.10) return false;
    const auto& p = odom_.pose.pose.position;
    const auto& v = odom_.twist.twist.linear;
    if (!finitePoint(p) || !std::isfinite(v.x) || !std::isfinite(v.y) || !std::isfinite(v.z))
      return false;
    const double dx = p.x - home_.x, dy = p.y - home_.y, dz = p.z - home_.z;
    return std::sqrt(dx * dx + dy * dy + dz * dz) <= position_tolerance_ &&
           std::sqrt(v.x * v.x + v.y * v.y + v.z * v.z) <= speed_tolerance_;
  }

  void timerCallback(const ros::TimerEvent&) {
    const ros::Time now = ros::Time::now();
    if (state_ == uav_mission_msgs::MissionStatus::EXPLORING) {
      if (battery_enabled_ && batteryValid() && battery_percentage_ < battery_threshold_) {
        beginReturn(uav_mission_msgs::MissionStatus::LOW_BATTERY,
                    "fresh battery percentage is strictly below threshold");
      } else if (explore_trajectory_ready_ && !fresh(planner_stamp_, planner_status_timeout_)) {
        beginReturn(uav_mission_msgs::MissionStatus::TRAJECTORY_TIMEOUT,
                    "FUEL planner status heartbeat timed out");
      } else if (!explore_trajectory_ready_ && !task_started_.isZero() &&
                 now >= task_started_ &&
                 (now - task_started_).toSec() >= initial_plan_timeout_) {
        beginReturn(uav_mission_msgs::MissionStatus::EXPLORE_PLAN_TIMEOUT,
                    "initial exploration trajectory timed out");
      }
    }
    if (returning()) {
      // The planning deadline ends at the first ACTIVE event, even if a later
      // failure enters recovery. Heartbeat/output watchdogs cover execution.
      const bool initial_return_timeout = last_active_trajectory_ == 0 &&
          !task_started_.isZero() && now >= task_started_ &&
          (now - task_started_).toSec() >= return_planning_timeout_;
      const ros::Time heartbeat = planner_stamp_.isZero() ? task_started_ : planner_stamp_;
      const bool heartbeat_timeout = (last_active_trajectory_ != 0 || !planner_stamp_.isZero()) &&
          !fresh(heartbeat, planner_status_timeout_);
      if (initial_return_timeout || heartbeat_timeout) {
        recoverReturn(uav_mission_msgs::MissionStatus::TRAJECTORY_TIMEOUT,
                      initial_return_timeout ? "first return trajectory activation timed out"
                                             : "return planner status heartbeat timed out", true);
      }
      if (atHome()) {
        if (stable_since_.isZero()) stable_since_ = now;
        if ((now - stable_since_).toSec() >= stable_time_) {
          state_ = uav_mission_msgs::MissionStatus::HOME_HOLD;
          publishCommand(uav_mission_msgs::MissionCommand::HOLD_HOME);
          publishStatus("MISSION_DONE: home reached and stable; holding home position");
        }
      } else {
        stable_since_ = ros::Time();
      }
    }
    if (returning() && replan_requested_ &&
        (last_replan_request_.isZero() || now < last_replan_request_ ||
         (now - last_replan_request_).toSec() >= return_retry_interval_)) {
      publishCommand(uav_mission_msgs::MissionCommand::REPLAN);
      last_replan_request_ = now;
      replan_requested_ = false;
      if (recovery_attempts_ < std::numeric_limits<uint32_t>::max()) ++recovery_attempts_;
    }
    if (last_status_.isZero() || now < last_status_ || (now - last_status_).toSec() >= 0.5) {
      publishStatus(detail_);
      last_status_ = now;
    }
  }

  ros::NodeHandle nh_, pnh_;
  ros::Publisher command_pub_, authorization_pub_, status_pub_;
  ros::Subscriber odom_sub_, control_sub_, battery_sub_, planner_sub_, trajectory_sub_,
      goal_sub_;
  ros::ServiceServer start_srv_, stop_srv_, return_srv_;
  ros::Timer timer_;
  nav_msgs::Odometry odom_;
  uav_controller::FlightStatus control_;
  uav_mission_msgs::TaskContext context_;
  geometry_msgs::Point home_;
  ros::Time control_stamp_, battery_stamp_, task_started_, planner_stamp_, stable_since_, last_status_;
  std::string world_frame_, source_id_, detail_{"ready"};
  double battery_threshold_{0.30}, battery_timeout_{2.0}, initial_plan_timeout_{10.0};
  double planner_status_timeout_{4.0};
  double return_retry_interval_{1.0}, return_planning_timeout_{10.0};
  ros::Time last_replan_request_;
  uint64_t last_active_trajectory_{0};
  uint32_t recovery_attempts_{0};
  uint8_t recovery_reason_{uav_mission_msgs::MissionStatus::REASON_NONE};
  bool replan_requested_{false};
  double position_tolerance_{0.20}, speed_tolerance_{0.15}, stable_time_{1.5}, data_timeout_{0.5};
  double battery_percentage_{std::numeric_limits<double>::quiet_NaN()};
  uint64_t manager_epoch_{0}, last_mission_id_{0};
  uint8_t state_{uav_mission_msgs::MissionStatus::READY};
  uint8_t return_reason_{uav_mission_msgs::MissionStatus::REASON_NONE};
  bool battery_enabled_{true}, battery_sample_valid_{false};
  bool explore_trajectory_ready_{false};
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "mission_manager");
  try {
    MissionManager manager;
    ros::spin();
  } catch (const std::exception& error) {
    ROS_FATAL("Mission manager startup failed: %s", error.what());
    return 1;
  }
  return 0;
}
