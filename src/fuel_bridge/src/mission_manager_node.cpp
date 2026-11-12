#include <cmath>
#include <cstdint>
#include <limits>
#include <iomanip>
#include <sstream>
#include <stdexcept>
#include <string>

#include <vector>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <sensor_msgs/BatteryState.h>
#include <std_srvs/Trigger.h>
#include <std_msgs/Empty.h>
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

const char* missionStateName(uint8_t state) {
  using Status = uav_mission_msgs::MissionStatus;
  switch (state) {
    case Status::READY: return "READY";
    case Status::EXPLORING: return "EXPLORING";
    case Status::PREPARE_RETURN: return "PREPARE_RETURN";
    case Status::RETURNING: return "RETURNING";
    case Status::HOME_HOLD: return "HOME_HOLD";
    case Status::RETURN_RECOVERY: return "RETURN_RECOVERY";
    default: return "UNKNOWN";
  }
}

const char* missionReasonName(uint8_t reason) {
  using Status = uav_mission_msgs::MissionStatus;
  switch (reason) {
    case Status::REASON_NONE: return "NONE";
    case Status::EXPLORATION_DONE: return "EXPLORATION_DONE";
    case Status::LOW_BATTERY: return "LOW_BATTERY";
    case Status::EXPLORE_PLAN_FAILED: return "EXPLORE_PLAN_FAILED";
    case Status::EXPLORE_PLAN_TIMEOUT: return "EXPLORE_PLAN_TIMEOUT";
    case Status::TRAJECTORY_TIMEOUT: return "TRAJECTORY_TIMEOUT";
    case Status::MANUAL_RETURN: return "MANUAL_RETURN";
    case Status::STOP_REQUESTED: return "STOP_REQUESTED";
    case Status::RETURN_PLAN_FAILED: return "RETURN_PLAN_FAILED";
    case Status::UNSAFE_TRAJECTORY: return "UNSAFE_TRAJECTORY";
    default: return "UNKNOWN";
  }
}

const char* plannerStateName(uint8_t state) {
  using Planner = uav_mission_msgs::PlannerStatus;
  switch (state) {
    case Planner::IDLE: return "IDLE";
    case Planner::PLANNING: return "PLANNING";
    case Planner::TRAJECTORY_READY: return "TRAJECTORY_READY";
    case Planner::EXECUTING: return "EXECUTING";
    case Planner::EXPLORATION_DONE: return "EXPLORATION_DONE";
    case Planner::GOAL_REACHED: return "GOAL_REACHED";
    case Planner::FAILED: return "FAILED";
    case Planner::OUTPUT_TIMEOUT: return "OUTPUT_TIMEOUT";
    case Planner::TRAJECTORY_UNSAFE: return "TRAJECTORY_UNSAFE";
    case Planner::OBSERVING_HOME: return "OBSERVING_HOME";
    case Planner::SETTLING: return "SETTLING";
    default: return "UNKNOWN";
  }
}
}  // namespace

class MissionManager {
 public:
  // 参数取自 fuel_bridge/config/mission.yaml，说明见 docs/REALFLIGHT_PARAMETERS.md。
  MissionManager() : pnh_("~") {
    // 返航点（本地 ENU，m），到达后悬停并把航向转到 0
    std::vector<double> home;
    if (!pnh_.getParam("home_position", home) || home.size() != 3) {
      home = {0.0, 0.0, 1.0};
    }
    home_.x = home[0];
    home_.y = home[1];
    home_.z = home[2];
    if (!finitePoint(home_)) throw std::invalid_argument("home_position must be finite");

    // 低电量返航：有效电量低于阈值（比例）时返航；电量数据超时视为无效
    pnh_.param("battery/enabled", battery_enabled_, true);
    pnh_.param("battery/return_threshold", battery_threshold_, 0.30);
    pnh_.param("battery/data_timeout", battery_timeout_, 2.0);
    // 探索保护：首条轨迹等待时间、规划器心跳超时（s），超时则返航
    pnh_.param("exploration/initial_plan_timeout", initial_plan_timeout_, 10.0);
    pnh_.param("exploration/planner_status_timeout", planner_status_timeout_, 4.0);
    // 返航：重试间隔、首条返航轨迹等待时间（s），以及到家判定的位置/速度误差和稳定时间
    pnh_.param("return_home/retry_interval", return_retry_interval_, 1.0);
    pnh_.param("return_home/planning_timeout", return_planning_timeout_, 10.0);
    pnh_.param("return_home/position_tolerance", position_tolerance_, 0.20);
    pnh_.param("return_home/speed_tolerance", speed_tolerance_, 0.15);
    pnh_.param("return_home/stable_time", stable_time_, 1.5);
    // 末段轨迹结束后多久还没到家就重新规划（s）
    pnh_.param("return_home/arrival_timeout", arrival_timeout_, 5.0);
    pnh_.param("data_timeout", data_timeout_, 0.5);            // 里程计、控制器状态新鲜度（s）
    pnh_.param("status_log_period", status_log_period_, 2.0);  // 终端状态行打印周期（s）
    pnh_.param<std::string>("world_frame", world_frame_, "world");
    // 任务授权标识，规划器、bridge 和控制器靠它匹配任务，不要修改
    pnh_.param<std::string>("source_id", source_id_, "fuel_planner");
    // 多会话地图复用：session=fresh/save/load。任务结束（停止/返航到家/落地）时
    // 保存栅格地图（occ_map.bin）与 LTAOM 点云先验，下次 session=load 时复用。
    pnh_.param<std::string>("session", session_, "fresh");
    pnh_.param<std::string>("multisession/map_save_topic", map_save_topic_, "/save_occ_map");
    pnh_.param<std::string>("multisession/prior_save_param", prior_save_param_, "/save_prior_info");
    pnh_.param<std::string>("multisession/descriptor_save_param", descriptor_save_param_,
                            "/save_prior_descriptor");
    pnh_.param("multisession/ground_confirm_time", ground_confirm_time_, 1.0);
    if (!std::isfinite(ground_confirm_time_) || ground_confirm_time_ < 0.0) {
      throw std::invalid_argument("multisession/ground_confirm_time must be finite and >= 0");
    }
    if (session_ != "fresh" && session_ != "save" && session_ != "load") {
      throw std::invalid_argument("session must be one of fresh, save or load");
    }
    if (!(battery_threshold_ >= 0.0 && battery_threshold_ <= 1.0) ||
        battery_timeout_ <= 0.0 || initial_plan_timeout_ <= 0.0 ||
        planner_status_timeout_ <= 0.0 ||
        !std::isfinite(return_retry_interval_) ||
        !std::isfinite(return_planning_timeout_) || return_retry_interval_ <= 0.0 ||
        return_planning_timeout_ <= 0.0 ||
        position_tolerance_ <= 0.0 || speed_tolerance_ <= 0.0 || stable_time_ <= 0.0 ||
        !std::isfinite(arrival_timeout_) || arrival_timeout_ <= 0.0 ||
        data_timeout_ <= 0.0 || !std::isfinite(status_log_period_) ||
        status_log_period_ <= 0.0 || source_id_.empty()) {
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
    save_map_pub_ = nh_.advertise<std_msgs::Empty>(map_save_topic_, 1, true);
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
    saved_maps_ = false;
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
    off_home_since_ = ros::Time();
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

  // 任务结束时持久化两类地图，供下一次飞行续用。栅格地图由 MapROS 在保存话题上
  // 写入；LTAOM 点云先验由 loop_optimization/std_loop 在对应参数翻转为 true 时写入
  // （对应节点未运行时均为无操作）。
  void saveSessionMaps() {
    if (session_ == "fresh" || saved_maps_) return;
    saved_maps_ = true;
    std_msgs::Empty signal;
    save_map_pub_.publish(signal);
    // 两个独立触发：loop_optimization 写关键帧位姿+点云，std_loop 写描述子+面片。
    nh_.setParam(prior_save_param_, true);
    nh_.setParam(descriptor_save_param_, true);
    ROS_INFO("Multi-session maps saved (session=%s, occupancy topic=%s, prior params=%s,%s)",
             session_.c_str(), map_save_topic_.c_str(), prior_save_param_.c_str(),
             descriptor_save_param_.c_str());
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
    logFlightStatus(msg);
  }

  // One console line: mission | FUEL planner | controller | battery. Normal
  // fields stay short; abnormal ones carry their reason and the line turns
  // into a warning, so a stalled exploration points at the failing layer.
  void logFlightStatus(const uav_mission_msgs::MissionStatus& status) {
    using Planner = uav_mission_msgs::PlannerStatus;
    const ros::Time now = status.header.stamp;
    const bool active = state_ == uav_mission_msgs::MissionStatus::EXPLORING || returning();

    std::ostringstream mission;
    mission << missionStateName(status.state);
    if (status.return_reason != uav_mission_msgs::MissionStatus::REASON_NONE)
      mission << " reason=" << missionReasonName(status.return_reason);
    const bool recovering = status.recovery_reason != uav_mission_msgs::MissionStatus::REASON_NONE;
    if (recovering)
      mission << " recovery=" << missionReasonName(status.recovery_reason)
              << " x" << status.recovery_attempts;

    // FUEL alternates PLANNING/TRAJECTORY_READY/EXECUTING on every replan;
    // they share one change class so normal replans do not print each time.
    std::ostringstream fuel;
    std::string fuel_class;
    bool fuel_bad = false;
    if (planner_stamp_.isZero()) {
      const bool overdue = active && !fresh(task_started_, planner_status_timeout_);
      fuel << (!active ? "IDLE" : overdue ? "NO_DATA" : "WAITING");
      fuel_class = fuel.str();
      fuel_bad = overdue;
    } else {
      const uint8_t s = planner_.state;
      const bool running = s == Planner::PLANNING || s == Planner::TRAJECTORY_READY ||
                           s == Planner::EXECUTING;
      fuel << plannerStateName(s);
      fuel_class = running ? "RUNNING" : plannerStateName(s);
      if (s == Planner::FAILED) fuel << " x" << planner_.consecutive_failures;
      fuel_bad = s == Planner::FAILED || s == Planner::OUTPUT_TIMEOUT ||
                 s == Planner::TRAJECTORY_UNSAFE;
      if (fuel_bad) fuel << " (" << planner_.detail << ")";
      if (active && !fresh(planner_stamp_, planner_status_timeout_)) {
        fuel << " STALE";
        fuel_class += "/STALE";
        fuel_bad = true;
      }
    }

    const bool control_fresh = fresh(control_stamp_, data_timeout_);
    std::ostringstream ctrl;
    bool ctrl_bad = !control_fresh;
    if (!control_fresh) {
      ctrl << "NO_DATA";
    } else {
      ctrl << control_.phase;
      if (control_.output_policy != "CLOSED_LOOP") ctrl << '/' << control_.output_policy;
      if (control_.phase != "TRAJECTORY") ctrl << '(' << control_.transition_reason << ')';
      if (control_.landing_state != "IN_AIR") ctrl << " landing=" << control_.landing_state;
      if (!control_.feedback_ready) ctrl << " feedback=NOT_READY";
      ctrl_bad = !control_.feedback_ready ||
                 (active && control_.output_policy != "CLOSED_LOOP");
    }
    const std::string ctrl_key = control_fresh
        ? control_.phase + '/' + control_.output_policy + '/' + control_.landing_state + '/' +
          (control_.feedback_ready ? "1" : "0")
        : std::string("NO_DATA");

    std::ostringstream battery;
    if (status.battery_valid)
      battery << std::fixed << std::setprecision(1) << 100.0 * status.battery_percentage << '%';
    else
      battery << "NO_DATA";
    const bool battery_bad = battery_enabled_ && !status.battery_valid;

    std::ostringstream key;
    key << unsigned(status.state) << '/' << unsigned(status.return_reason) << '/'
        << unsigned(status.recovery_reason) << '/' << status.context.mission_id << '/'
        << fuel_class << '/' << ctrl_key << '/' << status.battery_valid;
    const bool changed = key.str() != last_log_key_;
    if (!changed && !last_log_.isZero() && now >= last_log_ &&
        (now - last_log_).toSec() < status_log_period_) return;

    std::ostringstream line;
    line << "[FLIGHT_STATUS] mission=" << mission.str() << " | fuel=" << fuel.str()
         << " | ctrl=" << ctrl.str() << " | battery=" << battery.str();
    if (fuel_bad || ctrl_bad || battery_bad || recovering)
      ROS_WARN_STREAM(line.str());
    else
      ROS_INFO_STREAM(line.str());
    last_log_key_ = key.str();
    last_log_ = now;
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
    const bool changed = state_ != uav_mission_msgs::MissionStatus::RETURN_RECOVERY ||
                         recovery_reason_ != reason;
    state_ = uav_mission_msgs::MissionStatus::RETURN_RECOVERY;
    recovery_reason_ = reason;
    replan_requested_ = replan_requested_ || request_replan;
    publishStatus(detail);
    // Repeats while still recovering are covered by the status line.
    if (changed) ROS_WARN("Return recovery (same batch, retrying): %s", detail.c_str());
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
      saveSessionMaps();
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
    // 跟踪离地/落地状态，使手动降落（QGC land、AUTO.LAND 或任意下降）即便从未调用
    // /fuel/stop 也仍会保存地图。落地状态做去抖：单次 EKF 抖动不会结束本次会话。
    const ros::Time now = ros::Time::now();
    if (control_.landing_state == "IN_AIR" || control_.landing_state == "LANDING") {
      was_airborne_ = true;
      ground_since_ = ros::Time();
    } else if (control_.landing_state == "ON_GROUND" && was_airborne_) {
      if (ground_since_.isZero() || now < ground_since_) {
        ground_since_ = now;
      } else if ((now - ground_since_).toSec() >= ground_confirm_time_) {
        landed_ = true;
      }
    } else {
      ground_since_ = ros::Time();
    }
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
    planner_ = *msg;
    if (returning()) {
      reference_settled_ = last_active_trajectory_ != 0 &&
                           msg->state == uav_mission_msgs::PlannerStatus::GOAL_REACHED &&
                           msg->trajectory_id == last_active_trajectory_;
      if (reference_settled_) {
        settled_stamp_ = planner_stamp_;
      } else {
        // A withdrawn arrival restarts the whole stable time. The timer alone would miss a
        // withdrawal and a new confirmation that both arrive between two of its ticks.
        stable_since_ = ros::Time();
      }
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
      off_home_since_ = ros::Time();
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
    // 落地即结束本次会话（无论由谁触发），在继续前持久化地图；先停掉 FUEL，避免
    // 飞行器在地面时仍被持续下发轨迹。
    if (landed_ && state_ != uav_mission_msgs::MissionStatus::READY &&
        state_ != uav_mission_msgs::MissionStatus::HOME_HOLD) {
      landed_ = false;
      was_airborne_ = false;
      publishCommand(uav_mission_msgs::MissionCommand::STOP);
      saveSessionMaps();
      state_ = uav_mission_msgs::MissionStatus::READY;
      stable_since_ = ros::Time();
      publishStatus("landed; maps saved");
      ROS_INFO("Landing detected: exploration stopped and session maps saved");
    }
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
        off_home_since_ = ros::Time();
        if (stable_since_.isZero()) stable_since_ = now;
        if ((now - stable_since_).toSec() >= stable_time_) {
          state_ = uav_mission_msgs::MissionStatus::HOME_HOLD;
          publishCommand(uav_mission_msgs::MissionCommand::HOLD_HOME);
          saveSessionMaps();
          publishStatus("MISSION_DONE: home reached and stable; holding home position");
        }
      } else {
        stable_since_ = ros::Time();
        // FUEL does not replan a terminal segment by itself. If it finished with the aircraft
        // elsewhere (e.g. the controller fell back to HOLD and ignores the same trajectory id),
        // only a new trajectory from the measured state gets it moving again.
        if (!reference_settled_ || !fresh(settled_stamp_, 1.0)) {
          off_home_since_ = ros::Time();
        } else if (off_home_since_.isZero()) {
          off_home_since_ = now;
        } else if ((now - off_home_since_).toSec() >= arrival_timeout_) {
          off_home_since_ = ros::Time();
          recoverReturn(uav_mission_msgs::MissionStatus::TRAJECTORY_TIMEOUT,
                        "terminal trajectory finished but home was not reached", true);
        }
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
  ros::Publisher command_pub_, authorization_pub_, status_pub_, save_map_pub_;
  ros::Subscriber odom_sub_, control_sub_, battery_sub_, planner_sub_, trajectory_sub_,
      goal_sub_;
  ros::ServiceServer start_srv_, stop_srv_, return_srv_;
  ros::Timer timer_;
  nav_msgs::Odometry odom_;
  uav_controller::FlightStatus control_;
  uav_mission_msgs::PlannerStatus planner_;  // Latest status of the current task.
  uav_mission_msgs::TaskContext context_;
  geometry_msgs::Point home_;
  ros::Time control_stamp_, battery_stamp_, task_started_, planner_stamp_, stable_since_, last_status_;
  ros::Time off_home_since_;
  double arrival_timeout_{5.0};
  std::string world_frame_, source_id_, detail_{"ready"};
  std::string session_{"fresh"}, map_save_topic_{"/save_occ_map"},
      prior_save_param_{"/save_prior_info"},
      descriptor_save_param_{"/save_prior_descriptor"};
  std::string last_log_key_;
  ros::Time last_log_;
  double status_log_period_{2.0};
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
  bool saved_maps_{false};
  bool was_airborne_{false}, landed_{false};
  ros::Time ground_since_;
  double ground_confirm_time_{1.0};
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
