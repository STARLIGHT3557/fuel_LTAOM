#include "uav_controller/flight_controller.hpp"
#include "uav_controller/feedback_time.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace uav_controller {
namespace {
// 起转结束时的油门占估计悬停油门的比例。开环阶段没有反馈，留出余量：
// 悬停油门初值偏高 25% 以内也不会在开环阶段离地。
constexpr double kSpoolEndHoverFraction = 0.8;
// 起转中高度上升超过此值（m）说明已经离地，立即转闭环。
constexpr double kSpoolLiftoffRise = 0.1;
// 起飞阶段高度上升达到此值（m）才算真正离地，可以开始估计油门。
constexpr double kEstimateLiftoffRise = 0.2;

double yawOf(const Eigen::Quaterniond &q) {
  return std::atan2(2.0 * (q.w() * q.z() + q.x() * q.y()),
                    1.0 - 2.0 * (q.y() * q.y() + q.z() * q.z()));
}
bool positive(double v) { return std::isfinite(v) && v > 0.0; }
bool nonnegative(double v) { return std::isfinite(v) && v >= 0.0; }
bool finite(const ControllerSetpoint &sp) {
  return sp.pos.allFinite() && sp.vel.allFinite() && sp.acc.allFinite() &&
         std::isfinite(sp.yaw);
}
bool finite(const ControllerState &state) {
  return state.pos.allFinite() && state.vel.allFinite() &&
         state.quat.coeffs().allFinite() && state.quat.norm() > 1e-6;
}
}  // namespace

const char *phaseName(FlightPhase phase) {
  switch (phase) {
    case FlightPhase::WAIT: return "WAIT";
    case FlightPhase::TAKEOFF: return "TAKEOFF";
    case FlightPhase::HOVER_BEFORE: return "HOVER_BEFORE";
    case FlightPhase::TRAJECTORY: return "TRAJECTORY";
    case FlightPhase::HOLD: return "HOLD";
    case FlightPhase::FEEDBACK_LOST: return "FEEDBACK_LOST";
  }
  return "UNKNOWN";
}

const char *modeName(FlightMode mode) {
  switch (mode) {
    case FlightMode::WAIT: return "WAIT";
    case FlightMode::ACTIVE: return "ACTIVE";
    case FlightMode::HOLD: return "HOLD";
    case FlightMode::FEEDBACK_LOST: return "FEEDBACK_LOST";
  }
  return "UNKNOWN";
}

const char *stageName(MissionStage stage) {
  switch (stage) {
    case MissionStage::NONE: return "NONE";
    case MissionStage::TAKEOFF: return "TAKEOFF";
    case MissionStage::HOVER_BEFORE: return "HOVER_BEFORE";
    case MissionStage::TRAJECTORY: return "TRAJECTORY";
  }
  return "UNKNOWN";
}

const char *landingStateName(LandingState state) {
  switch (state) {
    case LandingState::UNKNOWN: return "UNKNOWN";
    case LandingState::ON_GROUND: return "ON_GROUND";
    case LandingState::IN_AIR: return "IN_AIR";
    case LandingState::TAKING_OFF: return "TAKING_OFF";
    case LandingState::LANDING: return "LANDING";
  }
  return "UNKNOWN";
}

const char *policyName(OutputPolicy policy) {
  switch (policy) {
    case OutputPolicy::NO_OUTPUT: return "NO_OUTPUT";
    case OutputPolicy::ZERO_THRUST: return "ZERO_THRUST";
    case OutputPolicy::CLOSED_LOOP: return "CLOSED_LOOP";
  }
  return "UNKNOWN";
}

const char *reasonName(TransitionReason reason) {
  switch (reason) {
    case TransitionReason::INITIAL: return "INITIAL";
    case TransitionReason::DISARMED: return "DISARMED";
    case TransitionReason::OFFBOARD_EXIT: return "OFFBOARD_EXIT";
    case TransitionReason::FEEDBACK_INVALID: return "FEEDBACK_INVALID";
    case TransitionReason::FEEDBACK_RECOVERED: return "FEEDBACK_RECOVERED";
    case TransitionReason::CLOCK_DISCONTINUITY: return "CLOCK_DISCONTINUITY";
    case TransitionReason::MISSION_INTERRUPTED: return "MISSION_INTERRUPTED";
    case TransitionReason::AIRBORNE_ENTRY: return "AIRBORNE_ENTRY";
    case TransitionReason::TAKEOFF_STARTED: return "TAKEOFF_STARTED";
    case TransitionReason::TAKEOFF_COMPLETE: return "TAKEOFF_COMPLETE";
    case TransitionReason::HOVER_COMPLETE: return "HOVER_COMPLETE";
    case TransitionReason::TRAJECTORY_COMPLETE: return "TRAJECTORY_COMPLETE";
    case TransitionReason::TAKEOFF_TIMEOUT: return "TAKEOFF_TIMEOUT";
    case TransitionReason::HOVER_TIMEOUT: return "HOVER_TIMEOUT";
    case TransitionReason::TRAJECTORY_TIMEOUT: return "TRAJECTORY_TIMEOUT";
    case TransitionReason::NONFINITE_COMMAND: return "NONFINITE_COMMAND";
    case TransitionReason::POSITION_STEP_STARTED: return "POSITION_STEP_STARTED";
    case TransitionReason::EXTERNAL_TRAJECTORY_STARTED: return "EXTERNAL_TRAJECTORY_STARTED";
  }
  return "UNKNOWN";
}

FlightController::FlightController(const ControllerParams &control,
                                   const FlightParams &flight,
                                   std::uint64_t mission_seed)
    : controller_(control), params_(flight), mission_id_(mission_seed) {
  if (!flight.step_target.allFinite() ||
      !std::isfinite(flight.takeoff_height) || !positive(flight.takeoff_speed) ||
      !nonnegative(flight.spool_time) || !nonnegative(flight.hover_before) ||
      !positive(flight.position_tolerance) || !positive(flight.speed_tolerance) ||
      !positive(flight.stable_time) || !positive(flight.trajectory_timeout) ||
      !positive(flight.takeoff_timeout) ||
      !positive(flight.hover_timeout) ||
      !nonnegative(flight.timestamp_future_tolerance) || !positive(flight.feedback_recovery_time) ||
      !positive(control.hover_percentage) ||
      control.hover_percentage > 1.0 || !positive(control.gravity) ||
      !positive(control.max_a_fb) || !control.k_pos.allFinite() ||
      !control.k_vel.allFinite() || (control.k_pos.array() < 0).any() ||
      (control.k_vel.array() < 0).any()) {
    throw std::invalid_argument("invalid flight/controller parameters");
  }
}

FlightPhase FlightController::phase() const {
  switch (mode_) {
    case FlightMode::WAIT: return FlightPhase::WAIT;
    case FlightMode::HOLD: return FlightPhase::HOLD;
    case FlightMode::FEEDBACK_LOST: return FlightPhase::FEEDBACK_LOST;
    case FlightMode::ACTIVE:
      switch (stage_) {
        case MissionStage::TAKEOFF: return FlightPhase::TAKEOFF;
        case MissionStage::HOVER_BEFORE: return FlightPhase::HOVER_BEFORE;
        case MissionStage::TRAJECTORY: return FlightPhase::TRAJECTORY;
        case MissionStage::NONE: break;
      }
  }
  throw std::logic_error("active flight requires a mission stage");
}

void FlightController::ConditionTimer::reset() {
  since = std::numeric_limits<double>::quiet_NaN();
}

bool FlightController::ConditionTimer::update(bool condition, double duration, double now) {
  if (!condition || !std::isfinite(now)) {
    reset();
    return false;
  }
  if (!std::isfinite(since) || now < since) since = now;
  return now - since >= duration;
}

void FlightController::changeMode(FlightMode mode, TransitionReason reason,
                                  const FlightInput &input, double now) {
  transitionTo(mode, MissionStage::NONE, reason, input, now);
}

void FlightController::changeStage(MissionStage stage, TransitionReason reason,
                                   const FlightInput &input, double now) {
  transitionTo(FlightMode::ACTIVE, stage, reason, input, now);
}

void FlightController::transitionTo(FlightMode mode, MissionStage stage,
                                    TransitionReason reason, const FlightInput &input,
                                    double now) {
  if ((mode == FlightMode::ACTIVE) != (stage != MissionStage::NONE))
    throw std::logic_error("invalid flight mode/stage combination");
  if (mode_ == mode && stage_ == stage) return;

  const auto previous = phase();
  mode_ = mode;
  stage_ = stage;
  entered_at_ = now;
  stable_timer_.reset();
  transition_ = {transition_.sequence + 1, previous, phase(), reason, now};

  // All entry actions live here. Holding the same state does not reinitialize it.
  switch (mode_) {
    case FlightMode::WAIT:
      lockCurrentPosition(input.state);
      controller_.resetThrustMapping();
      break;
    case FlightMode::HOLD:
      if (reason == TransitionReason::POSITION_STEP_STARTED) {
        reference_.pos = params_.step_target;
        reference_.vel.setZero();
        reference_.acc.setZero();
        reference_.yaw = 0.0;  // Fixed local ENU heading for the step test.
        // Pre-OFFBOARD outputs were not applied by the FCU; discard estimator history.
        controller_.resetThrustMapping();
      } else if (reason == TransitionReason::TRAJECTORY_COMPLETE) {
        // Normal completion holds the commanded endpoint and yaw, even if the
        // aircraft still has tracking error. Faults/takeovers lock actual pose.
        reference_.vel.setZero();
        reference_.acc.setZero();
      } else {
        lockCurrentPosition(input.state);
      }
      mission_started_this_arm_ = true;  // Also latch an airborne takeover.
      break;
    case FlightMode::FEEDBACK_LOST:
      controller_.resetThrustMapping();
      break;
    case FlightMode::ACTIVE:
      switch (stage_) {
        case MissionStage::TAKEOFF:
          lockCurrentPosition(input.state);
          takeoff_start_ = input.state.pos;
          origin_ = reference_;
          origin_.pos.z() = params_.takeoff_height;
          takeoff_duration_ = 1.875 * (params_.takeoff_height - takeoff_start_.z()) /
                              params_.takeoff_speed;
          saw_airborne_ = false;
          mission_started_this_arm_ = true;
          spooling_ = params_.spool_time > 0.0;
          takeoff_profile_start_ = now;
          break;
        case MissionStage::HOVER_BEFORE:
          reference_ = origin_;
          break;
        case MissionStage::TRAJECTORY:
          if (params_.direct_trajectory) {
            manager_epoch_ = input.authorization.manager_epoch;
            mission_id_ = input.authorization.mission_id;
            task_type_ = input.authorization.task_type;
            source_id_ = input.authorization.source_id;
            trajectory_id_ = 0;
          } else {
            ++mission_id_;
          }
          trajectory_received_ = false;
          last_trajectory_stamp_ = -1.0;
          break;
        case MissionStage::NONE: break;
      }
      break;
  }
}

void FlightController::lockCurrentPosition(const ControllerState &state) {
  reference_.pos = state.pos;
  reference_.vel.setZero();
  reference_.acc.setZero();
  reference_.yaw = yawOf(state.quat);
}

bool FlightController::atReference(const ControllerState &state) const {
  return (state.pos - reference_.pos).norm() < params_.position_tolerance &&
         state.vel.norm() < params_.speed_tolerance;
}

bool FlightController::updateFeedbackHealth(const FlightInput &input, double now, double dt) {
  if (!input.feedback_valid || !finite(input.state) || !std::isfinite(now)) {
    recovery_pending_ = true;
    recovery_timer_.reset();
    if (mode_ != FlightMode::WAIT)
      changeMode(FlightMode::FEEDBACK_LOST, TransitionReason::FEEDBACK_INVALID, input, now);
    return false;
  }
  if (!recovery_pending_) return true;
  if (dt < 0.0) recovery_timer_.reset();
  if (!recovery_timer_.update(true, params_.feedback_recovery_time, now)) return false;
  recovery_pending_ = false;
  recovery_timer_.reset();
  return true;
}

bool FlightController::authorizedTrajectory(const FlightInput &input,
                                            const TrajectoryPoint &point) const {
  const auto &auth = input.authorization;
  return auth.valid && auth.manager_epoch != 0 && auth.mission_id != 0 &&
         auth.task_type != 0 && !auth.source_id.empty() &&
         point.manager_epoch == auth.manager_epoch && point.mission_id == auth.mission_id &&
         point.task_type == auth.task_type && point.source_id == auth.source_id &&
         point.trajectory_id != 0 &&
         (manager_epoch_ != auth.manager_epoch || mission_id_ != auth.mission_id ||
          trajectory_id_ == 0 || point.trajectory_id >= trajectory_id_);
}

bool FlightController::manageFlight(const FlightInput &input, double now, double dt) {
  // Priority: confirmed disarm -> authority -> recovery ->
  // control-clock fault -> task admission. Feedback health is checked before all.
  if (!input.armed) {
    changeMode(FlightMode::WAIT, TransitionReason::DISARMED, input, now);
    saw_airborne_ = false;
    mission_started_this_arm_ = false;
    lockCurrentPosition(input.state);  // WAIT may follow observations, without re-entry.
    return false;
  }
  if (input.landing_state == LandingState::IN_AIR || input.landing_state == LandingState::LANDING)
    saw_airborne_ = true;

  if (!input.offboard) {
    changeMode(FlightMode::WAIT, TransitionReason::OFFBOARD_EXIT, input, now);
    lockCurrentPosition(input.state);
    return false;
  }
  if (mode_ == FlightMode::FEEDBACK_LOST) {
    changeMode(FlightMode::HOLD, TransitionReason::FEEDBACK_RECOVERED, input, now);
    return false;
  }
  if (dt < 0.0) {
    // A new clock epoch invalidates dwell measurements even if already in HOLD.
    // This is an explicit reset, not a same-state transition.
    stable_timer_.reset();
    lockCurrentPosition(input.state);
    changeMode(FlightMode::HOLD, TransitionReason::CLOCK_DISCONTINUITY, input, now);
    return false;
  }
  if (mode_ == FlightMode::WAIT) {
    lockCurrentPosition(input.state);
    if (params_.position_step) {
      // No automatic takeoff or trajectory handshake. Each airborne OFFBOARD
      // entry applies the full step in this update, even after an earlier test.
      if (input.landing_state == LandingState::IN_AIR)
        changeMode(FlightMode::HOLD, TransitionReason::POSITION_STEP_STARTED, input, now);
      return false;
    }
    if (mission_started_this_arm_) {
      changeMode(FlightMode::HOLD, TransitionReason::MISSION_INTERRUPTED, input, now);
    } else if (input.landing_state == LandingState::UNKNOWN) {
      return false;
    } else if (input.landing_state != LandingState::ON_GROUND || saw_airborne_) {
      changeMode(FlightMode::HOLD, TransitionReason::AIRBORNE_ENTRY, input, now);
    } else if (params_.takeoff_height > input.state.pos.z() + params_.position_tolerance) {
      changeStage(MissionStage::TAKEOFF, TransitionReason::TAKEOFF_STARTED, input, now);
    }
    return false;
  }
  if (params_.direct_trajectory && mode_ == FlightMode::HOLD) {
    const auto &point = input.trajectory;
    const bool new_trajectory = manager_epoch_ != point.manager_epoch ||
                                mission_id_ != point.mission_id ||
                                task_type_ != point.task_type || source_id_ != point.source_id ||
                                point.trajectory_id > trajectory_id_;
    const bool fresh = point.valid && !point.finished && new_trajectory &&
                       authorizedTrajectory(input, point) && finite(point.reference) &&
                       freshTimestamp(point.stamp, now, params_.trajectory_timeout,
                                      params_.timestamp_future_tolerance);
    if (fresh) {
      changeStage(MissionStage::TRAJECTORY,
                  TransitionReason::EXTERNAL_TRAJECTORY_STARTED, input, now);
    }
  }
  return mode_ == FlightMode::ACTIVE;
}

void FlightController::advanceMission(const FlightInput &input, double now) {
  const auto current_stage = stage_;
  switch (stage_) {
    case MissionStage::TAKEOFF: {
      // 起转时间到，或高度已经上升（初值偏高提前离地），转闭环并开始计时起飞轨迹。
      if (spooling_ && (now - entered_at_ >= params_.spool_time ||
                        input.state.pos.z() - takeoff_start_.z() > kSpoolLiftoffRise)) {
        spooling_ = false;
        takeoff_profile_start_ = now;
      }
      // Quintic position profile; velocity/acceleration are its derivatives.
      // 起转期间 s=0，参考保持在起飞点。
      const double s = spooling_
          ? 0.0 : std::clamp((now - takeoff_profile_start_) / takeoff_duration_, 0.0, 1.0);
      const double s2 = s * s, s3 = s2 * s;
      const double h = params_.takeoff_height - takeoff_start_.z();
      reference_.pos.z() = takeoff_start_.z() + h * s3 * (10.0 - 15.0*s + 6.0*s2);
      reference_.vel.z() = h / takeoff_duration_ * 30.0*s2*(1.0-s)*(1.0-s);
      reference_.acc.z() = h / (takeoff_duration_*takeoff_duration_) *
                            60.0*s*(1.0-s)*(1.0-2.0*s);
      if (stable_timer_.update(s >= 1.0 && saw_airborne_ && atReference(input.state),
                               params_.stable_time, now))
        changeStage(MissionStage::HOVER_BEFORE, TransitionReason::TAKEOFF_COMPLETE, input, now);
      break;
    }
    case MissionStage::HOVER_BEFORE: {
      if (stable_timer_.update(atReference(input.state),
                               std::max(params_.stable_time, params_.hover_before), now))
        changeStage(MissionStage::TRAJECTORY, TransitionReason::HOVER_COMPLETE, input, now);
      break;
    }
    case MissionStage::TRAJECTORY: {
      const auto &point = input.trajectory;
      const bool mission_matches = params_.direct_trajectory
          ? authorizedTrajectory(input, point) : point.mission_id == mission_id_;
      const bool started_after_entry = params_.direct_trajectory || point.stamp >= entered_at_;
      const bool fresh = point.valid && mission_matches && started_after_entry &&
                          freshTimestamp(point.stamp, now, params_.trajectory_timeout,
                                         params_.timestamp_future_tolerance) &&
                          point.stamp >= last_trajectory_stamp_ && finite(point.reference);
      if (fresh) {
        if (params_.direct_trajectory) {
          manager_epoch_ = point.manager_epoch;
          mission_id_ = point.mission_id;
          task_type_ = point.task_type;
          source_id_ = point.source_id;
          trajectory_id_ = point.trajectory_id;
        }
        reference_ = point.reference;
        last_trajectory_stamp_ = point.stamp;
        trajectory_received_ = true;
        if (point.finished && reference_.vel.norm() < 1e-3 && reference_.acc.norm() < 1e-3)
          changeMode(FlightMode::HOLD, TransitionReason::TRAJECTORY_COMPLETE, input, now);
      } else if (trajectory_received_ && now - last_trajectory_stamp_ > params_.trajectory_timeout) {
        changeMode(FlightMode::HOLD, TransitionReason::TRAJECTORY_TIMEOUT, input, now);
      }
      break;
    }
    case MissionStage::NONE: break;
  }
  // Completion wins at the deadline. Entry actions initialize the new stage,
  // which is advanced only on the next update (one normal transition per tick).
  if (mode_ == FlightMode::ACTIVE && stage_ == current_stage) checkStageTimeout(input, now);
}

void FlightController::checkStageTimeout(const FlightInput &input, double now) {
  double timeout = 0.0;
  TransitionReason reason = TransitionReason::INITIAL;
  switch (stage_) {
    case MissionStage::TAKEOFF:
      timeout = params_.takeoff_timeout;
      reason = TransitionReason::TAKEOFF_TIMEOUT;
      break;
    case MissionStage::HOVER_BEFORE:
      timeout = params_.hover_timeout;
      reason = TransitionReason::HOVER_TIMEOUT;
      break;
    case MissionStage::TRAJECTORY:
    case MissionStage::NONE: return;
  }
  if (now - entered_at_ >= timeout)
    changeMode(FlightMode::HOLD, reason, input, now);
}

OutputPolicy FlightController::selectOutputPolicy(const FlightInput &input, bool feedback_ready) const {
  if (!feedback_ready) return OutputPolicy::NO_OUTPUT;
  if (!input.armed) return OutputPolicy::ZERO_THRUST;
  const bool ground = input.landing_state == LandingState::ON_GROUND;
  const bool unknown = input.landing_state == LandingState::UNKNOWN;
  switch (mode_) {
    case FlightMode::WAIT:
      if (unknown) return OutputPolicy::NO_OUTPUT;
      return ground ? OutputPolicy::ZERO_THRUST : OutputPolicy::CLOSED_LOOP;
    case FlightMode::HOLD:
      if (ground) return OutputPolicy::ZERO_THRUST;
      if (unknown && !saw_airborne_) return OutputPolicy::NO_OUTPUT;
      return OutputPolicy::CLOSED_LOOP;
    case FlightMode::ACTIVE:
      return OutputPolicy::CLOSED_LOOP;
    case FlightMode::FEEDBACK_LOST: return OutputPolicy::NO_OUTPUT;
  }
  return OutputPolicy::NO_OUTPUT;
}

// 开环起转：油门从 0 线性升到 kSpoolEndHoverFraction × 估计悬停油门。姿态设定为
// 当前姿态，不走位置环，地面不平时也不会产生差动推力。
AttitudeThrustCommand FlightController::spoolCommand(const ControllerState &state,
                                                     double now) const {
  const double progress = std::clamp((now - entered_at_) / params_.spool_time, 0.0, 1.0);
  const double hover = controller_.params().gravity / controller_.thr2acc();
  AttitudeThrustCommand command;
  command.q = state.quat;
  command.thrust = progress * kSpoolEndHoverFraction * hover;
  command.thrust_raw = command.thrust;
  return command;
}

FlightOutput FlightController::output(const FlightInput &input, double now, bool feedback_ready) {
  auto policy = selectOutputPolicy(input, feedback_ready);
  AttitudeThrustCommand command;
  if (policy == OutputPolicy::CLOSED_LOOP) {
    if (stage_ == MissionStage::TAKEOFF && spooling_) {
      command = spoolCommand(input.state, now);
    } else {
      // 只在确认离地后估计油门：地面托着飞机时加速度计读数与油门无关，而 PX4 的
      // IN_AIR 可能早于实际离地，所以起飞阶段还要求高度确实上升。
      const bool lifted_off = input.landing_state == LandingState::IN_AIR &&
          (stage_ != MissionStage::TAKEOFF ||
           input.state.pos.z() - takeoff_start_.z() >= kEstimateLiftoffRise);
      controller_.setEstimateEnabled(params_.estimate_thrust && input.imu_valid &&
                                     std::isfinite(input.state.az_body) &&
                                     input.armed && input.offboard && lifted_off);
      command = controller_.computeAttitudeThrust(reference_, input.state, now);
    }
    if (!command.q.coeffs().allFinite() || !std::isfinite(command.thrust) ||
        !std::isfinite(command.thrust_raw)) {
      recovery_pending_ = true;
      recovery_timer_.reset();
      changeMode(FlightMode::FEEDBACK_LOST, TransitionReason::NONFINITE_COMMAND, input, now);
      feedback_ready = false;
      policy = OutputPolicy::NO_OUTPUT;
      command = AttitudeThrustCommand{};
    }
  } else if (policy == OutputPolicy::ZERO_THRUST) {
    command.q = Eigen::Quaterniond(Eigen::AngleAxisd(reference_.yaw, Eigen::Vector3d::UnitZ()));
  }
  if (policy != OutputPolicy::CLOSED_LOOP && last_output_policy_ == OutputPolicy::CLOSED_LOOP)
    controller_.resetThrustMapping();
  last_output_policy_ = policy;

  FlightOutput result;
  result.phase = phase();
  result.mode = mode_;
  result.mission_stage = stage_;
  result.transition = transition_;
  result.output_policy = policy;
  result.feedback_ready = feedback_ready;
  result.phase_elapsed = std::isfinite(now) && std::isfinite(entered_at_)
                             ? std::max(0.0, now - entered_at_) : 0.0;
  result.reference = reference_;
  result.command = command;
  result.publish = policy != OutputPolicy::NO_OUTPUT;
  result.manager_epoch = manager_epoch_;
  result.mission_id = mission_id_;
  result.trajectory_id = trajectory_id_;
  result.task_type = task_type_;
  result.source_id = source_id_;
  result.trajectory_active = mode_ == FlightMode::ACTIVE && stage_ == MissionStage::TRAJECTORY;
  result.trajectory_origin = origin_;
  return result;
}

FlightOutput FlightController::update(const FlightInput &input, double now) {
  const double dt = std::isfinite(last_update_) ? now - last_update_ : 0.0;
  last_update_ = now;
  if (!std::isfinite(entered_at_) && std::isfinite(now)) entered_at_ = now;
  const bool feedback_ready = updateFeedbackHealth(input, now, dt);
  if (feedback_ready && manageFlight(input, now, dt)) advanceMission(input, now);
  return output(input, now, feedback_ready);
}

}  // namespace uav_controller
