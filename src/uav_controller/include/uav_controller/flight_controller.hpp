#pragma once

#include <cstdint>
#include <limits>
#include <string>

#include "uav_controller/geometric_controller.hpp"

namespace uav_controller {

enum class FlightPhase {
  WAIT, TAKEOFF, HOVER_BEFORE, TRAJECTORY, HOLD, FEEDBACK_LOST
};
const char *phaseName(FlightPhase phase);

// FlightPhase remains the public, flattened view used by /fsm_state. Only an
// ACTIVE flight mode owns a mission stage; interruptions discard that stage.
enum class FlightMode { WAIT, ACTIVE, HOLD, FEEDBACK_LOST };
enum class MissionStage { NONE, TAKEOFF, HOVER_BEFORE, TRAJECTORY };
enum class LandingState { UNKNOWN, ON_GROUND, IN_AIR, TAKING_OFF, LANDING };
enum class OutputPolicy { NO_OUTPUT, ZERO_THRUST, CLOSED_LOOP };
enum class TransitionReason {
  INITIAL, DISARMED, OFFBOARD_EXIT, FEEDBACK_INVALID, FEEDBACK_RECOVERED,
  CLOCK_DISCONTINUITY, MISSION_INTERRUPTED, AIRBORNE_ENTRY, TAKEOFF_STARTED,
  TAKEOFF_COMPLETE, HOVER_COMPLETE, TRAJECTORY_COMPLETE,
  TAKEOFF_TIMEOUT, HOVER_TIMEOUT, TRAJECTORY_TIMEOUT,
  NONFINITE_COMMAND, POSITION_STEP_STARTED, EXTERNAL_TRAJECTORY_STARTED
};
const char *modeName(FlightMode mode);
const char *stageName(MissionStage stage);
const char *landingStateName(LandingState state);
const char *policyName(OutputPolicy policy);
const char *reasonName(TransitionReason reason);

struct FlightTransition {
  std::uint64_t sequence{0};
  FlightPhase from{FlightPhase::WAIT};
  FlightPhase to{FlightPhase::WAIT};
  TransitionReason reason{TransitionReason::INITIAL};
  double stamp{0.0};
};

struct FlightParams {
  bool position_step{false};  // Manual takeoff, then an immediate target on OFFBOARD entry.
  // Integration mode for planners such as FUEL: hold after an airborne
  // OFFBOARD entry, accept only manager-authorized task/trajectory batches,
  // and return to HOLD on completion or timeout.
  bool direct_trajectory{false};
  Eigen::Vector3d step_target{0.0, 0.0, 1.0};  // Absolute local ENU position [m].
  double takeoff_height{1.5};  // Absolute local ENU z, NOT an offset from launch.
  double takeoff_speed{0.3};
  double hover_before{5.0};
  double position_tolerance{0.15};
  double speed_tolerance{0.15};
  double stable_time{1.0};
  double trajectory_timeout{0.5};
  double takeoff_timeout{30.0};
  double hover_timeout{30.0};
  double timestamp_future_tolerance{0.05};
  double feedback_recovery_time{0.2};
  bool estimate_thrust{false};
};

struct TrajectoryPoint {
  ControllerSetpoint reference;
  std::uint64_t manager_epoch{0};
  std::uint64_t mission_id{0};
  std::uint64_t trajectory_id{0};
  std::uint8_t task_type{0};
  std::string source_id;
  double stamp{-1.0};
  bool valid{false};
  bool finished{false};
};

struct MissionAuthorization {
  std::uint64_t manager_epoch{0};
  std::uint64_t mission_id{0};
  std::uint8_t task_type{0};
  std::string source_id;
  bool valid{false};
};

struct FlightInput {
  ControllerState state;
  TrajectoryPoint trajectory;
  MissionAuthorization authorization;
  bool feedback_valid{false};  // Fresh finite pose, velocity and connected FCU.
  bool imu_valid{false};
  bool armed{false};
  bool offboard{false};
  // UNKNOWN includes missing, stale and unrecognized landed-state feedback.
  LandingState landing_state{LandingState::UNKNOWN};
};

struct FlightOutput {
  FlightPhase phase{FlightPhase::WAIT};
  FlightMode mode{FlightMode::WAIT};
  MissionStage mission_stage{MissionStage::NONE};
  FlightTransition transition;
  OutputPolicy output_policy{OutputPolicy::NO_OUTPUT};
  bool feedback_ready{false};
  double phase_elapsed{0.0};
  ControllerSetpoint reference;
  AttitudeThrustCommand command;
  bool publish{false};
  bool trajectory_active{false};
  std::uint64_t manager_epoch{0};
  std::uint64_t mission_id{0};
  std::uint64_t trajectory_id{0};
  std::uint8_t task_type{0};
  std::string source_id;
  ControllerSetpoint trajectory_origin;
};

// ROS-free controller module: flight sequencing, reference selection and
// geometric attitude/thrust control. The adapter only supplies observations
// and performs publications requested by the output. Landing and disarming
// remain under operator/FCU control.
class FlightController {
public:
  FlightController(const ControllerParams &control, const FlightParams &flight,
                   std::uint64_t mission_seed = 0);
  FlightOutput update(const FlightInput &input, double now);
  FlightPhase phase() const;

private:
  struct ConditionTimer {
    double since{std::numeric_limits<double>::quiet_NaN()};
    void reset();
    bool update(bool condition, double duration, double now);
  };

  void transitionTo(FlightMode mode, MissionStage stage, TransitionReason reason,
                    const FlightInput &input, double now);
  void changeMode(FlightMode mode, TransitionReason reason,
                  const FlightInput &input, double now);
  void changeStage(MissionStage stage, TransitionReason reason,
                   const FlightInput &input, double now);
  void lockCurrentPosition(const ControllerState &state);
  bool atReference(const ControllerState &state) const;
  bool updateFeedbackHealth(const FlightInput &input, double now, double dt);
  bool manageFlight(const FlightInput &input, double now, double dt);
  bool authorizedTrajectory(const FlightInput &input, const TrajectoryPoint &point) const;
  void advanceMission(const FlightInput &input, double now);
  void checkStageTimeout(const FlightInput &input, double now);
  OutputPolicy selectOutputPolicy(const FlightInput &input, bool feedback_ready) const;
  FlightOutput output(const FlightInput &input, double now, bool feedback_ready);

  GeometricController controller_;
  FlightParams params_;
  FlightMode mode_{FlightMode::WAIT};
  MissionStage stage_{MissionStage::NONE};
  FlightTransition transition_;
  OutputPolicy last_output_policy_{OutputPolicy::NO_OUTPUT};
  ControllerSetpoint reference_;
  ControllerSetpoint origin_;
  Eigen::Vector3d takeoff_start_{Eigen::Vector3d::Zero()};
  double takeoff_duration_{0.0};
  double entered_at_{std::numeric_limits<double>::quiet_NaN()};
  ConditionTimer stable_timer_, recovery_timer_;
  double last_update_{std::numeric_limits<double>::quiet_NaN()};
  double last_trajectory_stamp_{-1.0};
  bool recovery_pending_{false};
  bool trajectory_received_{false};
  bool saw_airborne_{false};
  bool mission_started_this_arm_{false};
  std::uint64_t manager_epoch_{0};
  std::uint64_t mission_id_{0};
  std::uint64_t trajectory_id_{0};
  std::uint8_t task_type_{0};
  std::string source_id_;
};

}  // namespace uav_controller
