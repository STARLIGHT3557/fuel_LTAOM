#include <cmath>
#include <limits>
#include <stdexcept>

#include <gtest/gtest.h>

#include "uav_controller/flight_controller.hpp"
#include "uav_controller/feedback_time.hpp"

namespace uc = uav_controller;
namespace {

uc::ControllerParams controlParams() {
  uc::ControllerParams p;
  p.k_pos << 3.0, 3.0, 4.0;
  p.k_vel << 3.0, 3.0, 3.5;
  p.hover_percentage = 0.65;
  return p;
}
uc::FlightParams flightParams() {
  uc::FlightParams p;
  p.takeoff_height = 1.5;
  p.takeoff_speed = 0.5;
  p.hover_before = 0.2;
  p.stable_time = 0.1;
  return p;
}

// Kinematic reference follower, not a flight-dynamics simulator. It exercises
// controller sequencing without ROS, MAVROS or any connected flight hardware.
class FlightHarness {
public:
  explicit FlightHarness(uc::FlightParams parameters = flightParams())
      : params(parameters), controller(controlParams(), params, 100) {
    input.feedback_valid = true;
    input.landing_state = uc::LandingState::ON_GROUND;
    input.state.pos << 2.0, -3.0, 0.4;
    tick();
  }
  void tick(bool follow = false) {
    if (follow && output.publish) {
      input.state.pos = output.reference.pos;
      input.state.vel = output.reference.vel;
      if (input.state.pos.z() > 0.45) input.landing_state = uc::LandingState::IN_AIR;
    }
    now += 0.01;
    output = controller.update(input, now);
  }
  void engage() {
    input.armed = input.offboard = true;
    tick();
  }
  bool until(uc::FlightPhase phase, int steps = 3000) {
    for (int i = 0; i < steps && output.phase != phase; ++i) tick(true);
    return output.phase == phase;
  }
  void finishTrajectory() {
    input.trajectory.valid = true;
    input.trajectory.mission_id = output.mission_id;
    input.trajectory.stamp = now + 0.01;
    input.trajectory.reference = output.reference;
    input.trajectory.reference.pos.x() += 1.0;
    input.trajectory.finished = true;
    tick();
  }
  uc::FlightParams params;
  uc::FlightController controller;
  uc::FlightInput input;
  uc::FlightOutput output;
  double now{10.0};
};

TEST(FlightController, RequiresArmOffboardAndGroundConfirmation) {
  FlightHarness h;
  EXPECT_EQ(h.output.phase, uc::FlightPhase::WAIT);
  EXPECT_DOUBLE_EQ(h.output.command.thrust, 0.0);
  h.input.offboard = true;
  h.tick();
  EXPECT_EQ(h.output.phase, uc::FlightPhase::WAIT);
  h.input.armed = true;
  h.input.landing_state = uc::LandingState::UNKNOWN;
  h.tick();
  EXPECT_EQ(h.output.phase, uc::FlightPhase::WAIT);
  EXPECT_EQ(h.output.output_policy, uc::OutputPolicy::NO_OUTPUT);
  EXPECT_FALSE(h.output.publish);
  EXPECT_DOUBLE_EQ(h.output.command.thrust, 0.0);
  h.input.landing_state = uc::LandingState::ON_GROUND;
  h.tick();
  EXPECT_EQ(h.output.phase, uc::FlightPhase::TAKEOFF);
}

TEST(FlightController, AbsoluteHeightAndFixedHoverTarget) {
  FlightHarness h;
  h.engage();
  double max_vz = 0.0;
  for (int i = 0; i < 3000 && h.output.phase == uc::FlightPhase::TAKEOFF; ++i) {
    max_vz = std::max(max_vz, h.output.reference.vel.z());
    h.tick(true);
  }
  ASSERT_EQ(h.output.phase, uc::FlightPhase::HOVER_BEFORE);
  EXPECT_DOUBLE_EQ(h.output.reference.pos.z(), 1.5);  // Ground is z=0.4.
  EXPECT_LE(max_vz, h.params.takeoff_speed + 1e-10);
  const Eigen::Vector3d locked = h.output.reference.pos;
  h.input.state.pos += Eigen::Vector3d(0.4, -0.3, 0.2);
  h.input.state.vel.setZero();
  for (int i = 0; i < 100; ++i) h.tick();
  EXPECT_EQ(h.output.phase, uc::FlightPhase::HOVER_BEFORE);
  EXPECT_TRUE(h.output.reference.pos.isApprox(locked));
  h.input.state.pos = locked;
  ASSERT_TRUE(h.until(uc::FlightPhase::TRAJECTORY));
  EXPECT_DOUBLE_EQ(h.output.trajectory_origin.pos.z(), 1.5);
}

TEST(FlightController, RefusesAbsoluteHeightBelowGround) {
  FlightHarness h;
  h.input.state.pos.z() = 2.0;
  h.engage();
  EXPECT_EQ(h.output.phase, uc::FlightPhase::WAIT);
  EXPECT_DOUBLE_EQ(h.output.command.thrust, 0.0);
}

TEST(PositionStep, RequiresManualTakeoffArmingAndOffboard) {
  auto p = flightParams();
  p.position_step = true;
  FlightHarness h(p);
  h.engage();
  for (int i = 0; i < 600; ++i) h.tick();
  EXPECT_EQ(h.output.phase, uc::FlightPhase::WAIT);
  EXPECT_EQ(h.output.output_policy, uc::OutputPolicy::ZERO_THRUST);

  for (auto state : {uc::LandingState::UNKNOWN, uc::LandingState::TAKING_OFF,
                     uc::LandingState::LANDING}) {
    h.input.landing_state = state;
    h.tick();
    EXPECT_EQ(h.output.phase, uc::FlightPhase::WAIT);
  }
  h.input.landing_state = uc::LandingState::IN_AIR;
  h.input.state.pos << 2.0, -1.0, 2.0;
  h.input.armed = false;
  h.tick();
  EXPECT_EQ(h.output.phase, uc::FlightPhase::WAIT);
  h.input.armed = true;
  h.input.offboard = false;
  h.tick();
  EXPECT_EQ(h.output.phase, uc::FlightPhase::WAIT);
  EXPECT_TRUE(h.output.reference.pos.isApprox(h.input.state.pos));
  EXPECT_TRUE(h.output.publish);  // Keep streaming before the manual mode switch.
  h.input.offboard = true;
  h.tick();
  EXPECT_EQ(h.output.phase, uc::FlightPhase::HOLD);
  EXPECT_EQ(h.output.transition.reason, uc::TransitionReason::POSITION_STEP_STARTED);
  EXPECT_TRUE(h.output.reference.pos.isApprox(p.step_target));
}

TEST(PositionStep, FirstOffboardUpdateAppliesFullStepAndHoldsItWithoutTrajectory) {
  auto p = flightParams();
  p.position_step = true;
  FlightHarness h(p);
  h.input.armed = true;
  h.input.landing_state = uc::LandingState::IN_AIR;
  h.input.state.pos << 2.0, -1.0, 2.0;
  h.input.state.vel << 0.1, -0.2, 0.3;
  h.input.state.quat = Eigen::AngleAxisd(0.7, Eigen::Vector3d::UnitZ());
  h.tick();
  ASSERT_EQ(h.output.phase, uc::FlightPhase::WAIT);
  h.input.offboard = true;
  h.tick();
  ASSERT_EQ(h.output.phase, uc::FlightPhase::HOLD);
  EXPECT_EQ(h.output.mission_stage, uc::MissionStage::NONE);
  EXPECT_EQ(h.output.output_policy, uc::OutputPolicy::CLOSED_LOOP);
  EXPECT_FALSE(h.output.trajectory_active);
  EXPECT_TRUE(h.output.reference.pos.isApprox(Eigen::Vector3d(0.0, 0.0, 1.0)));
  EXPECT_TRUE(h.output.reference.vel.isZero());
  EXPECT_TRUE(h.output.reference.acc.isZero());
  EXPECT_DOUBLE_EQ(h.output.reference.yaw, 0.0);
  // The same update must use the new target in the actual attitude/thrust law.
  uc::ControllerSetpoint target;
  target.pos = p.step_target;
  target.yaw = 0.0;
  uc::GeometricController direct(controlParams());
  const auto expected = direct.computeAttitudeThrust(target, h.input.state, h.now);
  EXPECT_NEAR(h.output.command.q.angularDistance(expected.q), 0.0, 1e-7);
  EXPECT_NEAR(h.output.command.thrust, expected.thrust, 1e-12);

  h.input.state.pos << 3.0, -2.0, 1.8;
  h.input.state.quat = Eigen::Quaterniond::Identity();
  h.input.trajectory.valid = true;
  h.input.trajectory.mission_id = h.output.mission_id;
  h.input.trajectory.reference.pos << 9.0, 8.0, 7.0;
  h.input.trajectory.finished = true;
  const auto sequence = h.output.transition.sequence;
  for (int i = 0; i < 4000; ++i) {
    h.input.trajectory.stamp = h.now + 0.01;
    h.tick();
  }
  EXPECT_EQ(h.output.phase, uc::FlightPhase::HOLD);
  EXPECT_EQ(h.output.transition.sequence, sequence);
  EXPECT_TRUE(h.output.publish);
  EXPECT_TRUE(h.output.reference.pos.isApprox(p.step_target));
  EXPECT_TRUE(h.output.reference.vel.isZero());
  EXPECT_TRUE(h.output.reference.acc.isZero());
  EXPECT_DOUBLE_EQ(h.output.reference.yaw, 0.0);
}

TEST(PositionStep, OffboardReentryRepeatsCustomTargetWithoutDisarming) {
  auto p = flightParams();
  p.position_step = true;
  p.step_target << -0.5, 0.7, 1.2;
  FlightHarness h(p);
  h.input.landing_state = uc::LandingState::IN_AIR;
  h.engage();
  ASSERT_TRUE(h.output.reference.pos.isApprox(p.step_target));
  h.input.offboard = false;
  h.input.state.pos << 3.0, -2.0, 2.0;
  h.input.state.quat = Eigen::AngleAxisd(-0.4, Eigen::Vector3d::UnitZ());
  h.tick();
  EXPECT_EQ(h.output.phase, uc::FlightPhase::WAIT);
  EXPECT_TRUE(h.output.reference.pos.isApprox(h.input.state.pos));
  h.input.offboard = true;
  h.tick();
  EXPECT_EQ(h.output.transition.reason, uc::TransitionReason::POSITION_STEP_STARTED);
  EXPECT_TRUE(h.output.reference.pos.isApprox(p.step_target));
  EXPECT_DOUBLE_EQ(h.output.reference.yaw, 0.0);
  EXPECT_TRUE(h.output.reference.vel.isZero());
  EXPECT_TRUE(h.output.reference.acc.isZero());
}

TEST(PositionStep, FeedbackRecoveryHoldsActualPositionUntilAnotherOffboardEntry) {
  auto p = flightParams();
  p.position_step = true;
  FlightHarness h(p);
  h.input.landing_state = uc::LandingState::IN_AIR;
  h.engage();
  h.input.feedback_valid = false;
  h.tick();
  ASSERT_EQ(h.output.phase, uc::FlightPhase::FEEDBACK_LOST);
  EXPECT_FALSE(h.output.publish);
  h.input.feedback_valid = true;
  for (int i = 0; i < 15; ++i) {
    h.input.offboard = !h.input.offboard;
    h.tick();
    EXPECT_FALSE(h.output.publish);
  }
  h.input.offboard = true;
  h.input.state.pos << 1.2, 0.5, 2.0;
  for (int i = 0; i < 10; ++i) h.tick();
  EXPECT_EQ(h.output.transition.reason, uc::TransitionReason::FEEDBACK_RECOVERED);
  EXPECT_TRUE(h.output.reference.pos.isApprox(h.input.state.pos));
  EXPECT_TRUE(h.output.publish);
  for (int i = 0; i < 50; ++i) h.tick();
  EXPECT_TRUE(h.output.reference.pos.isApprox(h.input.state.pos));
  h.input.offboard = false;
  h.tick();
  h.input.offboard = true;
  h.tick();
  EXPECT_TRUE(h.output.reference.pos.isApprox(p.step_target));
}

TEST(PositionStep, ClockResetLocksPositionWhileLongGapKeepsTarget) {
  for (double jump : {-1.0, 1.0}) {
    auto p = flightParams();
    p.position_step = true;
    FlightHarness h(p);
    h.input.landing_state = uc::LandingState::IN_AIR;
    h.engage();
    h.input.state.pos << 0.5, 0.3, 1.8;
    h.now += jump;
    h.tick();
    if (jump < 0.0)
      EXPECT_TRUE(h.output.reference.pos.isApprox(h.input.state.pos));
    else
      EXPECT_TRUE(h.output.reference.pos.isApprox(p.step_target));
    h.input.landing_state = uc::LandingState::ON_GROUND;
    h.tick();
    EXPECT_EQ(h.output.output_policy, uc::OutputPolicy::ZERO_THRUST);
    EXPECT_DOUBLE_EQ(h.output.command.thrust, 0.0);
    h.input.armed = false;
    h.tick();
    EXPECT_EQ(h.output.phase, uc::FlightPhase::WAIT);
    EXPECT_DOUBLE_EQ(h.output.command.thrust, 0.0);
  }
}

TEST(PositionStep, RejectsNonfiniteTarget) {
  for (double value : {std::numeric_limits<double>::quiet_NaN(),
                       std::numeric_limits<double>::infinity()}) {
    for (int axis = 0; axis < 3; ++axis) {
      auto p = flightParams();
      p.position_step = true;
      p.step_target[axis] = value;
      EXPECT_THROW(uc::FlightController(controlParams(), p), std::invalid_argument);
    }
  }
}

TEST(DirectTrajectory, AirborneHoldTracksFinishesAndRestarts) {
  auto p = flightParams();
  p.direct_trajectory = true;
  FlightHarness h(p);
  h.input.armed = true;
  h.input.landing_state = uc::LandingState::IN_AIR;
  h.input.state.pos << 1.0, -2.0, 1.5;
  h.tick();
  ASSERT_EQ(h.output.phase, uc::FlightPhase::WAIT);
  h.input.offboard = true;
  h.tick();
  ASSERT_EQ(h.output.phase, uc::FlightPhase::HOLD);
  EXPECT_TRUE(h.output.reference.pos.isApprox(h.input.state.pos));

  h.input.authorization.valid = true;
  h.input.authorization.manager_epoch = 77;
  h.input.authorization.mission_id = 999;
  h.input.authorization.task_type = 1;
  h.input.authorization.source_id = "fuel_planner";

  h.input.trajectory.valid = true;
  h.input.trajectory.manager_epoch = 77;
  h.input.trajectory.mission_id = 999;
  h.input.trajectory.task_type = 1;
  h.input.trajectory.source_id = "wrong_source";
  h.input.trajectory.trajectory_id = 1;
  h.input.trajectory.stamp = h.now + 0.01;
  h.input.trajectory.reference = h.output.reference;
  h.input.trajectory.reference.pos.x() += 2.0;
  h.tick();
  ASSERT_EQ(h.output.phase, uc::FlightPhase::HOLD);

  h.input.trajectory.source_id = "fuel_planner";
  h.input.trajectory.stamp = h.now + 0.01;
  h.tick();
  ASSERT_EQ(h.output.phase, uc::FlightPhase::TRAJECTORY);
  EXPECT_EQ(h.output.transition.reason, uc::TransitionReason::EXTERNAL_TRAJECTORY_STARTED);
  EXPECT_TRUE(h.output.reference.pos.isApprox(h.input.trajectory.reference.pos));

  h.input.trajectory.finished = true;
  h.input.trajectory.reference.vel.setZero();
  h.input.trajectory.reference.acc.setZero();
  h.input.trajectory.stamp = h.now + 0.01;
  h.tick();
  ASSERT_EQ(h.output.phase, uc::FlightPhase::HOLD);
  EXPECT_EQ(h.output.transition.reason, uc::TransitionReason::TRAJECTORY_COMPLETE);

  h.input.trajectory.finished = false;
  h.input.trajectory.stamp = h.now + 0.01;
  h.tick();
  ASSERT_EQ(h.output.phase, uc::FlightPhase::HOLD);

  ++h.input.trajectory.trajectory_id;
  h.input.trajectory.stamp = h.now + 0.01;
  h.tick();
  ASSERT_EQ(h.output.phase, uc::FlightPhase::TRAJECTORY);
  const auto accepted_reference = h.output.reference;
  h.input.trajectory.trajectory_id = 1;
  h.input.trajectory.stamp = h.now + 0.01;
  h.input.trajectory.reference.pos.x() += 5.0;
  h.tick();
  ASSERT_EQ(h.output.phase, uc::FlightPhase::TRAJECTORY);
  EXPECT_TRUE(h.output.reference.pos.isApprox(accepted_reference.pos));
  for (int i = 0; i < 60; ++i) h.tick();
  EXPECT_EQ(h.output.phase, uc::FlightPhase::HOLD);
  EXPECT_EQ(h.output.transition.reason, uc::TransitionReason::TRAJECTORY_TIMEOUT);
}

TEST(FlightController, FullSequenceEndsInPersistentEndpointHold) {
  FlightHarness h;
  h.engage();
  ASSERT_TRUE(h.until(uc::FlightPhase::TRAJECTORY));
  EXPECT_TRUE(h.output.trajectory_active);
  const auto mission_id = h.output.mission_id;
  // Actual pose/yaw differ from the commanded endpoint at completion.
  h.input.state.quat = Eigen::Quaterniond(Eigen::AngleAxisd(0.7, Eigen::Vector3d::UnitZ()));
  h.finishTrajectory();
  ASSERT_EQ(h.output.phase, uc::FlightPhase::HOLD);
  EXPECT_EQ(h.output.mode, uc::FlightMode::HOLD);
  EXPECT_EQ(h.output.mission_stage, uc::MissionStage::NONE);
  EXPECT_EQ(h.output.transition.from, uc::FlightPhase::TRAJECTORY);
  EXPECT_EQ(h.output.transition.reason, uc::TransitionReason::TRAJECTORY_COMPLETE);
  const auto endpoint = h.input.trajectory.reference;
  EXPECT_GT((endpoint.pos - h.input.state.pos).norm(), 0.5);
  EXPECT_TRUE(h.output.reference.pos.isApprox(endpoint.pos));
  EXPECT_DOUBLE_EQ(h.output.reference.yaw, endpoint.yaw);
  // Longer than every mission watchdog: completion must never begin descent,
  // restart the trajectory or stop the live attitude stream on its own.
  for (int i = 0; i < 15000; ++i) {
    h.tick(true);
    ASSERT_EQ(h.output.phase, uc::FlightPhase::HOLD);
    ASSERT_FALSE(h.output.trajectory_active);
    ASSERT_TRUE(h.output.publish);
    ASSERT_EQ(h.output.output_policy, uc::OutputPolicy::CLOSED_LOOP);
    ASSERT_TRUE(h.output.reference.pos.isApprox(endpoint.pos));
    ASSERT_DOUBLE_EQ(h.output.reference.yaw, endpoint.yaw);
    ASSERT_DOUBLE_EQ(h.output.reference.vel.norm(), 0.0);
    ASSERT_DOUBLE_EQ(h.output.reference.acc.norm(), 0.0);
    ASSERT_EQ(h.output.mission_id, mission_id);
  }
}

TEST(FlightController, EndpointHoldDoesNotFollowActualPositionDrift) {
  FlightHarness h;
  h.engage();
  ASSERT_TRUE(h.until(uc::FlightPhase::TRAJECTORY));
  h.finishTrajectory();
  const Eigen::Vector3d target = h.output.reference.pos;
  h.input.state.pos = target + Eigen::Vector3d(0.3, 0.0, 0.0);
  h.input.state.vel.setZero();
  for (int i = 0; i < 100; ++i) h.tick();
  EXPECT_EQ(h.output.phase, uc::FlightPhase::HOLD);
  EXPECT_TRUE(h.output.reference.pos.isApprox(target));
}

TEST(FlightController, AirborneOffboardReentryLocksHoldAndDoesNotRestartMission) {
  FlightHarness h;
  h.engage();
  ASSERT_TRUE(h.until(uc::FlightPhase::TRAJECTORY));
  h.input.offboard = false;
  h.tick();
  EXPECT_FALSE(h.output.trajectory_active);
  h.input.state.pos << 5.0, 6.0, 1.4;
  h.input.offboard = true;
  h.tick();
  ASSERT_EQ(h.output.phase, uc::FlightPhase::HOLD);
  const Eigen::Vector3d locked = h.output.reference.pos;
  h.input.state.pos.x() += 1.0;
  for (int i = 0; i < 100; ++i) h.tick();
  EXPECT_EQ(h.output.phase, uc::FlightPhase::HOLD);
  EXPECT_TRUE(h.output.reference.pos.isApprox(locked));
}

TEST(FlightController, FeedbackLossStopsOutputsAndRecoversToHold) {
  FlightHarness h;
  h.engage();
  ASSERT_TRUE(h.until(uc::FlightPhase::HOVER_BEFORE));
  h.input.feedback_valid = false;
  h.tick();
  EXPECT_EQ(h.output.phase, uc::FlightPhase::FEEDBACK_LOST);
  EXPECT_FALSE(h.output.publish);
  h.input.feedback_valid = true;
  h.tick();
  EXPECT_EQ(h.output.phase, uc::FlightPhase::FEEDBACK_LOST);
  EXPECT_FALSE(h.output.publish);
  for (int i = 0; i < 25; ++i) h.tick();
  EXPECT_EQ(h.output.phase, uc::FlightPhase::HOLD);
  EXPECT_TRUE(h.output.publish);
}

TEST(FlightController, InterruptedTakeoffCannotRestartOnDelayedGroundReport) {
  FlightHarness h;
  h.engage();
  ASSERT_EQ(h.output.phase, uc::FlightPhase::TAKEOFF);
  h.input.offboard = false;
  h.tick();
  ASSERT_EQ(h.output.phase, uc::FlightPhase::WAIT);
  h.input.offboard = true;
  h.tick();
  EXPECT_EQ(h.output.phase, uc::FlightPhase::HOLD);
  EXPECT_DOUBLE_EQ(h.output.command.thrust, 0.0);
  h.input.armed = false;
  h.tick();
  h.engage();
  EXPECT_EQ(h.output.phase, uc::FlightPhase::TAKEOFF);
}

TEST(FlightController, CompletedMissionRequiresManualDisarmBeforeAnotherTakeoff) {
  FlightHarness h;
  h.engage();
  std::uint64_t previous_mission = h.output.mission_id;
  for (int cycle = 0; cycle < 2; ++cycle) {
    ASSERT_TRUE(h.until(uc::FlightPhase::TRAJECTORY));
    EXPECT_GT(h.output.mission_id, previous_mission);
    previous_mission = h.output.mission_id;
    h.finishTrajectory();
    ASSERT_EQ(h.output.phase, uc::FlightPhase::HOLD);
    // Simulate manual landing. Ground feedback must not reset the mission latch.
    h.input.state.pos.z() = 0.4;
    h.input.state.vel.setZero();
    h.input.landing_state = uc::LandingState::ON_GROUND;
    for (int i = 0; i < 200; ++i) h.tick();
    EXPECT_EQ(h.output.phase, uc::FlightPhase::HOLD);
    EXPECT_EQ(h.output.output_policy, uc::OutputPolicy::ZERO_THRUST);
    h.input.offboard = false;
    h.tick();
    EXPECT_EQ(h.output.phase, uc::FlightPhase::WAIT);
    h.input.offboard = true;
    h.tick();
    EXPECT_EQ(h.output.phase, uc::FlightPhase::HOLD);
    EXPECT_EQ(h.output.mission_id, previous_mission);
    h.input.armed = false;
    h.tick();
    EXPECT_EQ(h.output.phase, uc::FlightPhase::WAIT);
    h.engage();
    EXPECT_EQ(h.output.phase, uc::FlightPhase::TAKEOFF);
  }
}

TEST(FlightController, StaleWrongMissionAndNonfiniteTrajectoriesAreIgnored) {
  for (int scenario = 0; scenario < 3; ++scenario) {
    FlightHarness h;
    h.engage();
    ASSERT_TRUE(h.until(uc::FlightPhase::TRAJECTORY));
    auto &point = h.input.trajectory;
    point.valid = true;
    point.finished = true;
    point.reference = h.output.reference;
    point.mission_id = h.output.mission_id;
    point.stamp = h.now;
    if (scenario == 0) --point.mission_id;
    if (scenario == 1) point.stamp -= 1.0;
    if (scenario == 2) point.reference.pos.x() = std::numeric_limits<double>::quiet_NaN();
    for (int i = 0; i < 70; ++i) h.tick();
    EXPECT_EQ(h.output.phase, uc::FlightPhase::TRAJECTORY);
    EXPECT_EQ(h.output.output_policy, uc::OutputPolicy::CLOSED_LOOP);
    EXPECT_TRUE(h.output.reference.pos.allFinite());
  }
}

TEST(FlightController, MovingCompletionMessageIsNotAcceptedAsHover) {
  FlightHarness h;
  h.engage();
  ASSERT_TRUE(h.until(uc::FlightPhase::TRAJECTORY));
  auto &point = h.input.trajectory;
  point.valid = point.finished = true;
  point.mission_id = h.output.mission_id;
  point.reference = h.output.reference;
  point.reference.vel.x() = 0.5;
  point.stamp = h.now + 0.01;
  h.tick();
  EXPECT_EQ(h.output.phase, uc::FlightPhase::TRAJECTORY);
}

TEST(FlightController, ClockResetCancelsAutomaticProgress) {
  FlightHarness h;
  h.engage();
  ASSERT_TRUE(h.until(uc::FlightPhase::HOVER_BEFORE));
  h.now -= 5.0;
  h.tick();
  EXPECT_EQ(h.output.phase, uc::FlightPhase::HOLD);
  EXPECT_EQ(h.output.transition.reason, uc::TransitionReason::CLOCK_DISCONTINUITY);
}

TEST(FlightController, LongControlGapDoesNotCancelAutomaticProgress) {
  FlightHarness h;
  h.engage();
  ASSERT_TRUE(h.until(uc::FlightPhase::HOVER_BEFORE));
  h.now += 1.0;
  h.tick();
  EXPECT_EQ(h.output.phase, uc::FlightPhase::HOVER_BEFORE);
}

TEST(FlightController, InvalidParametersAreRejected) {
  auto p = flightParams();
  p.takeoff_speed = 0.0;
  EXPECT_THROW(uc::FlightController(controlParams(), p), std::invalid_argument);
  p = flightParams();
  p.takeoff_height = std::numeric_limits<double>::quiet_NaN();
  EXPECT_THROW(uc::FlightController(controlParams(), p), std::invalid_argument);
}

TEST(FeedbackTime, SmallClockLeadIsToleratedWithoutAcceptingStaleOrFarFutureData) {
  EXPECT_TRUE(uc::freshTimestamp(10.005, 10.0, 0.5, 0.05));
  EXPECT_TRUE(uc::freshTimestamp(10.03, 10.0, 0.5, 0.05));
  EXPECT_FALSE(uc::freshTimestamp(10.1, 10.0, 0.5, 0.05));
  EXPECT_FALSE(uc::freshTimestamp(9.49, 10.0, 0.5, 0.05));
  EXPECT_FALSE(uc::freshTimestamp(0.0, 10.0, 0.5, 0.05));
  EXPECT_FALSE(uc::freshTimestamp(std::numeric_limits<double>::quiet_NaN(), 10.0, 0.5, 0.05));
}

TEST(FlightController, IntermittentFeedbackDoesNotChatterBetweenHoldAndLost) {
  FlightHarness h;
  h.engage();
  ASSERT_TRUE(h.until(uc::FlightPhase::HOVER_BEFORE));
  for (int i = 0; i < 100; ++i) {
    h.input.feedback_valid = i % 10 != 0;
    h.tick();
    EXPECT_EQ(h.output.phase, uc::FlightPhase::FEEDBACK_LOST);
    EXPECT_FALSE(h.output.publish);
  }
  h.input.feedback_valid = true;
  for (int i = 0; i < 25; ++i) h.tick();
  EXPECT_EQ(h.output.phase, uc::FlightPhase::HOLD);
}

TEST(FlightController, CompletedTrajectoryFeedbackLossRecoversToCurrentPositionHold) {
  FlightHarness h;
  h.engage();
  ASSERT_TRUE(h.until(uc::FlightPhase::TRAJECTORY));
  h.finishTrajectory();
  ASSERT_EQ(h.output.phase, uc::FlightPhase::HOLD);
  const auto mission_id = h.output.mission_id;
  h.input.feedback_valid = false;
  h.tick();
  ASSERT_EQ(h.output.phase, uc::FlightPhase::FEEDBACK_LOST);
  EXPECT_FALSE(h.output.publish);
  h.input.state.pos << 4.0, -1.0, 1.2;
  h.input.state.vel.setZero();
  h.input.feedback_valid = true;
  h.tick();
  EXPECT_FALSE(h.output.publish);
  for (int i = 0; i < 25; ++i) h.tick();
  EXPECT_EQ(h.output.phase, uc::FlightPhase::HOLD);
  EXPECT_EQ(h.output.transition.reason, uc::TransitionReason::FEEDBACK_RECOVERED);
  EXPECT_TRUE(h.output.reference.pos.isApprox(h.input.state.pos));
  EXPECT_EQ(h.output.output_policy, uc::OutputPolicy::CLOSED_LOOP);
  EXPECT_FALSE(h.output.trajectory_active);
  EXPECT_EQ(h.output.mission_id, mission_id);
}

TEST(FlightController, GroundedHoldBeforeAnyTakeoffKeepsZeroThrust) {
  FlightHarness h;
  h.engage();
  h.input.offboard = false;
  h.tick();
  h.input.offboard = true;
  for (int i = 0; i < 100; ++i) h.tick();
  EXPECT_EQ(h.output.phase, uc::FlightPhase::HOLD);
  EXPECT_DOUBLE_EQ(h.output.command.thrust, 0.0);
}

TEST(FlightController, SmallFutureTrajectoryStampIsAccepted) {
  FlightHarness h;
  h.engage();
  ASSERT_TRUE(h.until(uc::FlightPhase::TRAJECTORY));
  h.input.trajectory.valid = true;
  h.input.trajectory.mission_id = h.output.mission_id;
  h.input.trajectory.stamp = h.now + 0.03;
  h.input.trajectory.reference = h.output.reference;
  h.input.trajectory.reference.pos.x() += 0.1;
  h.tick();
  EXPECT_EQ(h.output.phase, uc::FlightPhase::TRAJECTORY);
  EXPECT_TRUE(h.output.reference.pos.isApprox(h.input.trajectory.reference.pos));
}

TEST(FlightController, ModeChangesCannotBypassFeedbackRecovery) {
  for (bool reenter_early : {false, true}) {
    FlightHarness h;
    h.engage();
    ASSERT_TRUE(h.until(uc::FlightPhase::HOVER_BEFORE));
    h.input.feedback_valid = false;
    h.tick();
    ASSERT_EQ(h.output.phase, uc::FlightPhase::FEEDBACK_LOST);
    h.input.feedback_valid = true;
    h.input.offboard = false;
    h.tick();
    const double recovery_start = h.now;
    h.input.offboard = reenter_early;
    while (h.now + 0.01 - recovery_start < h.params.feedback_recovery_time) {
      EXPECT_EQ(h.output.phase, uc::FlightPhase::FEEDBACK_LOST);
      EXPECT_FALSE(h.output.feedback_ready);
      EXPECT_FALSE(h.output.publish);
      EXPECT_FALSE(h.output.trajectory_active);
      h.tick();
    }
    for (int i = 0; i < 3; ++i) h.tick();
    EXPECT_TRUE(h.output.feedback_ready);
    EXPECT_TRUE(h.output.publish);
    EXPECT_EQ(h.output.phase, reenter_early ? uc::FlightPhase::HOLD : uc::FlightPhase::WAIT);
    h.input.offboard = true;
    h.tick();
    EXPECT_EQ(h.output.phase, uc::FlightPhase::HOLD);
    EXPECT_FALSE(h.output.trajectory_active);
  }
}

TEST(FlightController, StartupRecoveryGateAlsoAppliesWhileWaiting) {
  FlightHarness h;
  h.input.feedback_valid = false;
  h.tick();
  h.input.feedback_valid = true;
  h.engage();
  for (int i = 0; i < 15; ++i) {
    EXPECT_EQ(h.output.phase, uc::FlightPhase::WAIT);
    EXPECT_FALSE(h.output.publish);
    h.input.offboard = !h.input.offboard;
    h.tick();
  }
  h.input.offboard = true;
  for (int i = 0; i < 10; ++i) h.tick();
  EXPECT_EQ(h.output.phase, uc::FlightPhase::TAKEOFF);
}

TEST(FlightController, ClockResetRestartsTheRecoveryDwell) {
  FlightHarness h;
  h.engage();
  ASSERT_TRUE(h.until(uc::FlightPhase::HOVER_BEFORE));
  h.input.feedback_valid = false;
  h.tick();
  h.input.feedback_valid = true;
  for (int i = 0; i < 15; ++i) h.tick();
  h.now -= 1.0;
  h.tick();
  for (int i = 0; i < 15; ++i) {
    EXPECT_FALSE(h.output.publish);
    EXPECT_EQ(h.output.phase, uc::FlightPhase::FEEDBACK_LOST);
    h.tick();
  }
  for (int i = 0; i < 10; ++i) h.tick();
  EXPECT_EQ(h.output.phase, uc::FlightPhase::HOLD);
  EXPECT_TRUE(h.output.publish);
}

TEST(FlightController, LongControlGapDoesNotRestartTheRecoveryDwell) {
  FlightHarness h;
  h.engage();
  ASSERT_TRUE(h.until(uc::FlightPhase::HOVER_BEFORE));
  h.input.feedback_valid = false;
  h.tick();
  h.input.feedback_valid = true;
  for (int i = 0; i < 15; ++i) h.tick();
  h.now += 1.0;
  h.tick();
  EXPECT_EQ(h.output.phase, uc::FlightPhase::HOLD);
  EXPECT_TRUE(h.output.publish);
}

TEST(FlightController, AirborneStartupWithUnknownLandingStateDoesNotPublishZeroThrust) {
  for (bool offboard : {false, true}) {
    uc::FlightController controller(controlParams(), flightParams());
    uc::FlightInput input;
    input.feedback_valid = input.armed = true;
    input.offboard = offboard;
    input.state.pos.z() = 2.0;
    auto out = controller.update(input, 10.0);
    EXPECT_EQ(out.phase, uc::FlightPhase::WAIT);
    EXPECT_EQ(out.output_policy, uc::OutputPolicy::NO_OUTPUT);
    EXPECT_FALSE(out.publish);
    input.offboard = true;
    input.landing_state = uc::LandingState::IN_AIR;
    out = controller.update(input, 10.01);
    EXPECT_EQ(out.phase, uc::FlightPhase::HOLD);
    EXPECT_EQ(out.output_policy, uc::OutputPolicy::CLOSED_LOOP);
    EXPECT_GT(out.command.thrust, 0.0);
    EXPECT_TRUE(out.reference.pos.isApprox(input.state.pos));
  }
}

TEST(FlightController, TakeoffReportAloneDoesNotCountAsObservedAirborne) {
  FlightHarness h;
  h.engage();
  h.input.landing_state = uc::LandingState::TAKING_OFF;
  h.tick();
  h.input.offboard = false;
  h.tick();
  h.input.offboard = true;
  h.tick();
  ASSERT_EQ(h.output.phase, uc::FlightPhase::HOLD);
  h.input.landing_state = uc::LandingState::UNKNOWN;
  h.tick();
  EXPECT_EQ(h.output.output_policy, uc::OutputPolicy::NO_OUTPUT);
  h.input.landing_state = uc::LandingState::ON_GROUND;
  for (int i = 0; i < 100; ++i) h.tick();
  EXPECT_EQ(h.output.phase, uc::FlightPhase::HOLD);
  EXPECT_EQ(h.output.output_policy, uc::OutputPolicy::ZERO_THRUST);
}

TEST(FlightController, AirborneTakeoverCannotBecomeANewTakeoffWithoutDisarm) {
  FlightHarness h;
  h.input.landing_state = uc::LandingState::IN_AIR;
  h.engage();
  ASSERT_EQ(h.output.phase, uc::FlightPhase::HOLD);
  h.input.offboard = false;
  h.tick();
  h.input.landing_state = uc::LandingState::ON_GROUND;
  h.input.offboard = true;
  h.tick();
  EXPECT_EQ(h.output.phase, uc::FlightPhase::HOLD);
  EXPECT_EQ(h.output.transition.reason, uc::TransitionReason::MISSION_INTERRUPTED);
}

TEST(FlightController, SameStateDoesNotRepeatEntryOrResetElapsedTime) {
  FlightHarness h;
  const auto initial_transition = h.output.transition.sequence;
  for (int i = 0; i < 50; ++i) h.tick();
  EXPECT_EQ(h.output.transition.sequence, initial_transition);
  EXPECT_NEAR(h.output.phase_elapsed, 0.5, 1e-8);
  h.engage();
  EXPECT_EQ(h.output.mode, uc::FlightMode::ACTIVE);
  EXPECT_EQ(h.output.mission_stage, uc::MissionStage::TAKEOFF);
  const auto takeoff_transition = h.output.transition.sequence;
  for (int i = 0; i < 10; ++i) h.tick();
  EXPECT_EQ(h.output.transition.sequence, takeoff_transition);
  EXPECT_EQ(h.output.transition.from, uc::FlightPhase::WAIT);
  EXPECT_EQ(h.output.transition.to, uc::FlightPhase::TAKEOFF);
  EXPECT_EQ(h.output.transition.reason, uc::TransitionReason::TAKEOFF_STARTED);
  h.input.offboard = false;
  h.tick();
  EXPECT_EQ(h.output.mission_stage, uc::MissionStage::NONE);
  EXPECT_EQ(h.output.transition.reason, uc::TransitionReason::OFFBOARD_EXIT);
  const auto waiting_transition = h.output.transition.sequence;
  for (int i = 0; i < 10; ++i) h.tick();
  EXPECT_EQ(h.output.transition.sequence, waiting_transition);
  EXPECT_NEAR(h.output.phase_elapsed, 0.1, 1e-8);
}

TEST(FlightController, TakeoffTimeoutCancelsAutomaticProgress) {
  auto params = flightParams();
  params.takeoff_timeout = 0.3;
  FlightHarness h(params);
  h.engage();
  for (int i = 0; i < 35; ++i) h.tick();
  EXPECT_EQ(h.output.phase, uc::FlightPhase::HOLD);
  EXPECT_EQ(h.output.transition.reason, uc::TransitionReason::TAKEOFF_TIMEOUT);
  EXPECT_TRUE(h.output.reference.pos.isApprox(h.input.state.pos));
}

TEST(FlightController, HoverBeforeHasADeadlineIndependentOfStabilityTimer) {
  auto params = flightParams();
  params.hover_timeout = 0.4;
  FlightHarness h(params);
  h.engage();
  ASSERT_TRUE(h.until(uc::FlightPhase::HOVER_BEFORE));
  h.input.state.pos = h.output.reference.pos + Eigen::Vector3d(0.5, 0, 0);
  h.input.state.vel.setZero();
  for (int i = 0; i < 45; ++i) h.tick();
  EXPECT_EQ(h.output.phase, uc::FlightPhase::HOLD);
  EXPECT_EQ(h.output.transition.reason, uc::TransitionReason::HOVER_TIMEOUT);
}

TEST(FlightController, FirstTrajectoryPointCanArriveWithoutDeadline) {
  auto params = flightParams();
  params.trajectory_timeout = 0.1;
  FlightHarness h(params);
  h.engage();
  ASSERT_TRUE(h.until(uc::FlightPhase::TRAJECTORY));
  const auto locked = h.output.reference.pos;
  for (int i = 0; i < 250; ++i) h.tick();
  EXPECT_EQ(h.output.phase, uc::FlightPhase::TRAJECTORY);
  EXPECT_TRUE(h.output.reference.pos.isApprox(locked));
  auto &point = h.input.trajectory;
  point.valid = true;
  point.mission_id = h.output.mission_id;
  point.reference = h.output.reference;
  point.stamp = h.now + 0.01;
  h.tick();
  for (int i = 0; i < 15; ++i) h.tick();
  EXPECT_EQ(h.output.phase, uc::FlightPhase::HOLD);
  EXPECT_EQ(h.output.transition.reason, uc::TransitionReason::TRAJECTORY_TIMEOUT);
}

TEST(FlightController, MissingFirstPointKeepsWaiting) {
  FlightHarness h;
  h.engage();
  ASSERT_TRUE(h.until(uc::FlightPhase::TRAJECTORY));
  for (int i = 0; i < 500; ++i) h.tick();
  EXPECT_EQ(h.output.phase, uc::FlightPhase::TRAJECTORY);
  EXPECT_TRUE(h.output.trajectory_active);
}

TEST(FlightController, NonfiniteControllerCommandCancelsTrajectoryAndRequiresRecovery) {
  FlightHarness h;
  h.engage();
  ASSERT_TRUE(h.until(uc::FlightPhase::TRAJECTORY));
  auto &point = h.input.trajectory;
  point.valid = true;
  point.mission_id = h.output.mission_id;
  point.reference.pos = h.input.state.pos;
  point.reference.vel = h.input.state.vel;
  // Zero desired force is a singularity of the attitude construction.
  point.reference.acc.z() = -controlParams().gravity;
  point.stamp = h.now + 0.01;
  h.tick();
  EXPECT_EQ(h.output.phase, uc::FlightPhase::FEEDBACK_LOST);
  EXPECT_EQ(h.output.transition.reason, uc::TransitionReason::NONFINITE_COMMAND);
  EXPECT_FALSE(h.output.publish);
  EXPECT_FALSE(h.output.trajectory_active);
  h.tick();
  EXPECT_FALSE(h.output.publish);
  for (int i = 0; i < 25; ++i) h.tick();
  EXPECT_EQ(h.output.phase, uc::FlightPhase::HOLD);
  EXPECT_TRUE(h.output.publish);
}

TEST(FlightController, StageTimeoutParametersMustBeFiniteAndPositive) {
  for (auto member : {&uc::FlightParams::takeoff_timeout,
                      &uc::FlightParams::hover_timeout}) {
    for (double invalid : {0.0, -1.0, std::numeric_limits<double>::infinity(),
                           std::numeric_limits<double>::quiet_NaN()}) {
      auto params = flightParams();
      params.*member = invalid;
      EXPECT_THROW(uc::FlightController(controlParams(), params), std::invalid_argument);
    }
  }
}

}  // namespace

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
