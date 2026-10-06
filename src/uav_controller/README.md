# uav_controller: modular ROS 1 flight control

ROS 1/MAVROS position controller with a ROS-free flight state machine and geometric attitude/thrust control core. It never arms, disarms, or switches PX4 modes automatically.

## Components

- `uav_controller_core`: feedback validation, flight sequencing, reference selection, position/velocity feedback, acceleration feed-forward, attitude generation, and thrust estimation.
- `geometric_controller_node`: MAVROS input/output adapter and status publisher.
- `fixed_point_pub`: optional smooth fixed-point reference publisher.
- `trajectory_rviz_node`: generic desired/actual path visualization.

## Control modes

The state machine exposes `WAIT`, `TAKEOFF`, `HOVER_BEFORE`, `TRAJECTORY`, `HOLD`, and `FEEDBACK_LOST`.

- Normal mission mode can generate a smooth vertical takeoff, wait at the takeoff target, and then accept an external trajectory.
- `fsm/direct_trajectory=true` is intended for planners such as FUEL. An airborne OFFBOARD entry holds the measured pose; the first fresh external setpoint starts trajectory tracking. Completion or timeout returns to HOLD.
- `fsm/position_step=true` is a manual test mode. After manual takeoff, entering OFFBOARD immediately applies the configured absolute ENU target.

References contain position, velocity, acceleration, and yaw. The controller publishes a desired attitude quaternion and normalized collective thrust to `/mavros/setpoint_raw/attitude`; PX4 retains the attitude-rate and actuator inner loops.

## Run

Start the reusable controller:

```bash
roslaunch uav_controller geometric_controller.launch
```

Run the position-step test:

```bash
roslaunch uav_controller fixed_point_test.launch
```

The FUEL integration launches `geometric_controller.launch` with its own P600 parameter file and supplies `TrajectorySetpoint` messages directly.

## Main interfaces

| Topic | Type | Purpose |
|---|---|---|
| `/mavros/local_position/pose` | `geometry_msgs/PoseStamped` | Position and attitude feedback |
| `/mavros/local_position/velocity_local` | `geometry_msgs/TwistStamped` | Velocity feedback |
| `/mavros/state` | `mavros_msgs/State` | FCU connection, arming, and mode |
| `/mavros/extended_state` | `mavros_msgs/ExtendedState` | Ground/air state |
| `/uav_controller/trajectory_setpoint` | `uav_controller/TrajectorySetpoint` | External position, velocity, acceleration, yaw, and completion |
| `/uav_controller/active_setpoint` | `mavros_msgs/PositionTarget` | Active reference for diagnostics and visualization |
| `/uav_controller/fsm_status` | `uav_controller/FlightStatus` | State, transition reason, output policy, and feedback health |
| `/mavros/setpoint_raw/attitude` | `mavros_msgs/AttitudeTarget` | Desired attitude and thrust |

All controller math uses local ENU position and FLU attitude conventions. Mission IDs, finite-value checks, feedback freshness, trajectory timeouts, clock-discontinuity handling, and recovery dwell time prevent stale or invalid data from continuing a mission.

## Tests

```bash
catkin_make run_tests_uav_controller
catkin_test_results build/test_results
```

The remaining tests cover the controller core and the ROS position-step path.
