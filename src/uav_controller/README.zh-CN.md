# uav_controller：模块化 ROS 1 飞行控制器

这是一个基于 ROS 1/MAVROS 的位置控制包，包含无 ROS 依赖的飞行状态机和几何姿态/推力控制核心。程序不会主动解锁、上锁或切换 PX4 飞行模式。

## 组成

- `uav_controller_core`：反馈检查、飞行流程、参考目标选择、位置/速度反馈、加速度前馈、姿态计算和推力估计。
- `geometric_controller_node`：连接 MAVROS 输入输出并发布控制器状态。
- `fixed_point_pub`：可选的平滑定点参考生成器。
- `trajectory_rviz_node`：通用期望轨迹与实际轨迹可视化。

## 控制模式

状态机包含 `WAIT`、`TAKEOFF`、`HOVER_BEFORE`、`TRAJECTORY`、`HOLD` 和 `FEEDBACK_LOST`。

- 普通任务模式可以先让电机缓慢起转（`fsm/spool_time`），再生成平滑垂直起飞参考，到达目标后稳定悬停，再接收外部轨迹。
- `fsm/direct_trajectory=true` 用于 FUEL 等规划器。无人机在空中切入 OFFBOARD 后先保持实测位置，在地面切入则先执行上述起飞；收到第一个新鲜外部目标后开始轨迹跟踪；轨迹结束或超时后回到 `HOLD`。
- `fsm/position_step=true` 用于手动测试。手动起飞后切入 OFFBOARD，立即采用配置的本地 ENU 绝对目标。

轨迹参考包含位置、速度、加速度和 yaw。控制器向 `/mavros/setpoint_raw/attitude` 发布目标姿态四元数和归一化总推力；姿态角速度与执行器内环仍由 PX4 完成。

## 运行

启动通用控制器：

```bash
roslaunch uav_controller geometric_controller.launch
```

运行位置阶跃测试：

```bash
roslaunch uav_controller fixed_point_test.launch
```

FUEL 集成会使用 P600 专用参数启动 `geometric_controller.launch`，并直接发送 `TrajectorySetpoint`。

## 主要接口

| 话题 | 类型 | 用途 |
|---|---|---|
| `/mavros/local_position/pose` | `geometry_msgs/PoseStamped` | 位置与姿态反馈 |
| `/mavros/local_position/velocity_local` | `geometry_msgs/TwistStamped` | 速度反馈 |
| `/mavros/state` | `mavros_msgs/State` | FCU 连接、解锁和模式 |
| `/mavros/extended_state` | `mavros_msgs/ExtendedState` | 地面/空中状态 |
| `/uav_controller/trajectory_setpoint` | `uav_controller/TrajectorySetpoint` | 外部位置、速度、加速度、yaw 和完成标记 |
| `/uav_controller/active_setpoint` | `mavros_msgs/PositionTarget` | 当前参考目标，用于诊断和可视化 |
| `/uav_controller/fsm_status` | `uav_controller/FlightStatus` | 状态、切换原因、输出策略和反馈健康状态 |
| `/mavros/setpoint_raw/attitude` | `mavros_msgs/AttitudeTarget` | 目标姿态与推力 |

控制器内部采用本地 ENU 位置和 FLU 姿态约定。任务编号、有限值检查、轨迹超时、时钟中断处理和恢复等待用于阻止无效数据继续控制。MAVROS 已连接且收到过位姿和速度即视为反馈有效，不检查反馈时间戳的时效。

## 测试

```bash
catkin_make run_tests_uav_controller
catkin_test_results build/test_results
```

保留的测试覆盖控制器核心逻辑和 ROS 位置阶跃链路。
