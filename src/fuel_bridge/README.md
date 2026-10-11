# FUEL 实机桥接

本模块使用已有 MID360／Faster-LIO 和 MAVROS，连接 FUEL 规划器、任务管理器及 `uav_controller`。环境、编译及启动步骤见 [根目录 README](../../README.md)。

## 数据路径

```text
MID360 → Faster-LIO → 去畸变 IMU 系点云 ─┐
Faster-LIO → 机载 EKF/位姿转发 → PX4 → MAVROS 定位 ─┤
                                       ↓
                    fuel_bridge → 世界系点云 + 雷达位姿 → FUEL
                                                           ↓
                      任务管理器 ↔ 轨迹执行器 → fuel_bridge
                                                           ↓
                      uav_controller → MAVROS 姿态/推力 → PX4
```

规划器和控制器均使用 MAVROS 局部 ENU 坐标。去畸变点云在 LIO IMU 系，因此不依赖 FAST-LIO 世界原点与 PX4 原点相同。桥接将点云和 MAVROS 里程计近似同步，时间差不超过 50 ms；数据过旧、坐标系错误、非有限点或无效姿态会被拒绝。

本机 Faster-LIO 的 `/laserMapping/cloud_registered_body` 实际由 `PointBodyLidarToIMU` 生成，帧默认 `imu`，不是飞机 FLU 机体系。不要将 `/laserMapping/cloud_registered` 的世界系点云送入同一入口。

## 外参

`prepare_realflight.py` 读取机载 LIO YAML，只写本工作空间的运行配置。采用与机载源码一致的定义：

```text
p_I = R_IB p_B + t_IB       # mapping/imu_body_R, imu_body_T
p_I = R_IL p_L + t_IL       # mapping/extrinsic_R, extrinsic_T
R_BI = R_IBᵀ
 t_BI = -R_IBᵀ t_IB
R_BL = R_IBᵀ R_IL
 t_BL = R_IBᵀ(t_IL - t_IB)
p_W = p_WB + R_WB(t_BI + R_BI p_I)
雷达射线原点 = p_WB + R_WB t_BL
```

I 为 LIO 使用的 IMU 系，B 为飞机 FLU，L 为 LiDAR，W 为 MAVROS 局部 ENU。点云坐标原点和雷达射线原点分别处理；近距离过滤也相对真实雷达原点计算。桥接与候选观察点的视场使用相同的雷达安装位置和完整 roll/pitch/yaw。

只处理真实有效回波；无回波方向不会被伪造为自由空间。超过地图建图量程的真实回波仍可按 FUEL 原有射线截断逻辑清空沿途空间，截断位置不记为障碍。RViz 显示所有有效回波。

修改 LIO 外参后重新启动本工程；动态外参估计不在此固定外参接口支持范围内。启动脚本会校验旋转矩阵、有限值、发布开关及 `extrinsic_est_en`。

## 接口

| 接口 | 默认话题 / 类型 |
| --- | --- |
| 定位输入 | `/mavros/local_position/odom`，`nav_msgs/Odometry` |
| 点云输入 | `/laserMapping/cloud_registered_body`，`sensor_msgs/PointCloud2` |
| 世界系点云 | `/fuel/cloud_world`、`/fuel/cloud_hits_world` |
| 雷达位姿 | `/fuel/sensor_pose`，与点云时间戳、坐标系相同 |
| 规划轨迹指令 | `/fuel/position_cmd`，`quadrotor_msgs/PositionCommand` |
| 控制器输入 | `/uav_controller/trajectory_setpoint`，`uav_controller/TrajectorySetpoint` |
| 控制反馈 | `/uav_controller/fsm_status`，`uav_controller/FlightStatus` |
| 任务状态 | `/fuel/mission_status`，`uav_mission_msgs/MissionStatus` |
| 控制器输出 | `/mavros/setpoint_raw/attitude`，`mavros_msgs/AttitudeTarget` |
| 探索边界 | `/fuel/exploration_bounds` |

`world_frame` 必须与 MAVROS 里程计的 `header.frame_id` 一致，实机入口默认 `map`；所有建图、轨迹、任务状态和可视化使用同一坐标系。`odom_twist_in_body` 默认 `true`，将 MAVROS 的机体系里程计线速度转成规划所需世界系速度。控制器继续使用 `/mavros/local_position/velocity_local`。

`source.sh` 保留已有工作空间叠加环境。默认定位配置路径为 `src/realflight_modules/faster-lio/faster-lio/config/mid360.yaml`；可通过 `lio_config:=...` 或 `FAST_LIO_CONFIG` 指定，`FAST_LIO_ROOT` 可改变默认根目录。生成外参及 TSP 临时文件默认放在 `build/fuel_runtime`，可用 `FUEL_TSP_DIR` 覆盖。

仅启动规划器的 `fuel.launch` 不启动控制器，直接使用时必须传入正确的 `sensor_config`、`world_frame`、点云和定位话题；仅 FUEL 入口为 `fuel_realflight.sh`，完整机载入口为 `start_realflight.sh`。

## 任务行为

`/fuel/start` 与 RViz 启动共用数据新鲜度、控制状态、起飞高度、悬停速度及规划边界检查。管理器拥有任务批次授权，轨迹在接收和激活时检查连续性；过期、跨任务或未授权指令不能进入控制器。

`/fuel/stop` 取消任务并请求悬停。探索完成、低电量及既有探索超时保护会进入返航，默认目标是本地 `[0,0,1]`。人工 `/fuel/return_home` 使用同一路径。返航不设置失败次数或总时间上限：规划失败、交接拒绝或输出超时会进入 `RETURN_RECOVERY`，保留任务批次继续重试；必要时制动，恢复可用轨迹后继续。

只有末段参考执行完成、实际位置和速度收敛、偏航归零且角速度稳定，才进入 `HOME_HOLD`。不会自动降落、解锁、上锁或切换 PX4 模式。参数见 `mission.yaml`，其中返航点、30% 电量阈值及超时需要对应实际使用场景。

## 验证

```bash
bash scripts/test_fuel.sh
```

测试覆盖外参逆变换、IMU 点云至 MAVROS ENU 转换、真实雷达原点过滤、坐标系与数据时效、控制器状态机、地图边界、探索交接、超时与返航恢复。合成数据用于验证软件接口，不代表实机飞行验收。

排查地图不更新时，依次检查点云类型与帧、MAVROS 里程计帧、时间戳和外参；前沿长期不消失时检查是否存在真实未观测区域。真实 MID360 没有测量的方向仍是未知空间，不能通过补造无回波射线来清除。

完整机载系统入口为工程根目录 `start_realflight.sh`，统一启动 MAVROS、MID360／Faster-LIO、EKF、控制器和 FUEL；详见 [根目录说明](../../README.md#启动)。

## RViz 无人机模型

FUEL 的 RViz 使用简易四旋翼模型显示无人机，橙色为机头、蓝色为机尾。模型由 bridge 根据现有里程计发布到 /fuel/uav_model，跟随位置和完整姿态；定位数据停止更新后，模型在 0.5 秒内消失。

外形尺寸在 src/fuel_bridge.cpp 的 publishModel() 中集中定义，目前旋翼外缘跨度约 0.52 米。原有启动方式不变。
