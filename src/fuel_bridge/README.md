# FUEL 实机桥接

本模块使用已有 MID360／Faster-LIO 和 MAVROS，连接 FUEL 规划器、任务管理器及 `uav_controller`。环境、编译及启动步骤见 [根目录 README](../../README.md)。

`config/exploration.yaml` 配置 FUEL 建图、前沿筛选和雷达视场；雷达驱动及定位外参仍由 `realflight_modules` 中的配置维护。

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

`/fuel/start` 与 RViz 启动共用数据新鲜度、控制状态、起飞高度、悬停速度及规划边界检查。管理器分配任务批次，FUEL 把批次标识写入轨迹，桥接和控制器只放行当前批次的指令。FUEL 内部的避障与重规划保持原版行为：规划结果不做额外校验，执行中由原版 `checkTrajCollision` 触发重规划，`traj_server` 收到新 B 样条即切换，没有交接确认或主动制动。

`/fuel/stop` 取消任务并请求悬停。FUEL 第一次找不到可覆盖前沿即视为探索完成；探索完成、低电量及既有探索超时保护会进入返航，默认目标是本地 `[0,0,1]`。人工 `/fuel/return_home` 使用同一路径。返航不设置失败次数或总时间上限：规划失败或输出超时会进入 `RETURN_RECOVERY`，保留任务批次，FUEL 持续重试，输出超时后管理器请求重新规划。

桥接不会因探索边界拒绝返航任务。但起点在膨胀区、未知格或边界外时规划不出返航轨迹，FUEL 会一直重试，状态行显示 `FAILED` 和原因，这时需要操作员接管或执行 `/fuel/stop`。末段轨迹结束后超过 `return_home/arrival_timeout` 仍未到家，管理器会请求从当前位置重新规划。

桥接关闭或控制器悬停期间，FUEL 仍在沿自己的轨迹往前推进，它的参考点可能已在几米外。因此任务开始或桥接恢复后的第一条指令，以及控制器处于 `HOLD` 时的指令，离飞机超过 `reference_jump_limit` 都不转发：桥接上报 `OUTPUT_TIMEOUT`，返航时管理器请求 FUEL 从当前位置重新规划，探索时转为返航。控制器已在跟随轨迹时不做这项检查，跟踪误差不会打断飞行。

只有末段参考执行完成、实际位置和速度收敛、偏航归零且角速度稳定，才进入 `HOME_HOLD`。不会自动降落、解锁、上锁或切换 PX4 模式。参数见 `mission.yaml`，其中返航点、20% 电量阈值及超时需要对应实际使用场景。

末段轨迹结束后，FUEL 继续检查返航点，直到进入 `HOME_HOLD`。返航点不再是边界内已知、未膨胀的空闲点时，立即撤销到家确认，1.5 s 稳定计时从零开始；连续失效 0.5 s 后回到规划流程，对同一个返航点重试。这只保证不会在已发现返航点失效后确认到家，飞机不会因此让开。进入 `HOME_HOLD` 后不再检查。

返航点被占用、膨胀或未知时，FUEL 在附近寻找能看到阻塞体素的可达观察位，按雷达安装位姿、视场、射线距离和当前地图检查视线。观察轨迹结束后持续发布悬停参考和 `OBSERVING_HOME` 心跳。原点在途中或观察期间恢复可用时，继续返回原点。

如果阻塞体素仍有新的观测且保持占用，飞机在观察位等待。就位后连续约 3 s 没有相关观测，会重新选位；刚尝试的位置周围 1 m 在 30 s 内跳过。连续 3 个位置都没有观测后不再换位，原地悬停并报告 `FAILED`，原点恢复后继续返航；也可以执行 `/fuel/stop` 后重新 `/fuel/return_home` 再试一轮。没有可达观察位时报告规划失败并重试。观察点不会触发 `HOME_HOLD`。

任务管理器以执行器收到轨迹时发布的 `ACTIVE` 状态确认返航轨迹开始执行。`return_home/planning_timeout` 只约束首条返航轨迹到达执行器前的等待时间，之后不再用该计时触发恢复；规划器心跳、桥接输出超时及规划失败重试仍然有效。

## 相机视野裁剪

探索时配合云台录像，可以把 FUEL 的水平有效视野限制在相机视野内，参数在 `config/view_crop.yaml`（`view_config:=...` 可替换），bridge 和 FUEL 读取同一份：

- bridge 在**探索任务执行期间**只把机头两侧 `horizontal_fov_deg / 2` 以内的点写入地图，FUEL 只把这部分空间视为已探索；
- FUEL 评估候选视点时使用同样的水平范围，竖直方向不裁剪，仍按雷达自身视场（含斜装）判断；
- 任务开始前和返航时仍使用 360° 点云，保证起飞点周围和返航路径已知，否则返航可能因家点未知而无法规划。

方位角相对机头航向计算，假设云台跟随机头偏航。关闭裁剪设 `enabled: false`。

裁剪后侧方和后方的障碍物在探索期间不进入地图。A* 只经过已知空闲区域，飞行中机头朝前进方向，因此与原版 FUEL 使用前视相机时一致，但探索会比 360° 慢。任务开始前已用 360° 建好的区域（起飞点周围约 `max_ray_length` 范围）不会再安排相机去拍。

## 终端状态行

飞行中主要输出是 mission_manager 的一行状态，状态变化时立即打印，否则每 `status_log_period` 秒打印一次：

```
[FLIGHT_STATUS] mission=EXPLORING | fuel=EXECUTING | ctrl=TRAJECTORY | battery=76.0%
```

| 字段 | 含义 | 异常时 |
| --- | --- | --- |
| `mission` | 任务状态 | 附加 `reason=`（返航原因）、`recovery=... xN`（返航恢复原因与重规划请求次数） |
| `fuel` | FUEL 规划器状态（`/fuel/planner_status`） | `FAILED xN (原因)` 为连续规划失败及具体原因；`OUTPUT_TIMEOUT`、`TRAJECTORY_UNSAFE` 附带说明；`WAITING` 为任务刚开始尚未收到状态，超过 `planner_status_timeout` 变为 `NO_DATA`；`STALE` 为心跳超时 |
| `ctrl` | 控制器阶段 | 不在 `TRAJECTORY` 时附带切换原因，例如 `HOLD(TRAJECTORY_TIMEOUT)`；输出策略不是 `CLOSED_LOOP`、未离地、反馈无效时分别附加说明 |
| `battery` | 电量百分比 | 无有效数据时为 `NO_DATA` |

任一字段异常时整行以 WARN 级别输出。FUEL 正常重规划时在 `PLANNING`、`TRAJECTORY_READY`、`EXECUTING` 之间切换，不会因此逐次打印。FUEL 每次规划的耗时、搜索和优化细节为 `ROS_DEBUG` 级别，需要时可用 `rqt_logger_level` 把 `exploration_node` 调到 Debug 查看。

## 验证

```bash
bash scripts/test_fuel.sh
```

测试覆盖外参逆变换、IMU 点云至 MAVROS ENU 转换、真实雷达原点过滤、坐标系与数据时效、控制器状态机、地图边界、超时与返航恢复。合成数据用于验证软件接口，不代表实机飞行验收。

排查地图不更新时，依次检查点云类型与帧、MAVROS 里程计帧、时间戳和外参；前沿长期不消失时检查是否存在真实未观测区域。真实 MID360 没有测量的方向仍是未知空间，不能通过补造无回波射线来清除。

完整机载系统入口为工程根目录 `start_realflight.sh`，统一启动 MAVROS、MID360／Faster-LIO、EKF、控制器和 FUEL；详见 [根目录说明](../../README.md#启动)。

## RViz 无人机模型

FUEL 的 RViz 使用简易四旋翼模型显示无人机，橙色为机头、蓝色为机尾。模型由 bridge 根据现有里程计发布到 /fuel/uav_model，跟随位置和完整姿态；定位数据停止更新后，模型在 0.5 秒内消失。

外形尺寸在 src/fuel_bridge.cpp 的 publishModel() 中集中定义，目前旋翼外缘跨度约 0.52 米。原有启动方式不变。
