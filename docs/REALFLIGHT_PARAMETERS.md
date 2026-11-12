# 实飞操作与参数说明

本文汇总实机探索时要用到的命令、需要检查或调整的参数，以及每个参数所在的文件。表中“当前值”取自仓库中的配置文件；标 ★ 的参数在换机体、换场地或换安装方式时必须重新确认。

## 1. 启动链与配置文件

```
start_realflight.sh
├─ MAVROS            /opt/ros/noetic/share/mavros/launch/px4.launch（外部文件）
├─ MID360 + Faster-LIO  faster_lio/launch/mapping_mid360.launch
│    ├─ livox_ros_driver2/config/MID360_config.json   雷达网络
│    └─ faster-lio/config/mid360.yaml                  LIO 参数、雷达外参
├─ EKF → /mavros/vision_pose/pose   ekf_pose/launch/ekf_lidar.launch + PX4_vio_drone.yaml
└─ fuel_realflight.sh
     ├─ prepare_realflight.py  从 mid360.yaml 生成雷达外参（sensor_config）
     └─ fuel_bridge/launch/realflight.launch
          ├─ 控制器  uav_controller_realflight.yaml + uav_controller/config/fsm.yaml
          └─ fuel.launch
               ├─ FUEL 规划器  algorithm.xml（原版默认值）→ exploration.yaml 覆盖
               ├─ 任务管理器    mission.yaml
               ├─ 桥接          bridge.yaml
               └─ 相机视野裁剪  view_crop.yaml（规划器与桥接共用）
```

除 MAVROS 外，以上路径都相对于 `src/realflight_modules/faster-lio/` 或 `src/fuel_bridge/`。

`realflight.launch` 的参数可以直接加在启动命令后面临时覆盖，不用改文件：

```bash
bash start_realflight.sh rviz:=false max_vel:=0.8 box_max_x:=25
```

`lio_config:=/路径/mid360.yaml` 用于指定其他 LIO 配置。环境变量 `FAST_LIO_CONFIG`、`FAST_LIO_ROOT` 有同样作用；`FUEL_TSP_DIR` 指定外参和 TSP 临时文件目录，默认 `build/fuel_runtime`。

## 2. 飞行操作

### 2.1 起飞前

1. 接通雷达网线。机载电脑网口配置为 `192.168.1.50/24`，雷达地址为 `192.168.1.199`。
2. 在工程根目录启动全部模块：
   ```bash
   bash start_realflight.sh
   ```
3. 另开终端检查：
   ```bash
   source source.sh
   rostopic echo -n 1 /mavros/state                                   # connected: True
   rostopic hz /laserMapping/cloud_registered_body                     # 约 10 Hz
   rostopic echo -n 1 /mavros/local_position/pose                      # 位置随定位更新
   rosparam get /exploration_node/perception_utils/lidar_pitch         # 约 0.2618（15° 下倾）
   ```
4. 在 RViz 中确认点云、地图和探索边界框位置正确，飞机周围没有异常障碍。

### 2.2 起飞与开始探索

1. 选择一种起飞方式：
   - **手动起飞**：解锁，手动起飞并稳定悬停，再切换 OFFBOARD。控制器锁定切换时的位置进入 `HOLD`。
   - **地面自动起飞**：先解锁，再尽快切 OFFBOARD（PX4 默认约 10 s 未起飞会自动上锁）。切换后依次是：起转（`spool_time`，油门线性升到 0.8 倍悬停油门）→ 沿平滑轨迹升到绝对高度 `takeoff_height` → 悬停 `hover_before` 秒后等待任务，默认全程约 12 s。切完不要靠近，电机可能约 1 s 后才开始转；先切 OFFBOARD 再解锁则解锁后立刻起转。
2. 确认 `/mavros/state` 为 `OFFBOARD`、`armed: True`，状态行中 `ctrl=HOLD(...)`（手动起飞）或 `ctrl=TRAJECTORY`（自动起飞）。在此之前开始探索会被拒绝。
3. 开始探索：点击 RViz 的 **2D Nav Goal**，或执行
   ```bash
   rosservice call /fuel/start "{}"
   ```

### 2.3 飞行中

终端主要输出是任务管理器的状态行，变化时立即打印，否则每 2 s 一行：

```
[FLIGHT_STATUS] mission=EXPLORING | fuel=EXECUTING | ctrl=TRAJECTORY | battery=76.0%
```

任一字段异常时整行变为黄色并附带原因，例如 `fuel=FAILED x12 (no A* path to next viewpoint ...)`、`ctrl=HOLD(TRAJECTORY_TIMEOUT)`。各字段含义见 [FUEL 桥接说明](../src/fuel_bridge/README.md#终端状态行)。

```bash
rosservice call /fuel/stop "{}"          # 取消任务，原地悬停
rosservice call /fuel/return_home "{}"   # 人工返航
```

任务管理器在探索完成、电量低于阈值或探索超时保护触发时自动返航，到达 `home_position` 后悬停（`mission=HOME_HOLD`）。**不会自动降落或上锁**，降落由操作者接管。返航点不可用时，飞机尝试从附近有视线的位置观测阻塞区域，悬停观测时状态行显示 `OBSERVING_HOME`。连续约 3 s 未取得相关观测会重新选位，连续 3 个位置都没有观测就原地悬停并显示 `FAILED`，需要操作者接管；持续确认占用则等待，原点恢复可用后继续返航。

### 2.4 紧急接管

遥控器切出 OFFBOARD 即由 PX4 接管，控制器回到 `WAIT` 并停止使用估计的悬停油门。OFFBOARD 信号丢失后的动作由 PX4 参数决定（见第 10 节）。

切回 OFFBOARD 后飞机先原地悬停，FUEL 从当前位置重新规划后继续返航。探索途中接管时任务已转为返航，切回后也是返航，不会继续探索。不想让飞机继续飞时，先执行 `/fuel/stop` 再切回。

### 2.5 记录飞行数据

起飞前在机载电脑上另开终端启动录制，降落后按 Ctrl+C 停止：

```bash
bash scripts/record_flight.sh
```

默认录制 FUEL 状态机（`/fuel/planner_status`、`/fuel/mission_status`）、控制阶段（`/uav_controller/fsm_status`），以及控制跟踪所需的参考值与实际位姿、速度（`/uav_controller/active_setpoint`、`/mavros/local_position/pose`、`/mavros/local_position/velocity_local`），每小时只有几十 MB。bag 保存在 `~/flight_bags`，可用环境变量 `FLIGHT_BAG_DIR` 修改；需要额外话题时直接加在命令后面，例如 `bash scripts/record_flight.sh /mavros/setpoint_raw/attitude`。

分析时注意：`active_setpoint` 的坐标系标志写的是 `FRAME_LOCAL_NED`，但数值实际是 ENU，与 `local_position` 可以直接相减。`planner_status/state` 的编号依次为 IDLE、PLANNING、TRAJECTORY_READY、EXECUTING、EXPLORATION_DONE、GOAL_REACHED、FAILED、OUTPUT_TIMEOUT、TRAJECTORY_UNSAFE、SETTLING（0～9）。

## 3. 起飞前必查清单

| 项目 | 文件 / 参数 | 说明 |
| --- | --- | --- |
| ★ 雷达网络 | `MID360_config.json` | 主机 `192.168.1.50`，雷达 `192.168.1.199` |
| ★ 雷达安装角度 | `mid360.yaml: imu_body_R`、`PX4_vio_drone.yaml: rotation_imu` | 当前为 15° 下倾，两处必须一致 |
| ★ 雷达安装位置 | `mid360.yaml: imu_body_T`、`PX4_vio_drone.yaml: tr_i`、`ekf_lidar.launch: imu_trans_*` | **目前均为未标定的 0** |
| ★ 坐标系 | `realflight.launch: world_frame` | 必须与 MAVROS 里程计的 `frame_id` 一致，默认 `map` |
| ★ 探索边界与地图尺寸 | `realflight.launch: box_*`、`map_size_*` | 见第 4 节的注意事项 |
| ★ 返航点 | `mission.yaml: home_position` | 本地 ENU，通常为起飞点上方 |
| ★ 悬停油门 | `uav_controller_realflight.yaml: hover_percentage` | 换电池、换负载后重新确认 |
| ★ 速度与加速度 | `realflight.launch: max_vel`、`max_acc` | 首飞建议降低 |
| ★ 低电量返航阈值 | `mission.yaml: battery/return_threshold` | 需留足返航所需电量 |
| 相机视野裁剪 | `view_crop.yaml` | 是否只按相机前方视野探索 |

## 4. 探索范围与速度：`src/fuel_bridge/launch/realflight.launch`

| 参数 | 当前值 | 作用 | 调整建议 |
| --- | --- | --- | --- |
| ★ `box_min_x/y/z` | -1.0 / -15.0 / 0.1 | 探索边界下限（本地 ENU，m） | 按场地设置；`box_min_z` 也是允许的最低飞行高度，斜装雷达建议提到 0.5 左右 |
| ★ `box_max_x/y/z` | 40.0 / 4.0 / 2.5 | 探索边界上限 | 按场地设置，必须落在地图范围内 |
| ★ `map_size_x/y/z` | 60.0 / 50.0 / 6.0 | FUEL 地图尺寸（m） | 地图以原点为中心：x、y 范围为 ±size/2，z 从 `ground_height` 起 |
| ★ `max_vel` | 1.0 | 规划最大速度（m/s） | 首飞建议 0.5 |
| ★ `max_acc` | 0.4 | 规划最大加速度（m/s²） | 首飞建议 0.2～0.3 |
| `world_frame` | map | 建图、规划和指令使用的坐标系 | 与 MAVROS 一致 |
| `odom_topic` | /mavros/local_position/odom | 规划使用的里程计 | 一般不改 |
| `odom_twist_in_body` | true | MAVROS 里程计速度在机体系，需转到世界系 | 一般不改 |
| `cloud_topic` | /laserMapping/cloud_registered_body | 建图点云（Faster-LIO 的 IMU 系） | 一般不改 |
| `battery_topic` | /mavros/battery | 电量 | 一般不改 |
| `controller_params` | config/uav_controller_realflight.yaml | 控制器参数文件 | 可换成其他增益文件 |
| `mission_config` | config/mission.yaml | 任务与返航参数文件 | |
| `view_config` | config/view_crop.yaml | 相机视野裁剪参数文件 | |
| `rviz` | true | 是否启动 FUEL 的 RViz | 无图形环境时设 false |

> **注意：当前 `box_max_x = 40` 超出了地图范围。** `map_size_x = 60` 时地图 x 只覆盖 [-30, 30]，x > 30 的区域既不会建图也不会被探索。如果场地确实延伸到 40 m，需要把 `map_size_x` 调到至少 80（2 × 40）；否则把 `box_max_x` 降到 29 以内。y、z 方向目前在范围内。

## 5. 控制器：`src/fuel_bridge/config/uav_controller_realflight.yaml`

| 参数 | 当前值 | 作用 | 调整建议 |
| --- | --- | --- | --- |
| ★ `hover_percentage` | 0.40 | 悬停油门初值，推力估计从这里开始 | 手动悬停时读遥控器油门；在线估计会修正，但初值偏差大时切 OFFBOARD 瞬间会掉高或上冲 |
| `estimate_thrust` | true | 在线估计悬停油门 | 确认离地后才估计（PX4 报告在空中；自动起飞时还要求高度上升 0.2 m），结果限制在 [0.1, 0.8] |
| `k_pos` | x 4.0 / y 4.0 / z 6.0 | 位置误差增益 | 跟踪偏慢时增大，振荡时减小 |
| `k_vel` | x 3.0 / y 3.0 / z 4.0 | 速度误差增益 | 抖动或超调时调整 |
| `max_a_fb` | 8.0 | 反馈加速度上限（m/s²） | 限制大误差时的最大倾角 |
| `gravity` | 9.81 | 重力加速度 | 不改 |
| `control_rate_hz` | 100.0 | 控制频率（Hz） | 不建议低于 100，推力估计依赖 35～45 ms 前的指令 |
| `timestamp_future_tolerance` | 0.1 | 允许轨迹点时间戳超前本机的时间（s） | 机间时钟不同步时适当增大 |
| `setpoint_topic` | /uav_controller/trajectory_setpoint | 轨迹输入话题 | 不改 |
| `print_tracking` | false | 是否每隔一段时间打印跟踪误差 | 调参时可设 true |
| `print_period_ms` | 500 | 跟踪误差打印周期（ms） | |

### 5.1 控制器状态机：`src/uav_controller/config/fsm.yaml`

`realflight.launch` 固定 `direct_trajectory: true`、`position_step: false`。以下参数只在**地面直接切 OFFBOARD 自动起飞**时用到，以及判断轨迹超时。

| 参数 | 当前值 | 作用 |
| --- | --- | --- |
| `takeoff_height` | 1.0 | 自动起飞的绝对高度（本地 ENU z，m），不是相对起飞点的高度 |
| `takeoff_speed` | 0.3 | 自动起飞的参考最大上升速度（m/s）；悬停油门初值偏低时，离地较晚，实际上升会更快 |
| `spool_time` | 3.0 | 起飞前开环起转时长（s）：油门线性升到 0.8 倍悬停油门，高度上升超过 0.1 m 时提前转闭环；0 表示不起转 |
| `hover_before` | 2.0 | 起飞后稳定悬停多久才接受任务（s） |
| `position_tolerance` / `speed_tolerance` | 0.15 / 0.15 | 判断到位的位置（m）和速度（m/s）误差 |
| `stable_time` | 1.0 | 到位后需持续稳定的时间（s） |
| `trajectory_timeout` | 0.5 | 轨迹点超过此时间未更新则转为 HOLD（s） |
| `takeoff_timeout` / `hover_timeout` | 30.0 / 30.0 | 起飞（含起转）、悬停阶段超时后转为 HOLD（s） |
| `feedback_recovery_time` | 0.2 | 反馈恢复后需连续有效多久才继续控制（s） |

## 6. 任务与返航：`src/fuel_bridge/config/mission.yaml`

| 参数 | 当前值 | 作用 | 调整建议 |
| --- | --- | --- | --- |
| ★ `home_position` | [0.0, 0.0, 1.0] | 返航点（本地 ENU，m），到达后悬停，航向归零 | 必须在探索边界内且是已知空闲区域 |
| ★ `battery/return_threshold` | 0.20 | 电量低于此比例时自动返航 | 按场地大小和返航距离留余量 |
| `battery/enabled` | true | 是否启用低电量返航 | |
| `battery/data_timeout` | 3.0 | 电量数据超过此时间未更新视为无效（s） | |
| `status_log_period` | 2.0 | 状态行打印周期（s） | |
| `exploration/initial_plan_timeout` | 10.0 | 开始探索后多久没有第一条轨迹就返航（s） | 地图很大时规划变慢，可适当增大 |
| `exploration/planner_status_timeout` | 4.0 | 规划器心跳超时（s），探索中超时则返航 | |
| `return_home/replan_period` | 1.0 | 返航中途分段重规划周期（s） | |
| `return_home/retry_interval` | 1.0 | 返航超时后重新请求规划的间隔（s） | |
| `return_home/planning_timeout` | 10.0 | 第一条返航轨迹的最长等待时间（s） | |
| `return_home/position_tolerance` | 0.20 | 判断到家的位置误差（m） | |
| `return_home/speed_tolerance` | 0.15 | 判断到家的速度误差（m/s） | |
| `return_home/stable_time` | 1.5 | 到家后需稳定的时间（s），之后进入 HOME_HOLD | |
| `return_home/arrival_timeout` | 5.0 | 末段轨迹结束后多久还没到家就重新规划（s） | |
| `data_timeout` | 0.5 | 里程计、控制器状态的新鲜度要求（s） | 不建议改 |
| `source_id` | fuel_planner | 任务授权标识 | **不要改**，规划器、桥接和控制器靠它匹配 |

## 7. FUEL 建图与探索：`src/fuel_bridge/config/exploration.yaml`

此文件在原版 `algorithm.xml` 之后加载，同名参数以此文件为准。

| 参数 | 当前值 | 作用 | 调整建议 |
| --- | --- | --- | --- |
| `sdf_map/resolution` | 0.2 | 地图体素大小（m） | 减小会显著增加计算量 |
| ★ `sdf_map/obstacles_inflation` | 0.6 | 障碍物膨胀半径（m），规划与障碍保持的最小距离 | 不小于机体半径（桨尖约 0.26 m）加余量；过大会堵住窄通道 |
| `sdf_map/max_ray_length` | 10.0 | 雷达光线写入地图的最远距离（m），更远处只标记沿途空闲 | |
| `sdf_map/ground_height` | -1.0 | 地图 z 起点（m） | |
| `sdf_map/virtual_ceil_height` | 3.5 | 虚拟天花板高度（m），以上视为障碍 | 应高于 `box_max_z` |
| `map_ros/visualization_truncate_height` / `_low` | 3.5 / 0.0 | RViz 显示地图的高度范围（m） | 只影响显示 |
| `map_ros/show_all_map` | true | RViz 显示整张地图 | 只影响显示 |
| `frontier/cluster_min` | 30 | 前沿团的最少体素数，更小的团忽略 | 增大可忽略零碎前沿 |
| `frontier/min_candidate_clearance` | 0.5 | 候选视点与障碍的最小距离（m） | |
| `frontier/min_visib_num` | 10 | 候选视点至少能看到的前沿体素数 | |
| `perception_utils/lidar_360` | true | 使用 360° 雷达视场模型 | 不改 |
| `perception_utils/lidar_min_elevation` | -0.122173 | 雷达竖直视场下限（rad，-7°，雷达自身坐标系） | 可留 2° 余量改为 -0.087 |
| `perception_utils/lidar_max_elevation` | 0.907571 | 雷达竖直视场上限（rad，52°） | 可留 2° 余量改为 0.873 |
| `perception_utils/max_dist` | 10.0 | 评估候选视点时的最远可见距离（m） | 可设为 5～6 |
| `perception_utils/vis_dist` | 2.0 | RViz 中视场框的显示长度（m） | 只影响显示 |

雷达安装位置和角度（`perception_utils/lidar_x/y/z/roll/pitch/yaw`）由 `prepare_realflight.py` 从 `mid360.yaml` 自动生成，不在此文件中填写。

### 7.1 原版 FUEL 中可能需要调整的参数

下列参数在 `src/fuel/exploration_manager/launch/algorithm.xml` 中。需要调整时，在 `exploration.yaml` 中添加同名参数覆盖，不要直接改原版文件。

| 参数 | 默认值 | 作用 |
| --- | --- | --- |
| `exploration/yd` | 60°/s（1.047 rad/s） | 规划用的最大偏航角速度；云台录像要平稳时可降到 30～40°/s |
| `exploration/relax_time` | 1.0 | 每段轨迹末尾转向视点航向所用的时间（s） |
| `frontier/candidate_rmin` / `candidate_rmax` | 1.5 / 2.5 | 候选视点距前沿团中心的距离范围（m） |
| `fsm/thresh_replan3` | 1.5 | 执行轨迹多久后周期性重规划（s） |
| `manager/max_jerk` | 4 | 规划最大加加速度（m/s³） |
| `optimization/dist0` | 0.7 | 轨迹优化希望与（膨胀后）障碍保持的距离（m），软约束 |
| `sdf_map/p_hit` | 0.65 | 一帧中体素被点云命中时，占据概率的更新量 |
| `sdf_map/p_miss` | 0.35 | 一帧中光线只穿过体素时的更新量；越接近 0.5，障碍越不容易被“擦掉” |
| `sdf_map/p_min` / `p_max` | 0.12 / 0.90 | 占据概率的下限和上限；`p_max` 越高，已确认的障碍越稳定 |
| `sdf_map/p_occ` | 0.80 | 判定为障碍的概率阈值 |

体素从未知到第一次被命中时直接判为障碍；此后每帧按“本帧有命中则 +hit，只被穿过则 +miss”累计。默认值下 hit 与 miss 等量，已确认的障碍连续 2 帧只被穿过就会变回空闲，细小物体（树枝、电线）容易被擦掉。

## 8. 桥接与相机视野裁剪

### 8.1 `src/fuel_bridge/config/bridge.yaml`

| 参数 | 当前值 | 作用 | 调整建议 |
| --- | --- | --- | --- |
| `min_range` | 0.6 | 去掉离雷达原点小于此距离的点（m），主要是机架和桨叶 | 地图里出现机身“鬼影”时增大 |
| `data_timeout` | 0.5 | 点云、里程计、控制器状态的新鲜度要求（s） | |
| `command_timeout` | 0.5 | 规划指令超过此时间未更新则请求悬停（s） | |
| `planning_timeout` | 10.0 | 任务开始后等待第一条指令的最长时间（s） | |
| `reference_jump_limit` | 0.75 | 飞机未跟随轨迹时，参考点离它超过此距离就不转发，等 FUEL 从当前位置重新规划（m） | 不宜过大：控制器会以较大加速度直线飞向参考点，不绕障碍 |

### 8.2 `src/fuel_bridge/config/view_crop.yaml`

| 参数 | 当前值 | 作用 |
| --- | --- | --- |
| `view_crop/enabled` | true | 探索任务期间只把机头前方视野内的点写入地图，规划器按同样视野评估候选视点 |
| `view_crop/horizontal_fov_deg` | 90.0 | 水平视场全角（°），以机头为中心；竖直方向不裁剪 |

裁剪只在探索任务执行期间生效；任务开始前和返航时使用 360° 点云，保证起飞点周围和返航路径已知。关闭时设 `enabled: false`。

## 9. 定位：Faster-LIO 与 EKF

### 9.1 `src/realflight_modules/faster-lio/faster-lio/config/mid360.yaml`

| 参数 | 当前值 | 作用 | 调整建议 |
| --- | --- | --- | --- |
| ★ `mapping/imu_body_R` | 绕 y 轴 15°（机头朝下） | 机体系到 LIO IMU 系的旋转，即雷达安装角度 | 改安装角度时同时改 EKF 的 `rotation_imu` |
| ★ `mapping/imu_body_T` | [0, 0, 0]（**未标定**） | 机体原点在 IMU 系中的位置（m） | 实测填写，同时改 EKF 的 `tr_i`、`imu_trans_*` |
| `mapping/extrinsic_R` / `extrinsic_T` | 单位阵 / [-0.011, -0.02329, 0.04412] | MID360 雷达到其内置 IMU 的外参 | 厂家固定值，不改 |
| `mapping/extrinsic_est_en` | false | 是否在线估计外参 | **必须为 false**，FUEL 依赖固定外参 |
| `publish/scan_publish_en`、`scan_bodyframe_pub_en` | true | 发布去畸变点云 | **必须为 true**，FUEL 订阅此点云 |
| `preprocess/blind` | 0.4 | LIO 忽略近于此距离的点（m） | |
| `point_filter_num` | 3 | 点云降采样间隔 | 增大可降低计算量，但建图更稀疏 |
| `filter_size_surf` / `filter_size_map` | 0.5 / 0.5 | LIO 内部体素滤波尺寸（m） | |

### 9.2 EKF：`ekf_pose/launch/PX4_vio_drone.yaml` 与 `ekf_lidar.launch`

EKF 融合 MID360 内置 IMU 和 LIO 里程计，输出 `/mavros/vision_pose/pose` 给 PX4。

| 参数 | 当前值 | 作用 |
| --- | --- | --- |
| ★ `rotation_imu` | 绕 y 轴 15° | IMU 到机体的旋转，必须与 `mid360.yaml` 的安装角度一致 |
| ★ `tr_i` | [0, 0, 0]（**未标定**） | 机体原点在 IMU 系中的位置（m） |
| ★ `imu_trans_x/y/z`（launch） | 0 / 0 / 0 | 与 `tr_i` 填同一向量 |
| `scale_g` | 9.8 | Livox IMU 加速度单位为 g，乘以此值换算 |
| `offset_px/py/pz`（launch） | 0 / 0 / 0 | 输出位姿的初始偏移（m） |
| `gyro_cov`、`acc_cov`、`position_cov`、`q_rp_cov`、`q_yaw_cov` | 0.05、0.20、0.01、0.01、0.01 | 过程与观测噪声 |
| `cutoff_freq` | 20 | 低通滤波截止频率（Hz） |

## 10. 外部配置（本工程不修改）

- **MAVROS 串口**：`/opt/ros/noetic/share/mavros/launch/px4.launch`，当前 `fcu_url` 为 `/dev/ttyTHS0:921600`。
- **PX4 参数**：外部视觉融合（如 `EKF2_EV_CTRL`、`EKF2_HGT_REF`）和 OFFBOARD 信号丢失后的动作（如 `COM_OBL_RC_ACT`）需在地面站中按 [PX4 1.14 外部定位](https://docs.px4.io/v1.14/en/ros/external_position_estimation) 与 [OFFBOARD](https://docs.px4.io/v1.14/en/flight_modes/offboard) 文档配置。

## 11. 修改参数后

- 只改 YAML 或 launch：重新运行 `start_realflight.sh` 即可，不需要重新编译。
- 改了 C++ 代码：执行 `bash compile_fuel.sh` 后再启动。
- 软件回归测试：`bash scripts/test_fuel.sh`（使用合成数据，不代表实飞验证）。
