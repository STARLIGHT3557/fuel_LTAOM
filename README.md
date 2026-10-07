# MID360 + FUEL + PX4 实机探索

运行环境：**Ubuntu 20.04、ROS Noetic、PX4 1.14**。本分支使用机载端已有的 MID360／FAST-LIO 定位和串口 MAVROS，启动 FUEL、任务管理器及 `uav_controller`。

## 编译

```bash
sudo apt-get update
sudo apt-get install ros-noetic-mavros ros-noetic-mavros-msgs ros-noetic-pcl-ros \
  ros-noetic-cv-bridge ros-noetic-rostest libnlopt-dev libnlopt-cxx-dev \
  python3-numpy python3-yaml
bash compile_fuel.sh
```

FUEL 规划源码包含在 `third_party/FUEL/fuel_planner/`，不需要下载 PX4 源码或编译 SITL。构建输出为 `build/fuel/`、`devel/fuel/`。

## 接入机载定位

先按机载端现有配置启动串口 MAVROS、MID360／FAST-LIO，以及将定位送入 PX4 的 EKF／位姿转发节点。本工程复用这些节点；不要重复启动。

已核对 `/home/ubuntu/fast_lio` 中 Faster-LIO 的接口：

| 输入 | 默认话题 | 用途 |
| --- | --- | --- |
| MAVROS 定位 | `/mavros/local_position/odom` | 规划、建图与控制共用 PX4 局部 ENU 坐标 |
| 去畸变点云 | `/laserMapping/cloud_registered_body` | `sensor_msgs/PointCloud2`，实际在 LIO **IMU** 坐标系 |
| 控制反馈 | `/mavros/local_position/pose`、`/mavros/local_position/velocity_local` | 控制器位置、姿态和世界系速度 |
| 飞控状态 | `/mavros/state`、`/mavros/extended_state` | 连接、解锁、模式及着陆状态 |
| 电量 | `/mavros/battery` | 任务管理器的低电量返航 |

启动脚本读取 `~/fast_lio/faster-lio/config/mid360.yaml`，根据 `mapping/imu_body_R/T` 和 `mapping/extrinsic_R/T` 生成统一外参，同时供点云转换和规划器视场检查使用。默认点云帧为 `imu`，输出帧为 MAVROS 默认的 `map`。若机载 MAVROS 使用其他局部帧，传入对应的 `world_frame`，不要通过改点云标签混用坐标系。

**现有 FAST-LIO 配置注明 `imu_body_T` 尚未标定。** 应将实际机体原点在 LIO IMU 系中的位置填回机载定位配置；下次启动本工程会重新读取。本工程不会修改外部定位或 MAVROS 文件。

FAST-LIO 必须启用 `publish/scan_publish_en`、`publish/scan_bodyframe_pub_en`，并保持 `mapping/extrinsic_est_en: false`。这里使用已有去畸变点云，不直接订阅 Livox 驱动的 `CustomMsg`，也不使用 FAST-LIO 世界系点云。

## 启动

在工程根目录执行：

```bash
bash fuel_realflight.sh
```

可覆盖定位配置、控制器参数、坐标系及规划边界：

```bash
bash fuel_realflight.sh \
  lio_config:=/home/ubuntu/fast_lio/faster-lio/config/mid360.yaml \
  controller_params:=/你的路径/已标定控制器参数.yaml \
  world_frame:=map rviz:=false \
  box_min_x:=-5 box_max_x:=5 box_min_y:=-5 box_max_y:=5 \
  box_min_z:=0.5 box_max_z:=2.0 max_vel:=0.5 max_acc:=0.5
```

默认探索框为局部 x/y `[-5,5]` m、z `[0.5,2]` m，地图分配为 `20×20×6` m，速度和加速度上限为 `0.5 m/s`、`0.5 m/s²`。这些是初始配置，必须按实际场地设置。RViz 可通过 `rviz:=false` 关闭。

`config/uav_controller_realflight.yaml` 延续原控制参数，其中增益和 `hover_percentage: 0.258` **尚未在这架实机验证**。使用机载端已标定参数时传 `controller_params`；启动入口始终启用 FUEL 的 `direct_trajectory` 模式。配置不自动写入 PX4 参数。

## 开始、停止与返航

在起飞前启动建图。由操作者解锁、手动起飞并稳定悬停，再切换 OFFBOARD；控制器锁定切换时的位置。确认：

```bash
source source.sh
rostopic echo -n 1 /mavros/state
rostopic echo -n 1 /uav_controller/fsm_status
```

MAVROS 应为 `OFFBOARD`、`armed: True`，控制器为 `HOLD`、`CLOSED_LOOP`。等 FUEL 进入 `WAIT_TRIGGER` 后，点击 RViz 的 **2D Nav Goal**，或调用：

```bash
rosservice call /fuel/start "{}"
rosservice call /fuel/stop "{}"          # 取消任务并请求悬停
rosservice call /fuel/return_home "{}"   # 人工请求规划返航
```

默认任务管理器在探索完成、有效电量低于 30% 或探索超时保护触发时规划返航，在本地 `[0,0,1]` m 悬停；不会自动降落或上锁。返航目标和电量阈值在 `src/fuel_bridge/config/mission.yaml`，也可通过 `mission_config:=/路径/任务参数.yaml` 覆盖。固定原点必须对应实际希望返回的位置。

PX4 的外部定位融合和 OFFBOARD 丢失后的行为由已有飞控配置负责，版本说明见 [PX4 1.14 外部定位](https://docs.px4.io/v1.14/en/ros/external_position_estimation) 与 [OFFBOARD](https://docs.px4.io/v1.14/en/flight_modes/offboard)。

接口、转换细节与测试见 [FUEL 桥接说明](src/fuel_bridge/README.md)，本次检查结果见 [软件验证记录](docs/REALFLIGHT_VALIDATION.md)。
