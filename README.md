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

FUEL 规划源码直接位于 `src/fuel/`，不需要下载 PX4 源码或编译 SITL。构建输出为 `build/fuel/`、`devel/fuel/`。

## 接入机载定位

`start_realflight.sh` 统一启动串口 MAVROS、MID360／Faster-LIO、EKF／位姿转发和 FUEL。单独调试 FUEL 时，`fuel_realflight.sh` 继续复用已有定位节点。

已集成本工程 `src/realflight_modules/faster-lio` 中的 MID360 驱动、Faster-LIO 和 EKF，接口如下：

| 输入 | 默认话题 | 用途 |
| --- | --- | --- |
| MAVROS 定位 | `/mavros/local_position/odom` | 规划、建图与控制共用 PX4 局部 ENU 坐标 |
| 去畸变点云 | `/laserMapping/cloud_registered_body` | `sensor_msgs/PointCloud2`，实际在 LIO **IMU** 坐标系 |
| 控制反馈 | `/mavros/local_position/pose`、`/mavros/local_position/velocity_local` | 控制器位置、姿态和世界系速度 |
| 飞控状态 | `/mavros/state`、`/mavros/extended_state` | 连接、解锁、模式及着陆状态 |
| 电量 | `/mavros/battery` | 任务管理器的低电量返航 |

启动脚本读取 `src/realflight_modules/faster-lio/faster-lio/config/mid360.yaml`，根据 `mapping/imu_body_R/T` 和 `mapping/extrinsic_R/T` 生成统一外参，同时供点云转换和规划器视场检查使用。默认点云帧为 `imu`，输出帧为 MAVROS 默认的 `map`。若机载 MAVROS 使用其他局部帧，传入对应的 `world_frame`，不要通过改点云标签混用坐标系。

**现有 FAST-LIO 配置注明 `imu_body_T` 尚未标定。** 应将实际机体原点在 LIO IMU 系中的位置填回机载定位配置；下次启动本工程会重新读取。本工程不会修改外部定位或 MAVROS 文件。

FAST-LIO 必须启用 `publish/scan_publish_en`、`publish/scan_bodyframe_pub_en`，并保持 `mapping/extrinsic_est_en: false`。这里使用已有去畸变点云，不直接订阅 Livox 驱动的 `CustomMsg`，也不使用 FAST-LIO 世界系点云。

## 启动

在工程根目录执行：

```bash
bash start_realflight.sh
```

脚本依次调用原有 MAVROS、MID360／Faster-LIO、EKF 的 launch，以及原有 `fuel_realflight.sh`；按 Ctrl+C 统一停止本脚本启动的进程。参数沿用各模块原有 launch。FUEL 的 RViz 默认开启，无图形环境时执行 `bash start_realflight.sh rviz:=false`。
启动前接通雷达网线，电脑网口配置为 `192.168.1.50/24`，雷达地址为 `192.168.1.199`。

实地飞行及手持静态建图/PCD 续用的完整步骤见[实地飞行与手持静态测试指南](docs/实地飞行与手持静态测试README.md)。

查看已保存的 FUEL 拼接点云（无需启动飞控或雷达）：

```bash
bash scripts/view_saved_pcd.sh
# 指定 LTAOM 点云；frame_id 必须与该 PCD 所在坐标系一致
bash scripts/view_saved_pcd.sh "$HOME/fuel_sim_logs/ltaom/map_prior/clouds_corrected.pcd" world
```

维护时直接修改对应配置，每个参数的含义、当前值和调整建议见 [实飞操作与参数说明](docs/REALFLIGHT_PARAMETERS.md)：

- 飞控串口：MAVROS 原有 `/opt/ros/noetic/share/mavros/launch/px4.launch`，当前为 `/dev/ttyTHS0:921600`。
- RViz、探索范围、速度和加速度：`src/fuel_bridge/launch/realflight.launch`。
- 控制器参数：`src/fuel_bridge/config/uav_controller_realflight.yaml`。
- 任务和返航参数：`src/fuel_bridge/config/mission.yaml`。
- FUEL 建图、前沿筛选和雷达视场参数：`src/fuel_bridge/config/exploration.yaml`。
- 相机视野裁剪：`src/fuel_bridge/config/view_crop.yaml`。
- 雷达网络与定位外参：`src/realflight_modules/faster-lio` 内对应的 JSON / YAML。

当前最大速度为 `1.0 m/s`、最大加速度为 `0.4 m/s²`，控制器悬停推力参数为 `0.40`。
单独调试 FUEL 时仍可使用 `fuel_realflight.sh`，它复用已运行的 MAVROS、定位和 EKF。

## 开始、停止与返航

在起飞前启动建图。由操作者解锁、手动起飞并稳定悬停，再切换 OFFBOARD；控制器锁定切换时的位置。也可以解锁后在地面直接切 OFFBOARD 自动起飞，见[实飞操作与参数说明](docs/REALFLIGHT_PARAMETERS.md)。手动起飞时确认：

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

默认任务管理器在探索完成、有效电量低于 20% 或探索超时保护触发时规划返航，在本地 `[0,0,1]` m 悬停；不会自动降落或上锁。返航目标和电量阈值在 `src/fuel_bridge/config/mission.yaml`，也可通过 `mission_config:=/路径/任务参数.yaml` 覆盖。固定原点必须对应实际希望返回的位置。

PX4 的外部定位融合和 OFFBOARD 丢失后的行为由已有飞控配置负责，版本说明见 [PX4 1.14 外部定位](https://docs.px4.io/v1.14/en/ros/external_position_estimation) 与 [OFFBOARD](https://docs.px4.io/v1.14/en/flight_modes/offboard)。

接口、转换细节与测试见 [FUEL 桥接说明](src/fuel_bridge/README.md)，本次检查结果见 [软件验证记录](docs/REALFLIGHT_VALIDATION.md)。

MID360 驱动、定位与 EKF 的统一编译和启动方法见 [Faster-LIO 集成说明](docs/FASTER_LIO_INTEGRATION.md)。复制目录不包含独立 Git 仓库。
