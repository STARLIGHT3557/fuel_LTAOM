> 本目录已复制到 uav_explor，随主工程统一管理，无独立 Git 仓库。构建与启动路径见 [集成说明](../../../docs/FASTER_LIO_INTEGRATION.md)。下文保留源工程历史资料。

# workspace_test 部署说明

本目录保留用户已有 Git 仓库，在干净工作区上重新应用已验证配置。

- 当前分支：master。
- Git 远程：https://gitee.com/starsarebright/fast_lio.git
- 修改前 HEAD：e5b5121bfa1278d0c45928f17789fcac5b7885b6
- 工作空间：`/home/ubt/workspace_test`。
- 本次修改保留在工作区，没有执行 commit、push、切换分支或替换 Git 历史。
- 原文件备份：`/home/ubt/workspace_test/.deployment-backups/faster-lio-20260923T081148`。

## 配置

- MID360：`192.168.1.199`；电脑 eth0：`192.168.1.50/24`。
- LIO 使用 `/livox/lidar` 与 `/livox/imu`，无需先启动 MAVROS。
- MID360 内部外参：T = [-0.011, -0.02329, 0.04412] 米，R 为单位阵。
- 向机头下倾 15 度：`imu_body_R = Ry(-15°)`，前提是安装前雷达 X 向机头、Y 向左、Z 向上。
- 位移尚未提供：`imu_body_T = [0,0,0]`。机体输出轴向已转换，但位置原点仍在雷达 IMU 处。
- 关闭无限累计 PCD 保存。
- 保留重力初始化及 EKF 算法源码，包括此前讨论的 mean_gyr_ 旋转代码。

## 修改文件

| 文件 | 内容 |
|---|---|
| livox_ros_driver2/config/MID360_config.json | 电脑 .50、雷达 .199 |
| livox_ros_driver2/src/parse_cfg_file/parse_livox_lidar_cfg.cpp | 从 JSON 读取 IP，取消强制依赖 LIDAR_IP 环境变量 |
| faster-lio/config/mid360.yaml | 内置 IMU、内部外参、15 度机体转换、零位移占位、关闭 PCD 保存 |
| faster-lio/launch/mapping_mid360.launch | BD_LIST 从 env 改为 optenv |
| faster-lio/config/rviz/faster_lio.rviz | world 坐标系、正确话题、轨迹可见度、初始视角 |
| ekf_pose/launch/ekf_lidar.launch | 可选升频输入改为 data_raw 与 odometry_body |
| ekf_pose/launch/PX4_vio_drone.yaml | 标注位移未标定、MAVROS 加速度单位；数值保持原值 |
| scripts/start_mid360.sh | 自动加载本工作空间的启动脚本 |
| DEPLOYMENT_zh.md | 本说明 |

## 编译

```bash
source /opt/ros/noetic/setup.bash
cd /home/ubt/workspace_test
catkin_make -j4 -l4 -DCMAKE_BUILD_TYPE=Release
```

## 启动

推荐使用脚本，它会加载 workspace_test 的 devel 环境：

```bash
/home/ubt/workspace_test/src/realflight_modules/faster-lio/scripts/start_mid360.sh
```

机载电脑图形桌面中同时打开 RViz：

```bash
/home/ubt/workspace_test/src/realflight_modules/faster-lio/scripts/start_mid360.sh rviz:=true
```

也可以手动启动：

```bash
source /opt/ros/noetic/setup.bash
source /home/ubt/workspace_test/devel/setup.bash
rospack find faster_lio
roslaunch faster_lio mapping_mid360.launch rviz:=true
```

`rospack find faster_lio` 应返回包含 `workspace_test` 的路径。不要同时启动旧 workspace 的雷达驱动或定位实例。
启动时保持设备静止，等待 IMU Initial Done；前台按 Ctrl+C 停止。

## 输出及 RViz

- `/livox/imu`：雷达内置 IMU，预期约 200 Hz。
- `/livox/lidar`：原始点云，预期约 10 Hz。
- `/laserMapping/odometry`：world -> imu，预期约 10 Hz。
- `/laserMapping/odometry_body`：world -> body，已转换机体轴向。
- `/laserMapping/cloud_registered`：世界坐标点云，RViz PointCloud2 使用这个话题。
- `/laserMapping/path`：轨迹。

RViz Fixed Frame 和 Views Target Frame 均为 world。
已打开的旧 RViz 要通过 File -> Open Config 加载本目录 `faster-lio/config/rviz/faster_lio.rviz`。

## 可选升频 EKF

`ekf` 包一起编译，输入配置为 `/mavros/imu/data_raw` 和 `/laserMapping/odometry_body`。
此启动脚本不自动运行它，也不向 PX4 发布定位、不修改飞控参数。
启用前仍需确认飞控连接、共同时间基准以及 tr_i / imu_trans_* 位移标定。

## Git 检查

```bash
cd /home/ubt/workspace_test/src/realflight_modules/faster-lio
git status --short
git diff --stat
git diff
```

启动脚本与本文是新增文件，在加入暂存区前只显示在 git status 中，默认 git diff 不展示其内容。
本次不会自动提交，便于用户审阅后自行决定如何记录。

## 日志

编译：`/home/ubt/workspace_test/logs/faster-lio-build.log`。
编译退出码：`/home/ubt/workspace_test/logs/faster-lio-build.exit`。
部署记录：`/home/ubt/workspace_test/logs/faster-lio-deployment.json`。
实机检查完成后：`/home/ubt/workspace_test/logs/mid360-live-check.json`。

## 本次验证结果

- catkin_make 退出码 0；驱动、在线/离线定位、EKF 全部编译通过。
- 三个 ROS 包均解析到 workspace_test，CMake 缓存未引用旧 workspace。
- 约 25 秒实机采样：雷达 IMU 200.03 Hz，点云 10.01 Hz，两路里程计及注册点云约 10.00 Hz。
- 检查期间时间戳递增，位姿和 IMU 数值有限。末尾雷达 IMU pitch 15.003 度，机体 pitch 0.003 度。
- 仅验证雷达数据通路和短时定位输出，不代表飞控融合或飞行验证。
- 测试采用独立 ROS master 11321，测试节点已停止；正常使用按上面的启动命令即可。
- Git HEAD 与 origin 均保持不变；git diff --check 通过。
