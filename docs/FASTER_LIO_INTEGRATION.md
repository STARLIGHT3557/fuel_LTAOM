# uav_explor_LTAOM 中的 MID360 / Faster-LIO 代码

## 目录与来源

完整复制自 `/home/ubt/workspace_test/src/realflight_modules/faster-lio` 的当前文件。
源仓库版本：`494cc2eccd81ee2e7f2fd541b3e1e29773ed4475`。
原目录保留不动；复制目录及其子目录不含 `.git`，统一由 uav_explor_LTAOM 主仓库管理。

```text
src/realflight_modules/faster-lio/
  faster-lio/          Faster-LIO 算法，ROS 包 faster_lio
  livox_ros_driver2/   MID360 雷达驱动，ROS 包 livox_ros_driver2
  ekf_pose/           EKF 与 MAVROS 位姿转发，ROS 包 ekf
  scripts/            本工程定位启动脚本
```

保留全部源码、配置、文档、第三方依赖压缩包及原有 PCD 文件。
复制时 199 个文件逐一通过 SHA-256 校验；随后适配启动脚本并补充迁移说明。
校验清单位于 `build/faster_lio_import_manifest.json`，不纳入版本管理。
原目录内的旧部署文档是历史资料，本工程的路径和构建方法以本文为准。

## 编译

在 `/home/ubt/uav_explor_LTAOM` 执行：

```bash
bash compile_fuel.sh
source source.sh
rospack find faster_lio
rospack find livox_ros_driver2
rospack find ekf
```

三个包应解析到本工程的 `src/realflight_modules/faster-lio`。
构建仍使用 `build/fuel` 和 `devel/fuel`，无需建立第二个工作空间。
Livox SDK2 静态库和头文件、glog、yaml-cpp、TBB、PCL 等系统依赖沿用机载电脑现有安装。

## 启动

完整系统使用 `bash start_realflight.sh`，由单个 sh 调用各模块原有入口，维护位置见 [根目录说明](../README.md#启动)。以下为单模块调试入口。

单独启动 MID360 驱动与定位：

```bash
bash src/realflight_modules/faster-lio/scripts/start_mid360.sh
# 可选 RViz
bash src/realflight_modules/faster-lio/scripts/start_mid360.sh rviz:=true
```

在已连接 MAVROS、已启动定位的环境下，按原实飞流程单独启动 EKF：

```bash
source source.sh
roslaunch ekf ekf_lidar.launch
```

启动 FUEL：

```bash
bash fuel_realflight.sh
```

FUEL 默认读取本工程的
`src/realflight_modules/faster-lio/faster-lio/config/mid360.yaml`。
仍支持通过 `lio_config:=...`、`FAST_LIO_CONFIG` 或 `FAST_LIO_ROOT` 指定其他配置。
FUEL 启动脚本继续复用已运行的 MAVROS、定位和 EKF，避免重复启动。

## 配置

复制保留雷达 IP、话题、IMU 外参、算法与 EKF 参数；未改写原工程或 PX4 参数。
MID360 网络配置位于 `livox_ros_driver2/config/MID360_config.json`。
LIO 外参与发布配置位于 `faster-lio/config/mid360.yaml`。
机体原点位移仍为原配置中的未标定占位值，本文不代表实飞验证。

## 本次集成检查

- 完整 Catkin 编译通过，退出码为 0。
- 雷达驱动、在线/离线定位和 EKF 可执行文件均已生成；加载 ROS 与本工程环境后，动态库依赖完整，无旧 workspace_test 库路径引用。
- 三个 ROS 包均解析到本工程的新目录。
- Shell 语法、launch XML、外参配置生成和三个入口的 roslaunch --nodes 配置解析通过。
- 195 个未适配文件与源目录逐字节一致；4 个适配文件仅涉及启动脚本和文档。
- 源目录与主工程原有 4 个未提交修改保持不变。
- 复制目录无独立 Git 元数据，主仓库归属已确认。
- 构建日志：build/faster_lio_import_build.log。
- 仅进行编译与配置检查，未启动任何雷达、定位、EKF 或飞行节点。
