# FUEL 专用分支重构记录

分支：`refactor/fuel`。起点：`dfd9134`（包含 `b950945` 的 FUEL 接入及后续 README 更新）。日期：2026-09-18。

## 保留范围与目录变化

只支持 Gazebo P600＋MID360＋GPS/PX4/MAVROS＋单机 FUEL。根目录保留一个编译入口 `compile_fuel.sh`、一个仿真入口 `fuel_simulation.sh` 和环境脚本 `source.sh`。

| 原位置/功能 | 本分支 |
| --- | --- |
| `build/fuel_upstream` 下载的 FUEL | 完整源码纳入 `third_party/FUEL/`，1538 个上游文件全部保留，原修复直接合入 |
| `Modules/fuel_bridge` | `src/fuel_bridge` |
| `Modules/uav_control` | 由 `src/uav_controller` 的 `ros1` 分支替换，FUEL 使用直连轨迹模式 |
| `Modules/common` | `src/messages` 保留现有消息定义；FUEL 控制链只使用 `PositionCommand` |
| `Simulator/gazebo_simulator` | `src/gazebo_simulator`，仅保留 P600、MID360、GPS、圆柱世界及引用资源 |
| `Simulator/livox_laser_gazebo_plugins` | `src/livox_laser_gazebo_plugins`，只保留实际使用的 PointCloud2 XYZ 和建图射线输出 |
| SUPER、EGO、FAST_LIO、通信、Experiment、旧 Scripts | 删除 |
| RealSense、Velodyne、其他机型/场景/传感器配置 | 删除 |
| PID/UDE/NE、遥控器/摇杆/轨迹测试、地面站参数管理、其他定位源 | 删除 |
| 两套工作空间、FUEL/RViz 环境切换脚本 | 合并为一个 Catkin 工作空间，删除多余切换脚本 |

完整 FUEL 的 `uav_simulator/`、演示及资源仍在仓库，这是用户明确要求保留的上游完整源码。它们不加入本工程的 Catkin 包搜索路径；`src/fuel` 只链接 `fuel_planner/`，避免重复的消息包和仿真器。未使用的演示可执行目标不参加默认构建。许可证、源版本及文件清单见 [FUEL 来源说明](../third_party/FUEL/LOCAL_CHANGES.md)。

## 控制行为

FUEL 的 `quadrotor_msgs/PositionCommand` 由桥接节点直接转换为 `uav_controller/TrajectorySetpoint`。新控制器使用 ROS 1/MAVROS 接口，几何控制仍按位置、速度反馈与加速度前馈计算姿态和推力；P600 集成参数位于 [uav_controller_p600.yaml](../src/fuel_bridge/config/uav_controller_p600.yaml)。

FUEL 启用 `fsm/direct_trajectory`：无人机在空中切入 OFFBOARD 时先锁定当前位置进入 `HOLD`；新鲜轨迹到达后进入 `TRAJECTORY`；`/fuel/stop`、规划超时或轨迹超时后锁定位置回到 `HOLD`。之后可再次触发探索。状态失效时停止输出，由 PX4 处理 offboard 链路丢失。

定位改用 MAVROS 原生 `/uav1/mavros/local_position/odom`。MAVROS 局部坐标帧配置为 `world`，与点云桥接和 FUEL 一致。旧 `prometheus_uav_control` 包及 UAVCommand 控制链已删除。

## 可移植性

MID360 网格、扫描 CSV 与 GPS 模型纳入本仓库。雷达插件通过 `model://MID360/scan_mode/mid360.csv` 查找数据；移除用户家目录绝对路径及固定 `libprotobuf.so.9` 链接，改由 Gazebo 开发库解析依赖。

PX4 保持外部依赖，通过 `PX4_ROOT`、`PX4_BUILD_DIR` 配置。启动脚本直接执行所选构建目录内的 PX4，避免 `rosrun` 意外找到另一个构建。模型生成只向标准输出写入，不在源码目录生成 SDF。

构建源目录为 `src/`，新产物在 `build/fuel` 和 `devel/fuel`，不复用旧编译缓存。旧的本地构建目录未纳入 Git，也未作为本次构建输入；源码和 Git 分支可独立恢复。

## 验证

- 从空的 `build/fuel`、`devel/fuel` 完整编译通过；随后构建边界回归可执行文件通过。
- roslaunch 节点和参数解析通过，启动图包含 `uav_controller/geometric_controller_node`，不再启动旧控制器。
- 完整 FUEL 上游清单核对：1538 个路径全部存在。
- `uav_controller` 核心直连测试验证空中接管、轨迹开始、完成、重启和超时悬停，通过。
- `bridge.test`：点云转换/过滤、RViz 触发、边界及停止保护，通过。
- `frontier_boundary.test`：4 个实际地图/前沿回归用例，通过。
- `planner.test`：实际 FUEL 建图、前沿、B 样条及控制指令链路，通过。

- `scripts/check_simulation.py`：替换控制器后的 Gazebo/PX4 无界面检查通过；验证 MAVROS 里程计、真实回波/40 m 无回波射线、OFFBOARD/解锁、新控制器自动起飞、RViz 触发、FUEL 轨迹接管及停止悬停。10 秒探索窗口内最远移动 8.62 m，日志目录为 `/tmp/fuel-simulation-check.jz37ocue`（临时文件，不入库）。
- 无界面脚本在切换 OFFBOARD 前等待 2 秒设定值流，并按新接口检查 `FlightStatus` 和 MAVROS odom。

复现命令见 [根 README](../README.md)。此次实际飞行是短程验收，没有验证整场覆盖、长时间边界行为、所有终止场景或 GUI 渲染画面。规划过程中仍可能出现上游的 NLopt 优化失败重试日志，本次短程检查未因此中断。
