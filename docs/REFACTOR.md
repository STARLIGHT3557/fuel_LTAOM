# FUEL 专用分支重构记录

分支：`refactor/fuel`。起点：`dfd9134`（包含 `b950945` 的 FUEL 接入及后续 README 更新）。日期：2026-09-18。

## 保留范围与目录变化

只支持 Gazebo P600＋MID360＋GPS/PX4/MAVROS＋单机 FUEL。根目录保留一个编译入口 `compile_fuel.sh`、一个仿真入口 `fuel_simulation.sh` 和环境脚本 `source.sh`。

| 原位置/功能 | 本分支 |
| --- | --- |
| `build/fuel_upstream` 下载的 FUEL | 完整源码纳入 `third_party/FUEL/`，1538 个上游文件全部保留，原修复直接合入 |
| `Modules/fuel_bridge` | `src/fuel_bridge` |
| `Modules/uav_control` | `src/uav_control`，保留实际轨迹控制计算及 GPS 状态适配 |
| `Modules/common` | `src/messages`，仅保留 UAVCommand/UAVState/UAVControlState/PositionCommand；保留使用中消息的原定义 |
| `Simulator/gazebo_simulator` | `src/gazebo_simulator`，仅保留 P600、MID360、GPS、圆柱世界及引用资源 |
| `Simulator/livox_laser_gazebo_plugins` | `src/livox_laser_gazebo_plugins`，只保留实际使用的 PointCloud2 XYZ 和建图射线输出 |
| SUPER、EGO、FAST_LIO、通信、Experiment、旧 Scripts | 删除 |
| RealSense、Velodyne、其他机型/场景/传感器配置 | 删除 |
| PID/UDE/NE、遥控器/摇杆/轨迹测试、地面站参数管理、其他定位源 | 删除 |
| 两套工作空间、FUEL/RViz 环境切换脚本 | 合并为一个 Catkin 工作空间，删除多余切换脚本 |

完整 FUEL 的 `uav_simulator/`、演示及资源仍在仓库，这是用户明确要求保留的上游完整源码。它们不加入本工程的 Catkin 包搜索路径；`src/fuel` 只链接 `fuel_planner/`，避免重复的消息包和仿真器。未使用的演示可执行目标不参加默认构建。许可证、源版本及文件清单见 [FUEL 来源说明](../third_party/FUEL/LOCAL_CHANGES.md)。

## 控制行为

保留 `TRAJECTORY_ATTITUDE`（9）的原始数值计算：位置/速度反馈、8 m/s² 反馈限幅、加速度前馈、姿态转换、35–45 ms 数据窗口的 RLS 推力估计、0.01 推力偏置及 [原增益](../src/uav_control/config/p600.yaml)。保留 `XYZ_POS`（0）定点起飞、悬停及降落，其余控制算法/移动模式删除。

清理时修正两处旧状态处理错误，并为输入补上新鲜度检查：

- 旧主循环无条件写入 `COMMAND_CONTROL`，覆盖降落/保护状态。现在只有解锁、OFFBOARD 且定位有效时进入指令控制，降落状态保持到上锁。
- 旧悬停发送函数每帧用当前位置覆盖目标。现在触发停止时锁定位置和朝向，后续保持这个固定目标。
- 轨迹中断超过 0.5 s 请求悬停。状态/定位失效或越过控制器围栏触发保护；失去连接后停止设定值，由 PX4 自身处理链路丢失。
- PX4 改为其他飞行模式时取消旧轨迹，重新进入 OFFBOARD 后悬停，不恢复旧轨迹。
- GPS/位置/速度/IMU 必须持续更新，失效时不再发布伪装成新数据的里程计。世界到机体、雷达射线原点 TF 与当前模型外参一致。

消息的旧枚举仍保留，以保持原 MD5 和类型兼容；未使用的枚举不表示功能仍可用。控制状态中的历史 `pos_controller=0` 同样保留，其轨迹模式 9 的实际算法见上文。

## 可移植性

MID360 网格、扫描 CSV 与 GPS 模型纳入本仓库。雷达插件通过 `model://MID360/scan_mode/mid360.csv` 查找数据；移除用户家目录绝对路径及固定 `libprotobuf.so.9` 链接，改由 Gazebo 开发库解析依赖。

PX4 保持外部依赖，通过 `PX4_ROOT`、`PX4_BUILD_DIR` 配置。启动脚本直接执行所选构建目录内的 PX4，避免 `rosrun` 意外找到另一个构建。模型生成只向标准输出写入，不在源码目录生成 SDF。

构建源目录为 `src/`，新产物在 `build/fuel` 和 `devel/fuel`，不复用旧编译缓存。旧的本地构建目录未纳入 Git，也未作为本次构建输入；源码和 Git 分支可独立恢复。

## 验证

- 从空的 `build/fuel`、`devel/fuel` 完整编译通过；随后构建边界回归可执行文件通过。
- roslaunch 节点和参数解析通过，启动图只含一个 RViz。模型和控制器包可以在统一环境解析；`resource_retriever` 成功读取 6,365,657 字节的 RViz mesh；共享库检查无缺失项。
- 完整上游清单核对：1538 个路径全部存在，无嵌套 `.git` 或子模块。
- `controller.test`：实际状态适配器和控制器，验证起飞位置指令、模式 9 姿态计算、超时悬停、固定悬停点、拒绝已删除模式、降落保持及定位失效保护，通过。
- `bridge.test`：点云转换/过滤、RViz 触发、边界及停止保护，通过。
- `frontier_boundary.test`：4 个实际地图/前沿回归用例，通过。
- `planner.test`：实际 FUEL 建图、前沿、B 样条及控制指令链路，通过。

- `scripts/check_simulation.py`：实际 Gazebo/PX4 无界面检查通过，GPS、真实回波/40 m 无回波射线、无人机 Marker、OFFBOARD/解锁、1.5 m 起飞悬停、RViz 2D Nav Goal 话题触发、模式 9 飞行与停止后固定点悬停均通过。探索观察窗口 30 秒墙钟时间，距检查起点最远约 21.28 m；无空前沿 ADD Marker、无控制器 failsafe、停止后不再转发轨迹。日志目录为本机 `/tmp/fuel-simulation-check.efft_orv`（临时文件，不入库）。
- 首次短程脚本过早请求 OFFBOARD 导致等待超时；脚本增加了 2 秒仿真时间的设定值预发送及模式请求重试后通过。该要求已写入启动说明。

复现命令见 [根 README](../README.md)。此次实际飞行是短程验收，没有验证整场覆盖、长时间边界行为、所有终止场景或 GUI 渲染画面。规划过程中仍可能出现上游的 NLopt 优化失败重试日志，本次短程检查未因此中断。
