# FUEL / Prometheus P600 MID360

本模块接入完整 [HKUST FUEL](https://github.com/HKUST-Aerial-Robotics/FUEL)：前沿搜索、探索任务规划、B 样条轨迹规划和轨迹执行。Gazebo、PX4、MAVROS 和 Prometheus 控制器复用当前工程。

接入过程中各项报错、原因、修复与验证记录见 [仿真问题与解决记录](../../docs/TROUBLESHOOTING.md)。

## 编译

依赖准备见 [项目 README](../../README.md)。从工程根目录编译全部节点和插件：

```bash
bash compile_fuel.sh
```

完整 FUEL 源码已纳入 `third_party/FUEL/`，固定来源提交为 `662dd23c7b52b258d3c4a0155ff6632118e8984f`，修复直接合入源码。`src/fuel` 链接到其中的规划模块；上游演示和仿真器完整保留，但不参与本工程构建。构建输出为 `build/fuel/`、`devel/fuel/`，默认两个任务，可设置 `FUEL_BUILD_JOBS=4`。

所有节点使用同一 Catkin 工作空间，运行 `source source.sh` 即可加载环境。无需下载上游或手动编译旧工程。

## 启动和探索

关闭已有仿真，然后在工程根目录运行：

```bash
bash fuel_simulation.sh
```

切换到 XTDrone 的 `indoor4` 迷宫场景（先关闭已有仿真）：

```bash
bash fuel_simulation.sh scene:=indoor4
```

默认仍为圆柱场，也可使用 `scene:=cylinder`。`indoor4` 包含 150 面墙，场地约 68×54 m；出生点为 Gazebo `(0, -1)`，起飞指令仍使用局部坐标 `[0, 0, 1.5]`。入口自动设置 80×64×6 m 地图，局部探索边界为 x∈[-31,39]、y∈[-26,29]、z∈[0.5,2.0] m。墙高 2.5 m，探索目标高度上限设为 2 m。自定义出生点时边界会自动换算，若超出地图分配范围，还需扩大 `map_size_x/map_size_y`。

此场景已通过 launch 参数解析及静态资源检查；实际飞行验证因系统磁盘空间不足尚未完成。现有 `scripts/check_simulation.py` 使用圆柱场边界，仅适用于默认圆柱场。


它启动原 P600/MID360 Gazebo 场景、MAVROS、Prometheus 控制器和 FUEL。FUEL 自己维护占据栅格和 ESDF，不依赖 OctoMap，也不需要启动 `3dlidar_to_octomap.launch` 或 SUPER。只打开一个 FUEL RViz，默认显示世界坐标点云、地图、前沿、无人机模型和黄色探索边界框；`rviz:=false` 可关闭 RViz。模型复用 Prometheus 发布的 `hummingbird.mesh` 示意模型，并非 Gazebo P600 的完整外观。所有节点及模型资源在同一工作空间解析。

按 [根目录启动说明](../../README.md) 切换 OFFBOARD、解锁并起飞到约 1.5 m，等待无人机悬停、FUEL 日志出现 `WAIT_TRIGGER` 且 RViz 中已有地图。点击 RViz 工具栏的 **2D Nav Goal**，在地图上按住左键拖出方向箭头并松开，即可开始探索，无需调用服务。

FUEL 应在起飞前就开始接收点云，保留起飞过程的地图。MID360 的下视角很小且雷达高于机体，仅在空中静止时启动建图，机体附近可能仍是未知空间，导致 `No path to next viewpoint`。

需要命令行触发时，也可使用：

```bash
source /opt/ros/noetic/setup.bash
rosservice call /fuel/start "{}"
```

RViz 的 **2D Nav Goal** 发布到 `/move_base_simple/goal`，可在 **Tool Properties** 中检查。点击位置只用于启动探索，探索目标由 FUEL 自主选择。若条件未满足，终端会打印 `RViz 2D Nav Goal did not start exploration` 及原因；满足条件后重新点击即可。桥接节点检查最近的定位、点云、控制状态、起飞高度和悬停速度；条件未满足时会返回失败原因，不自动解锁或起飞。

停止探索并请求当前位置悬停：

```bash
rosservice call /fuel/stop "{}"
```

悬停后可再次调用 `/fuel/start`。规划器会保留已建地图，从当前位置重新规划。不要同时运行 SUPER/EGO 的轨迹服务器，以免多个节点向同一控制器发指令。

## 数据接口与参数

| 接口 | 默认话题 / 类型 |
| --- | --- |
| 定位 | `/uav1/mavros/local_position/odom`，`nav_msgs/Odometry` |
| 原始点云 | `/livox/lidar`，`sensor_msgs/PointCloud2`，`uav1/lidar_link` 坐标系 |
| 仿真建图输入（默认） | `/livox/lidar_mapping`，真实回波及 40 m 无回波射线端点 |
| RViz 真实回波 | `/fuel/cloud_hits_world`，不显示无回波端点 |
| 转换后的建图射线与传感器位姿 | `/fuel/cloud_world`、`/fuel/sensor_pose`，相同时间戳、`world` 坐标系 |
| FUEL 轨迹指令 | `/fuel/position_cmd`，本工程的 `quadrotor_msgs/PositionCommand` |
| 控制器轨迹输入 | `/uav_controller/trajectory_setpoint`，`uav_controller/TrajectorySetpoint` |
| 控制状态 | `/uav_controller/fsm_status`，`uav_controller/FlightStatus` |
| FUEL 地图、前沿 | `/sdf_map/occupancy_all`、`/planning_vis/frontier` |
| 探索边界框 | `/fuel/exploration_bounds`，黄色线框，局部定位坐标系 |

`config/bridge.yaml` 设置传感器外参、滤波和超时。实际加载的模型是本工程 `src/gazebo_simulator/gazebo_models/uav_models/p600_mid360/p600_mid360.sdf.jinja`。其中雷达安装位置为 `(0.13, 0, 0.23)` m、俯仰角为 `0.35 rad`；叠加雷达局部 z 轴上 `0.05 m` 的射线原点偏移后，桥接外参为 `(0.1471448904, 0, 0.2769686356)` m、俯仰角 `0.35 rad`。修改模型后必须同步修改桥接外参及 `mid360.yaml` 中的雷达位置和安装角；候选观察点的视场、遮挡射线均从实际雷达位置计算。输入必须是雷达局部坐标点云，不能将世界坐标点云再次送入转换。同步时间差上限为 50 ms；桥接节点剔除非有限点和 0.6 m 内点，过滤旧话题的零距离点及机体点。

仿真插件通过独立的 `/livox/lidar_mapping` 保留没有碰到物体的射线方向，端点位于模型最大量程 40 m；零距离、近距离和 NaN 不会作为清空射线。FUEL 将超出 `sdf_map/max_ray_length` 的端点截断，仅更新沿途自由空间，不把截断端点标成障碍物。当前建图量程 10 m 必须小于传感器量程 40 m。`bridge.yaml` 的 `no_return_range` 与模型最大量程保持一致，用于从 RViz 真实回波显示中排除这些端点。

原 `/livox/lidar` 保持原有输出。对照实验使用仅有真实回波的旧话题时可传 `cloud_topic:=/livox/lidar`，但无法凭丢失的射线方向清除无回波区域，不能把零点当作远距离自由空间。

`config/mid360.yaml` 设置 0.2 m 地图分辨率、0.4 m 障碍膨胀、10 m 建图射线范围及 MID360 360° 水平、约 -7°～53° 垂直视场。桥接将 FUEL 的位置、速度、加速度和 yaw 直接转换为新控制器的 `TrajectorySetpoint`；`config/uav_controller_p600.yaml` 启用 `direct_trajectory`，并保留 P600 已验证的增益与悬停油门。

默认地图分配尺寸为 48×32×6 m，探索区域按 `planning_easy2.world` 的圆柱场及西侧起飞入口设置。圆柱中心约覆盖 Gazebo x ∈ [-6.89, 11.00]、y ∈ [-9.87, 8.38] m；包含圆柱半径、周边余量和出生点后的探索矩形为：

| 坐标系 | x 范围 / m | y 范围 / m |
| --- | --- | --- |
| Gazebo 场景坐标 | [-11, 12] | [-11, 10] |
| Prometheus/FUEL 局部坐标（默认出生点 -10, 0） | [-1, 22] | [-11, 10] |

探索高度使用局部定位 z ∈ [0.5, 2.5] m。PX4 局部原点在出生点附近，因此当前场景满足 `x_local ≈ x_gazebo + 10`、`y_local ≈ y_gazebo`，有少量估计误差。虽然消息中的坐标系名称都是 `world`，Gazebo 真值与 Prometheus 定位的数值原点不同，不能直接混用。

`simulation.launch` 中 `spawn_x/spawn_y` 同时控制出生位置和默认边界换算；`box_min/max_*` 参数仍然是局部定位坐标。黄色框表示前沿搜索及轨迹优化范围。桥接节点不再拦截范围外的轨迹指令，运行边界由 FUEL 规划参数负责；启动探索时当前位置仍必须在框内。默认保持原出生点，允许沿入口飞入圆柱区。

原始 `MID360 world cloud` 会显示传感器看到的框外物体，这些点不代表框外探索目标；可关闭该显示项，仅查看框内的 `FUEL occupancy`。调整边界时应使其位于地图分配范围内，例如：

```bash
bash fuel_simulation.sh box_min_x:=-1.0 box_max_x:=22.0 \
  box_min_y:=-11.0 box_max_y:=10.0 max_vel:=1.0 max_acc:=1.0
```

可传入 `uav_id`、`odom_topic`、`cloud_topic`、`lidar_frame`、`map_size_x/y/z`、`box_min/max_x/y/z`、`max_vel`、`max_acc` 和 `rviz`，仿真总启动还支持 `spawn_x/spawn_y`。本工程只支持单机，模型与默认 `lidar_frame` 随 `uav_id` 一致设置。RViz 的模型和里程计话题按 `uav_id`、`odom_topic` 自动重映射。

`WAIT_TRIGGER` / `wait for trigger` 表示正常等待探索指令。若出现 `FUEL frame mismatch`，日志会同时列出收到与期望的坐标系；用 `rostopic echo -n 1 /livox/lidar/header` 核对。不要仅为消除报错将原始点云标记为 `world`，它仍需经过外参和里程计转换。

在已有 Gazebo/控制器运行时，可只启动 FUEL：

```bash
src/fuel_bridge/scripts/fuel_env.sh roslaunch prometheus_fuel fuel.launch
```

`fuel_env.sh` 自动准备可写的 TSP 临时目录，默认 `build/fuel_runtime`，可通过环境变量 `FUEL_TSP_DIR` 覆盖。

## 边界附近反复绕圈

前沿指已知自由空间旁边的未知栅格。本接入已修正 FUEL 的边界判断：只有探索框内的未知邻居才形成前沿，同时排除落在框上界之外的搜索种子。这样，框内已经观测完成后，框外未知区域不会继续产生探索目标；框内的真实未知区域仍然保留。该修复已合入 `third_party/FUEL/fuel_planner/active_perception/src/frontier_finder.cpp`，需要通过 `bash compile_fuel.sh` 编译生效。

框内前沿仍不消失时，应首先检查建图输入：旧 `/livox/lidar` 丢弃了没有碰到物体的射线，开阔处扫过后仍可能保留未知体素。默认已改用 `/livox/lidar_mapping`，并修正观察点可见性中的雷达安装位置。日志后期始终只剩少量前沿、不断出现 `Replan: traj fully executed` 时，单纯调低重规划频率不能消除这类目标。

FUEL 以观测未知区域为目标，不要求机体飞过每个网格。中心如果已经被周围扫描覆盖，可以不进入中心；完整覆盖巡航需要另一种任务目标。仅凭障碍物点云和历史轨迹，不能判断中心的自由空间是否已知。

先使用现有参数验证上述修复。如果仍频繁更换局部目标，再检查终端的 `Replan:` 原因，在 `config/mid360.yaml` 中逐项试调：

| 参数 | 当前有效值 | 适用情况与试调方向 |
| --- | --- | --- |
| `frontier/cluster_min` | 30 | 若剩余目标是零散小簇，可先试 50；会忽略部分小型未知区域 |
| `frontier/min_visib_num` | 10 | 若候选点只能看到少量前沿，可先试 15；会减少可用观察点 |
| `frontier/min_view_finish_fraction` | 0.2（上游默认） | 若大量出现 `Replan: cluster covered`，可先试 0.4，减少小幅地图变化触发的重规划 |
| `fsm/thresh_replan3` | 1.5 s（上游默认） | 仅在 `Replan: periodic call` 过于频繁时试 2.5 s；会降低定期更新目标的频率 |

这些是后续对照实验的起点，并未作为新的默认参数。`min_view_finish_fraction` 是触发重规划的前沿变化比例，不是任务完成覆盖率。不要靠扩大探索框或提高速度掩盖问题。若 RViz 规划轨迹正常、实际无人机却绕圈，应检查轨迹跟踪误差，前沿参数不能解决该类问题。

## 持续返航与恢复

任务管理器在探索完成、电量低于 30% 或探索规划持续失败时，锁定返回本地 ENU `(0, 0, 1)` 的任务。`/fuel/return_home` 可人工触发；`/fuel/stop` 明确取消任务并请求悬停。返航过程中重复启动返航不会创建新批次，电量回升也不会恢复探索。

返航没有失败次数上限，也没有因恢复超时而终止的状态。一次规划失败、轨迹交接被拒绝或输出超时后，任务进入 `RETURN_RECOVERY`，保留原批次与返航目标。FUEL 按 `return_home/retry_interval` 间隔重新搜索，并在失败后采用更短的局部轨迹段和路径点优化。失败候选不会覆盖执行中的轨迹；碰撞检查覆盖重试和等待交接期间的已执行轨迹。发现旧轨迹不安全时立即请求制动，同时继续规划；有效新轨迹获准后退出制动并继续返航。

`return_home/planning_timeout`（默认 10 秒）表示多久没有新返航轨迹接管时重新请求同批次规划，不是整个返航任务的时间上限。桥接因超时关闭转发后，同批次的 `REPLAN` 会恢复接入。人工停止或到家悬停已关闭的任务不会被延迟重试消息重新启动。

`/fuel/mission_status` 中 `RETURN_RECOVERY=5` 是非终止状态；`return_reason` 保留低电量或人工返航等原始原因，`recovery_reason` 描述本次恢复原因，`recovery_attempts` 统计管理器发出的重规划请求。返航只在末段参考完成，实际位置、速度、零偏航和偏航角速度持续达标后进入 `HOME_HOLD`，或由人工停止。地图中暂时没有可行通路时会继续尝试，不会忽略障碍或未知空间强行直飞原点。

`fuel.launch` 的 `odom_twist_in_body` 默认 `true`，对应 MAVROS `/local_position/odom` 的机体系线速度。规划器和轨迹执行器先按姿态旋转到世界 ENU 系，再用于规划、交接和制动；若接入已输出世界系速度的自定义里程计，应显式设为 `false`。控制器仍使用原有的世界系速度输入。

返航末段在距目标约 3.5 m 内尝试平滑收敛，速度较高时按制动距离扩大介入范围。采用起止位置、速度、加速度连续的三次 B 样条，平移时长至少 2 秒，终点精确为 home 且速度、加速度为零。偏航以最短角度归零，目标角速度/角加速度上限为 0.6 rad/s、0.8 rad/s²；若初始导数更大则保持交接连续。转向需要更长时间时，延长原点的静止段，不拉长制动路径。

末段候选仍需通过动态和已知自由空间检查，无法通过时继续地图路径搜索；普通返航候选若被优化压缩到不足 1 秒，也会拒绝并重试，避免重新引入极短轨迹。后备路径若直接抵达 home，也会强制零速度、零加速度终点，再重新检查动态和碰撞。有效末段不受周期和临近结束重规划触发，碰撞检查继续；跟踪位置误差超过 0.6 m 并持续 0.5 秒，或末段结束 3 秒后实际状态仍未收敛，会请求制动并从反馈状态重新规划。`SETTLING` 心跳表示安全末段仍在执行，避免将正常的减速、转向误判为无新轨迹超时。进入 `HOME_HOLD` 还要求偏航误差不超过 0.08 rad、偏航角速度不超过 0.10 rad/s，且参考轨迹已到达精确终点，因此切换悬停不会再突然修正参考位置或清零非零导数。

新增回归测试：`return_recovery.test` 验证超过三次失败、输出超时、制动后新轨迹接管和人工停止；`return_planner.test` 使用实际 FUEL 和连续有效点云，验证目标被遮挡时至少六次失败后仍规划，并在合成障碍清除后恢复同一返航任务。

## 验证与限制

```bash
src/fuel_bridge/scripts/fuel_env.sh rostest prometheus_fuel bridge.test
# 首次运行边界回归测试时，先构建测试可执行文件
bash compile_fuel.sh fuel_frontier_boundary
src/fuel_bridge/scripts/fuel_env.sh rostest prometheus_fuel frontier_boundary.test
src/fuel_bridge/scripts/fuel_env.sh rostest prometheus_fuel planner.test
```

接口测试使用合成 ROS 消息，验证旋转和平移外参、点过滤、时间戳配对、探索触发门控、轨迹字段与递增命令编号、轨迹中断悬停、边界框几何、越界轨迹转发、低高度与高速度启动、无效数值消息忽略及停止后不再转发。它不等同于 Gazebo/PX4 闭环飞行验收。

边界回归测试使用实际 SDF 地图和前沿搜索器，验证框内全部观测后不再追逐框外未知区域，以及框内未知区域被观测前后前沿的保留与移除；同时验证最大量程射线只清除有限范围自由空间、可见性从雷达安装位置计算。

规划链路测试启动实际 FUEL 算法和轨迹服务器，通过合成起飞扫描数据验证占据地图、前沿、B 样条及 Prometheus 指令输出，并检查前沿 `ADD` Marker 的点不为空、轨迹控制点在探索边界内。本分支已通过完整编译、launch 参数解析及上述 ROS 测试，并通过实际 Gazebo/PX4 短程起飞、探索与停止悬停检查。完整区域覆盖和长期行为尚未验收，详见 [重构记录](../../docs/REFACTOR.md)。

点云或定位超过 0.5 s 未更新、退出指令控制或轨迹中断时会停止探索并请求悬停，初次规划允许 10 s。控制器自身的保护逻辑仍生效。仿真建图话题保留无回波射线，但传感器盲区、遮挡及建图量程之外仍可能存在未知体素。FUEL 的完成状态表示没有可访问的前沿，不保证边界内所有体素都已观测。

上游适配修改涉及 NLopt 查找、并行构建消息依赖、MID360 视场与射线原点、空触发保护、空 Marker 清理、探索边界前沿判断、停止/重新启动，以及移除演示轨迹服务器启动时上下移动的动作。上游完整源码和许可证保留在 `third_party/FUEL`；FUEL 采用 GPL-3.0。
