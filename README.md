# P600 / MID360 / FUEL 自主探索仿真

本分支只维护 **ROS Noetic＋Gazebo 11＋P600＋MID360＋单机 FUEL**。定位来自 PX4/MAVROS 的 GPS/惯性融合，MID360 提供探索建图点云。

FUEL 完整源码直接纳入 [third_party/FUEL](third_party/FUEL)，包括上游规划、仿真器、示例、资源与许可证。构建只选择当前 P600 仿真需要的规划模块。来源版本及本地修复见 [FUEL 修改记录](third_party/FUEL/LOCAL_CHANGES.md)。

## 目录

```text
src/
  fuel -> ../third_party/FUEL/fuel_planner
  fuel_bridge/                 # 点云转换、探索触发、边界及停止保护
  uav_control/                 # 当前轨迹控制器、PX4 状态适配、RViz 模型
  messages/                    # 使用中的 Prometheus/PositionCommand 消息
  gazebo_simulator/            # P600/MID360/GPS、圆柱场及 indoor4、MAVROS 配置
  livox_laser_gazebo_plugins/   # 真实回波与无回波建图射线
third_party/FUEL/              # 完整上游源码，含已有修复
scripts/                      # 检查和测试入口
docs/                         # 排障与重构记录
compile_fuel.sh                # 构建全部本地组件
fuel_simulation.sh             # 启动完整仿真
source.sh                     # 加载同一工作空间及外部 PX4 路径
```

## 准备和编译

在 Ubuntu 20.04 / ROS Noetic / Gazebo 11 环境中安装依赖：

```bash
sudo apt-get update
sudo apt-get install ros-noetic-gazebo-ros-pkgs ros-noetic-mavros ros-noetic-mavros-extras \
  ros-noetic-pcl-ros ros-noetic-cv-bridge ros-noetic-rostest ros-noetic-rviz \
  libnlopt-dev libnlopt-cxx-dev libarmadillo-dev python3-jinja2 python3-numpy
```

PX4 使用原工程配套的 `prometheus_px4`，继续单独维护。已有可运行的 checkout 可以直接复用；本仓库不下载、修改或编译它。需要的产物是 `build/amovlab_sitl_default/bin/px4` 和 `build/amovlab_sitl_default/build_gazebo/` 中的飞行动力学/GPS 插件，固件须包含 `p600_mid360` 启动配置。

如果需要准备外部 PX4，可从 `https://gitee.com/amovlab/prometheus_px4.git` 获取，按其说明安装依赖、初始化子模块，并构建 `amovlab_sitl_default` 及 Gazebo 插件。当前验证所用版本见 [模型来源说明](src/gazebo_simulator/ASSET_SOURCES.md)。MAVROS 的 GeographicLib 数据也应按其安装说明准备。

从本工程根目录执行：

```bash
bash compile_fuel.sh
```

这一次构建包含 FUEL、桥接、控制器、消息和雷达插件，不需要先构建旧工程。默认两个编译任务，可用 `FUEL_BUILD_JOBS=4 bash compile_fuel.sh` 调整。

源码为 `src/`，产物为 `build/fuel/`、`devel/fuel/`。旧的 `build/fuel_ws` 和根目录旧 `devel/setup.bash` 不再使用。切换分支后重新运行上述脚本；所有源代码及模型数据均已入库，构建过程不访问 GitHub。

## 启动和探索

终端一：

```bash
# 默认 $HOME/prometheus_px4；其他位置在启动前指定。
export PX4_ROOT="$HOME/prometheus_px4"
# 默认 $PX4_ROOT/build/amovlab_sitl_default；必要时指定其他构建目录。
# export PX4_BUILD_DIR=/path/to/px4/build/amovlab_sitl_default
bash fuel_simulation.sh
```

切换到 XTDrone 的 `indoor4` 迷宫场景（先关闭已有仿真）：

```bash
bash fuel_simulation.sh scene:=indoor4
```

默认仍为圆柱场，也可使用 `scene:=cylinder`。`indoor4` 包含 150 面墙，场地约 68×54 m；出生点为 Gazebo `(0, -1)`，起飞指令仍使用局部坐标 `[0, 0, 1.5]`。入口自动设置 80×64×6 m 地图，局部探索边界为 x∈[-31,39]、y∈[-26,29]、z∈[0.5,2.0] m。墙高 2.5 m，探索目标高度上限设为 2 m。自定义出生点时边界会自动换算，若超出地图分配范围，还需扩大 `map_size_x/map_size_y`。

此场景已通过 launch 参数解析及静态资源检查；实际飞行验证因系统磁盘空间不足尚未完成。现有 `scripts/check_simulation.py` 使用圆柱场边界，仅适用于默认圆柱场。


启动后只有一个 RViz。`gui:=false` 关闭 Gazebo 窗口，`rviz:=false` 关闭 RViz；`spawn_x/spawn_y` 控制出生点并自动换算探索边界。默认探索圆柱场，局部范围 x∈[-1,22]、y∈[-11,10]、z∈[0.5,2.5] m。

等待 PX4 连接、定位有效、RViz 点云和地图出现，再等待至少 2 秒仿真时间，让连续设定值流建立。终端二，从工程根目录操作：

```bash
source source.sh
rosservice call /uav1/mavros/set_mode "{base_mode: 0, custom_mode: 'OFFBOARD'}"
rosservice call /uav1/mavros/cmd/arming "{value: true}"
rostopic echo -n 1 /uav1/mavros/state
```

服务返回 `mode_sent: True` 只表示请求已发送；若实际模式尚未切换，等待后重试。确认实际状态为 `OFFBOARD`、`armed: True`，再发送局部坐标中出生点上方 1.5 m 的目标：

```bash
rostopic pub -1 /uav1/prometheus/command prometheus_msgs/UAVCommand \
  "{Agent_CMD: 4, Move_mode: 0, position_ref: [0.0, 0.0, 1.5], yaw_ref: 0.0, Command_ID: 1}"
```

如果已经用其他模式起飞，切入 OFFBOARD 会锁定切换时的当前位置和朝向；无需再次发送上述固定高度目标。位置和轨迹 `Move` 指令只在解锁且进入 OFFBOARD 后接受，不会预存其他模式下的旧目标。

稳定悬停、日志显示 `WAIT_TRIGGER` 后，在 RViz **2D Nav Goal** 按住左键拖出箭头并松开，即可开始自主探索。点击位置仅作启动信号，后续目标由 FUEL 选择。

停止探索并锁定当前位置悬停：

```bash
rosservice call /fuel/stop "{}"
```

需要降落时，先停止探索，再发送降落命令：

```bash
rostopic pub -1 /uav1/prometheus/command prometheus_msgs/UAVCommand "{Agent_CMD: 3}"
```

探索完成后维持最终有效轨迹段的停止点，不自动返航或降落。再次探索前先调用 `/fuel/stop` 复位，然后重新点击 **2D Nav Goal**。

## 控制与参数

控制链为 **FUEL 位置/速度/加速度 → 位置和速度反馈＋加速度前馈 → 姿态/推力 → MAVROS → PX4**。保留原 P600 增益、反馈限幅和推力估计，仅保留模式 9 的轨迹跟踪、模式 0 的定点，以及悬停和降落。参数在 [p600.yaml](src/uav_control/config/p600.yaml)。

[bridge.yaml](src/fuel_bridge/config/bridge.yaml) 管理雷达外参和超时，[mid360.yaml](src/fuel_bridge/config/mid360.yaml) 管理地图、视场和规划；探索边界由 [simulation.launch](src/fuel_bridge/launch/simulation.launch) 设置。控制器仅在解锁、OFFBOARD 且定位有效时报告 `COMMAND_CONTROL`。

详细接口、建图射线和边界参数见 [FUEL 使用说明](src/fuel_bridge/README.md)。此前两套 RViz、空 Marker、模型加载、坐标系与边界绕圈等问题见 [排障记录](docs/TROUBLESHOOTING.md)，本次删减和验证结果见 [重构记录](docs/REFACTOR.md)。

## 测试

```bash
bash scripts/test_fuel.sh
```

包括控制器/状态适配、桥接、地图边界和实际 FUEL 规划链路的隔离 ROS 测试。它们使用合成输入，不替代 Gazebo/PX4 飞行动力学验证。

关闭已有仿真后，可运行隔离的无界面短程飞行检查（会自动解锁、起飞、探索并停止，只控制本次启动的仿真实例）：

```bash
src/fuel_bridge/scripts/fuel_env.sh python3 scripts/check_simulation.py
```

它验证真实插件的回波/无回波射线、起飞、RViz 话题触发和停止悬停，不代表圆柱场整场探索覆盖验收。检查脚本检测到已有 PX4/Gazebo 时会退出。
