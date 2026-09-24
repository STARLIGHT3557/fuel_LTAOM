# P600＋MID360＋FUEL 自主探索仿真

环境：**Ubuntu 20.04、ROS Noetic、Gazebo 11**。按下面顺序完成 PX4 配置、本工程编译和探索启动。

## 1. 配置 prometheus_px4（首次使用,已有可运行的 `prometheus_px4` 可直接复用）

按照官方教程完成下载、环境安装、子模块初始化和编译：

**[prometheus_px4 配置教程](https://wiki.amovlab.com/public/prometheus-wiki/快速上手/Prometheus仿真环境配置_Ubuntu/prometheus_px4配置.html#prometheus_px4配置)**

教程使用 `make amovlab_sitl_default gazebo_p450` 验证 PX4 环境。验证完成后关闭该仿真；本工程会自行启动 P600。

默认 PX4  `~/prometheus_px4`，编译后应有：

```text
~/prometheus_px4/build/amovlab_sitl_default/bin/px4
~/prometheus_px4/build/amovlab_sitl_default/build_gazebo/libgazebo_mavlink_interface.so
```

已有可运行的 `prometheus_px4` 可直接复用。如果安装在其他位置，在启动 FUEL 的终端中设置：

```bash
export PX4_ROOT=/你的路径/prometheus_px4
# 仅在构建目录也不同时设置：
# export PX4_BUILD_DIR="$PX4_ROOT/build/amovlab_sitl_default"
```

## 2. 配置并编译 FUEL（首次使用或更新代码后）

在已安装 ROS Noetic 的环境中安装本工程依赖：

```bash
sudo apt-get update
sudo apt-get install ros-noetic-pcl-ros ros-noetic-cv-bridge libnlopt-dev libnlopt-cxx-dev libarmadillo-dev python3-jinja2 python3-numpy

```

进入本工程根目录（包含 `compile_fuel.sh` 的目录），执行：

```bash
bash compile_fuel.sh
```

FUEL 源码已包含在仓库中；该脚本统一编译 FUEL、控制器、桥接节点和雷达插件。

## 3. 启动仿真并开始探索


### 3.1 终端一：进入本工程根目录，启动 indoor4 迷宫场景：

```bash
bash fuel_simulation.sh scene:=indoor4
```

若使用圆柱场，改为 `bash fuel_simulation.sh scene:=cylinder`。两个命令选择一个运行；不传 `scene` 时默认圆柱场。启动脚本自动加载环境，并打开 Gazebo 和一个 RViz。


### 3.2 起飞并切换 OFFBOARD

在 QGroundControl 中解锁，手动起飞到约 1.2 m 并稳定悬停，然后切换到 OFFBOARD。新控制器会锁定切换时的位置。

终端中可确认实际状态：

```bash
source source.sh
rostopic echo -n 1 /uav1/mavros/state
rostopic echo -n 1 /uav_controller/fsm_status
```

确认 MAVROS 为 `mode: "OFFBOARD"`、`armed: True`，控制器为 `phase: "HOLD"` 且 `output_policy: "CLOSED_LOOP"`。

### 3.3 开始探索：稳定悬停、日志显示 `WAIT_TRIGGER` 后，点击 RViz 的 **2D Nav Goal**，在地图上按住左键拖出箭头并松开。点击位置只作启动信号，探索目标由 FUEL 自主选择。

**停止并悬停：**在终端二执行：

```bash
rosservice call /fuel/stop "{}"
```

**降落：**先停止探索，再执行：

```bash
rosservice call /uav1/mavros/set_mode "{base_mode: 0, custom_mode: 'AUTO.LAND'}"
```

探索完成后停在最后的轨迹停止点，不自动返航或降落。

详细参数见 [FUEL 使用说明](src/fuel_bridge/README.md)，常见问题见 [排障记录](docs/TROUBLESHOOTING.md)。
