# P600 + MID360 + FUEL 多会话探索仿真启动手册

本文按 **Ubuntu 20.04、ROS Noetic、Gazebo 11、PX4 SITL** 编写，说明在 `indoor4` 场景下启动仿真、进行首次探索、停止并手动降落、保存地图，以及重新启动后加载地图。所有飞行操作均针对 Gazebo/PX4 SITL 仿真，不要将本文命令直接用于真实无人机。

> **工程目录**：`~/LTAOM（2）/fuelgazebo-master`。`fuel_simulation.sh` 位于该目录，不在 `~/LTAOM（2）` 上一级目录。

## 0. 首次使用前检查

### PX4 SITL

默认 PX4 工程位置为 `~/prometheus_px4`。编译后应存在：

```text
~/prometheus_px4/build/amovlab_sitl_default/bin/px4
~/prometheus_px4/build/amovlab_sitl_default/build_gazebo/libgazebo_mavlink_interface.so
```

配置、依赖和 PX4 编译步骤见 [prometheus_px4 配置教程](https://wiki.amovlab.com/public/prometheus-wiki/快速上手/Prometheus仿真环境配置_Ubuntu/prometheus_px4配置.html#prometheus_px4配置)。如果 PX4 安装在其他目录，在启动 FUEL 的终端中设置实际路径：

```bash
export PX4_ROOT="$HOME/你的PX4目录/prometheus_px4"
# 只有 PX4 build 目录也不是默认目录时才需要设置：
# export PX4_BUILD_DIR="$PX4_ROOT/build/amovlab_sitl_default"
```

### 编译本工程

确认 ROS Noetic 已安装并在当前 shell 中可用。首次编译前可安装工程依赖（已安装的包会自动跳过）：

```bash
sudo apt-get update
sudo apt-get install ros-noetic-pcl-ros ros-noetic-cv-bridge \
  libnlopt-dev libnlopt-cxx-dev libarmadillo-dev \
  python3-jinja2 python3-numpy
```

首次使用或修改代码后，在工程根目录编译：

```bash
cd "$HOME/LTAOM（2）/fuelgazebo-master"
bash compile_fuel.sh
```

如果编译报告依赖缺失，按工程及 ROS Noetic 的依赖提示安装后重试。仿真启动脚本会检查必要的构建产物和 PX4 文件；检查失败时先完成编译或修正 `PX4_ROOT`。

## 1. 第一次飞行：创建并保存地图

### 终端 1：启动 Gazebo、PX4、控制器和 FUEL

```bash
cd "$HOME/LTAOM（2）/fuelgazebo-master"
bash fuel_simulation.sh scene:=indoor4 session:=save
```

脚本会启动 `indoor4` 仿真、PX4 SITL、控制器、FUEL 和 RViz。不要再额外传入 `odom_topic:=/ltaom/odom_aligned`：当前推荐的飞控与 FUEL 闭环使用 PX4 本地位置/里程计。

常用场景：

```bash
# 室内迷宫
bash fuel_simulation.sh scene:=indoor4 session:=save

# 圆柱场
bash fuel_simulation.sh scene:=cylinder session:=save
```

### 终端 2（可选）：运行 LTAOM/FAST-LIO 先验建图

如果本次也要记录 LTAOM 点云先验，在另一终端执行：

```bash
cd "$HOME/LTAOM（2）/fuelgazebo-master"
source source.sh
roslaunch prometheus_fuel ltaom_multisession.launch session:=save
```

这个 launch 会启动 FAST-LIO、STD 描述子/平面提取和 LTAOM 位姿图优化。它是独立的 SLAM 流水线；不要因为它已启动，就假定 FUEL 的控制和栅格地图已经切换到 LTAOM 坐标系。

### 解锁、起飞并开始探索

1. 等待 Gazebo、PX4、MAVROS、控制器和 FUEL 均正常启动；在 QGroundControl 中确认连接状态正常。
2. 在 QGroundControl 中解锁，手动起飞到约 **1.2 m**，稳定悬停后切换到 **OFFBOARD**。切换模式前确认周围有足够空间。
3. 确认 FUEL 已进入等待触发状态。在工程终端执行以下命令可检查飞控和控制器状态：

   ```bash
   cd "$HOME/LTAOM（2）/fuelgazebo-master"
   source source.sh
   rostopic echo -n 1 /mavros/state
   rostopic echo -n 1 /uav_controller/fsm_status
   ```

   检查 MAVROS 状态为 `armed: True`、`mode: "OFFBOARD"`，控制器状态为 `phase: "HOLD"` 且 `output_policy: "CLOSED_LOOP"`。
4. 在 RViz 点击 **2D Nav Goal**，在地图上按住左键拖出方向箭头后松开。该点击用于启动探索；后续 frontier 和探索目标由 FUEL 自主选择。

### 停止探索、自动返航并降落

手动停止探索会被解释为立即返航：无人机规划经已探索通道返回本地 ENU `(0, 0, 1)`，而不是原地悬停或降落。

```bash
cd "$HOME/LTAOM（2）/fuelgazebo-master"
source source.sh
rosservice call /fuel/stop "{}"
```

`/fuel/return_home` 与此等价（幂等）。低电量、探索完成或超时保护也会触发同一返航。无人机到家稳定后进入 `HOME_HOLD` 并触发地图与已运行的 LTAOM 先验保存。

返航完成并确认保存后，再通过 QGroundControl 执行 **AUTO.LAND** 降落；也可在终端请求：

```bash
rosservice call /mavros/set_mode "{base_mode: 0, custom_mode: 'AUTO.LAND'}"
```

无论使用何种降落方式，都应等待日志出现类似 `Multi-session maps saved` 或 `Landing detected: exploration stopped and session maps saved`，再检查文件。任务管理器也会在检测到落地并持续确认 1 秒后触发保存，但不要在保存完成前关闭 ROS 节点。

### 核对保存文件

在终端执行：

```bash
find "$HOME/fuel_sim_logs" -type f -size +0c -printf '%p (%s bytes)\n' | sort
```

至少应检查 FUEL 栅格地图：

```text
~/fuel_sim_logs/map_prior/occ_map.bin
```

如果终端 2 的 LTAOM 流水线正在运行并已完成建图，还应检查以下先验文件：

```text
~/fuel_sim_logs/ltaom/map_prior/clouds_corrected.pcd
~/fuel_sim_logs/ltaom/map_prior/descriptors.pcd
~/fuel_sim_logs/ltaom/map_prior/plane_all.pcd
```

确认本次需要的文件存在且大小大于 0 后，才能关闭仿真和 LTAOM 终端。若没有运行 LTAOM，三个 LTAOM 先验文件不会生成；FUEL 栅格图保存不依赖 LTAOM。

## 2. 第二次飞行：加载地图并继续探索

### 先关闭上次仿真，再启动本次仿真

确认上次保存完成后，在终端 1 关闭仿真及相关节点。随后使用相同场景、地图配置和启动位置运行：

```bash
cd "$HOME/LTAOM（2）/fuelgazebo-master"
bash fuel_simulation.sh scene:=indoor4 session:=load
```

不要删除或覆盖 `~/fuel_sim_logs/map_prior/occ_map.bin`。启动时 FUEL 会从该文件加载已保存的 occupancy grid，并在本次任务结束时继续更新保存。

如果第一次飞行也运行了 LTAOM，在终端 2 使用相同的 session：

```bash
cd "$HOME/LTAOM（2）/fuelgazebo-master"
source source.sh
roslaunch prometheus_fuel ltaom_multisession.launch session:=load
```

继续执行前述 QGroundControl 起飞/OFFBOARD 检查和 RViz **2D Nav Goal** 触发步骤。不要在两次运行之间更换场景、地图大小、地图分辨率或 spawn 设置；否则栅格地图可能无法加载（例如出现 `map dims mismatch`），即使文件存在也不能视为成功续飞。

## 3. session 模式说明

| 参数 | 用途 |
|---|---|
| `session:=fresh` | 单次运行，不加载或保存多会话地图。 |
| `session:=save` | 首次运行，不加载旧 FUEL 栅格图；任务停止/返航完成/落地时保存。 |
| `session:=load` | 加载已有 FUEL 栅格图，并在任务结束时更新保存。 |

`session` 默认值是 `fresh`。两次飞行必须分别使用 `save` 和 `load`，并保持场景与地图配置一致。

## 4. 当前多会话能力边界

- `session:=load` 会让 FUEL 加载先前的 occupancy grid；这不等于已完成跨飞行的全局定位。当前推荐控制与规划里程计仍为 PX4 `/mavros/local_position/odom`，所以二次启动需要保证场景、spawn 和 PX4 local 原点可复现，否则旧栅格与当前飞行坐标可能错位。
- LTAOM/STD 的点云、描述子和面片先验可独立保存/加载，但当前启动说明不把 LTAOM 重定位结果宣称为已可靠地注入 FUEL 的地图坐标系。不要仅凭 `ltaom_multisession.launch session:=load` 已启动，就认定飞行器已经对齐到首飞地图坐标系并能无缝从中断处继续。
- 因此，现阶段可验证的行为是“重用同一仿真坐标条件下的 FUEL 已探索栅格，并独立保存/加载 LTAOM 先验”；严格的异地重启后全局重定位、坐标变换与可靠续飞仍需要端到端验证。

## 5. 常见问题

### `bash: fuel_simulation.sh: 没有那个文件或目录`

脚本在工程子目录中。先进入正确目录，再启动：

```bash
cd "$HOME/LTAOM（2）/fuelgazebo-master"
bash fuel_simulation.sh scene:=indoor4 session:=save
```

### 提示 PX4 SITL 或构建产物缺失

确认已编译本工程和 PX4；若 PX4 不在 `~/prometheus_px4`，在启动仿真的同一终端先设置 `PX4_ROOT`，再运行脚本。

### `map dims mismatch` 或加载后地图位置不对

检查 session 文件是否来自同一场景/相同地图参数、是否使用相同 spawn；不要在两次飞行间切换 `indoor4` 和 `cylinder`。注意栅格图加载成功也不能替代坐标系对齐。

### 无法启动探索或模式切换后位置不稳

先确认 `/mavros/state`、`/uav_controller/fsm_status`、传感器数据和控制器输出均正常；不要为了绕过问题而直接切换到未验证的 `/ltaom/odom_aligned`。更多 FUEL 参数和任务行为见 [FUEL 使用说明](src/fuel_bridge/README.md)。
