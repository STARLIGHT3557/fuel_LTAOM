# 跨架次地图复用（FUEL 栅格地图保存与加载）

本文说明如何在本工程（MID360 + Faster-LIO + PX4 实机）中启用**地图复用**：第一架次把 FUEL 已探索的 occupancy grid 存盘，第二架次启动时直接加载该地图，飞行器不必把已探索区域重新跑一遍。

本功能从 `LTAOM（2）/fuelgazebo-master` 的 `saveMap` / `loadMap` 移植而来，**只增加地图保存与加载，不改变 FUEL 的探索算法、也不改变现有定位链路**（仍是 Faster-LIO + `ekf` + MAVROS）。

---

## 0. 功能与边界

### 做了什么

| 能力 | 说明 |
| --- | --- |
| 保存栅格图 | 任务结束（开始返航 / 手动停止 / 确认落地）时把整张 occupancy grid 写入 `occ_map.bin` |
| 加载栅格图 | `session:=load` 时在规划器启动阶段恢复该栅格，前沿检测从“已探索边界”继续 |
| 参数校验 | 尺寸、分辨率、原点任一项不匹配就拒绝加载并打印明确原因，不会把障碍物放到错误坐标 |
| 手动保存 | 服务或话题随时触发一次保存，不中断飞行 |

### 没做什么

- 不引入 LTAOM 的 STD 描述子／位姿图先验。那部分依赖 LTAOM 改版 FAST-LIO 的 `/aft_mapped_to_init`、`/odom_correction_info`、`multisession_mode` 等专属话题，本工程的 Faster-LIO 不产生这些数据。
- 因此**本功能不提供跨架次的全局重定位**。它复用的是“同一坐标系下的已探索栅格”，前提见第 7 节。

---

## 1. 改动清单

| 文件 | 改动 |
| --- | --- |
| `third_party/FUEL/fuel_planner/plan_env/include/plan_env/sdf_map.h` | 新增 `saveMap()` / `loadMap()` / `knownVoxelCount()` / `voxelStats()` / `resetMap()` |
| `third_party/FUEL/fuel_planner/plan_env/src/sdf_map.cpp` | 二进制栅格读写、几何参数校验、加载后重建膨胀层与 ESDF、内存清空 |
| `third_party/FUEL/fuel_planner/plan_env/include/plan_env/map_ros.h` | 新增保存话题订阅、`save_map` 服务、`compute_esdf` 开关 |
| `third_party/FUEL/fuel_planner/plan_env/src/map_ros.cpp` | 读取 `map_ros/map_load_path`、`map_ros/map_save_path`，实现保存触发 |
| `src/fuel_bridge/src/mission_manager_node.cpp` | `multisession/session` 参数、任务结束保存触发、`/fuel/save_map` 服务 |
| `src/fuel_bridge/src/map_reuse_node.cpp` | **新增**：只托管栅格地图的节点（无规划器、无探索） |
| `src/fuel_bridge/launch/map_reuse.launch` | **新增**：手持测试入口（雷达点云 → 栅格 → RViz） |
| `src/fuel_bridge/config/map_reuse.yaml` | **新增**：手持测试的建图参数 |
| `src/fuel_bridge/config/map_reuse.rviz` | **新增**：手持测试的 RViz 布局 |
| `handheld_map.sh` | **新增**：手持测试启动脚本 |
| `src/fuel_bridge/launch/fuel.launch` | `session` / `map_dir` 参数，换算加载与保存路径 |
| `src/fuel_bridge/launch/realflight.launch` | 透传 `session` / `map_dir` |

> 说明：这些改动只新增分支，`session:=fresh`（默认）时加载路径与保存路径都是空字符串，行为与改动前完全一致。

---

## 2. 依赖与编译

依赖与原有实机流程相同，另需 `std_srvs`（`ros-noetic-rostest` 已包含）。首次或改动后编译：

```bash
cd ~/uav_explor_LTAOM
bash compile_fuel.sh
```

编译成功后关键产物：

```text
devel/fuel/lib/libplan_env.so
devel/fuel/lib/prometheus_fuel/mission_manager_node
```

---

## 3. 第一架次：探索并保存地图

### 3.1 启动

```bash
cd ~/uav_explor_LTAOM
bash start_realflight.sh session:=save
```

`start_realflight.sh` 依次启动 MAVROS、MID360／Faster-LIO、EKF，并把 `session:=save` 透传给 `fuel_realflight.sh` → `realflight.launch` → `fuel.launch`。

启动后确认日志出现（`exploration_node`）：

```text
MapROS: session map reuse enabled, save path /home/<user>/fuel_sessions/occ_map.bin (topic /save_occ_map, service save_map)
```

### 3.2 起飞与探索

与原有流程完全一致：

1. 接通雷达网线，电脑网口配置 `192.168.1.50/24`，雷达 `192.168.1.199`。
2. QGroundControl 解锁，手动起飞到约 1.2 m，稳定悬停后切换 **OFFBOARD**。
3. 确认状态：

   ```bash
   cd ~/uav_explor_LTAOM
   source source.sh
   rostopic echo -n 1 /mavros/state           # armed: True, mode: OFFBOARD
   rostopic echo -n 1 /uav_controller/fsm_status   # phase: HOLD, output_policy: CLOSED_LOOP
   ```

4. 在 RViz 用 **2D Nav Goal** 触发探索，或 `rosservice call /fuel/start "{}"`。

### 3.3 结束任务时自动保存

以下任一事件都会触发一次保存（`session:=save`/`load` 时）：

| 触发 | 日志 |
| --- | --- |
| 开始返航（探索完成、低电量、超时） | `[MAP] save requested (exploration ended / returning home)` |
| 手动 `/fuel/stop` | `[MAP] save requested (mission stopped)` |
| 落地并持续确认 `ground_confirm_time` | `[MAP] save requested (landing confirmed)` |

保存执行在规划器进程，成功时打印：

```text
SDFMap: saved occupancy grid to /home/<user>/fuel_sessions/occ_map.bin (1152000 voxels, 18234 observed)
```

如果地图从未观测到任何体素，会拒绝写盘并保留原有先验：

```text
SDFMap: map has no observed voxels, keeping the existing prior at ...
```

### 3.4 核对保存结果

```bash
ls -l ~/fuel_sessions/occ_map.bin
```

文件大小 ≈ `体素数 × 8 字节`：

| 地图尺寸（`realflight.launch`） | 体素数 | 文件大小 |
| --- | --- | --- |
| 60 × 50 × 10 m @ 0.2 m（默认） | 3,750,000 | ≈ 30 MB |
| 48 × 32 × 6 m @ 0.2 m（`fuel.launch` 默认） | 1,152,000 | ≈ 9 MB |

确认大小大于 0 且与上表量级一致，再关闭节点。

### 3.5 手动补充保存（可选）

任务没结束也可以随时落盘，不会中断飞行：

```bash
rosservice call /fuel/save_map "{}"
# 或直接触发规划器侧保存：
rosservice call /exploration_node/save_map "{}"
rostopic pub -1 /save_occ_map std_msgs/Empty "{}"
```

---

## 4. 第二架次：加载地图继续探索

### 4.1 前提

**必须使用与第一架次完全相同的地图配置**，否则加载会被拒绝：

- `map_size_x` / `map_size_y` / `map_size_z`
- `sdf_map/resolution`、`sdf_map/ground_height`（决定地图原点）
- 地图尺寸参数不在 YAML 里，而在 launch 参数里，见下表

| 参数 | `realflight.launch` 默认 | `fuel.launch` 默认 |
| --- | --- | --- |
| `map_size_x` / `map_size_y` / `map_size_z` | 60.0 / 50.0 / 10.0 | 48.0 / 32.0 / 6.0 |

只要两次启动用同一个入口脚本且不改这几个参数，就满足条件。

### 4.2 启动

```bash
cd ~/uav_explor_LTAOM
bash start_realflight.sh session:=load
```

加载成功时日志打印：

```text
SDFMap: loaded occupancy grid from /home/<user>/fuel_sessions/occ_map.bin (3750000 voxels, 18234 observed, saved inflation 0.60 m, active 0.60 m)
```

文件不存在时不会阻塞启动，只是从空图开始：

```text
SDFMap: no prior map at /home/<user>/fuel_sessions/occ_map.bin, starting with an empty map
```

参数不一致时会明确拒绝：

```text
SDFMap: prior map dims 240x160x30 (1152000 voxels) do not match current 300x250x50 (3750000 voxels); ignoring ...
SDFMap: prior map resolution 0.300 does not match current 0.200; ignoring ...
SDFMap: prior map origin [-30.00 -25.00 -1.00] does not match current [-24.00 -16.00 -1.00]; ignoring ...
```

### 4.3 起飞与继续探索

与第一架次相同。加载后 FUEL 的未知区域只剩真正没看过的地方，因此 **2D Nav Goal 触发后规划器会直接向未探索区域推进**，已探索区域不会重复覆盖。

任务结束时按同一套规则更新保存（`session:=load` 也会保存）。

---

## 5. 手持静态测试（不飞无人机）

只想验证地图复用本身是否好用、不需要飞控和探索时，用这个模式：**只启动雷达 + Faster-LIO + 栅格建图 + RViz**，位姿来自 LIO 自身，人拿着无人机走动即可。

### 5.1 与飞行模式的区别

| | 飞行模式 | 手持测试 |
| --- | --- | --- |
| 启动脚本 | `start_realflight.sh` | `handheld_map.sh` |
| MAVROS | 需要 | **不需要** |
| 飞控 / 解锁 / OFFBOARD | 需要 | **不需要** |
| 控制器 `uav_controller` | 需要 | **不需要** |
| 探索规划（前沿、轨迹、FSM） | 需要 | **不需要** |
| 位姿来源 | PX4 `/mavros/local_position/odom` | LIO `/laserMapping/odometry` |
| 栅格地图保存/加载 | 同一套代码 | 同一套代码 |

点云转世界系用的仍然是飞行时的 `fuel_bridge`，所以这张图的生成方式与飞行时完全一致，只有位姿来源不同。

### 5.2 第一圈：走一圈并保存地图

```bash
cd ~/uav_explor_LTAOM
bash handheld_map.sh session:=save
```

会依次启动 MID360 驱动、Faster-LIO、`fuel_bridge`、`map_reuse_node` 和 RViz。

RViz 中应看到：

- **Occupancy grid**（`/sdf_map/occupancy_all`）随走动逐格填充；
- **Raw hits (world)**（`/fuel/cloud_hits_world`）是当前实测点；
- **LIO path**（`/laserMapping/path`）是走过的轨迹。

终端每 5 秒打印一次进度：

```text
[MAP] observed 23706/1600000 voxels (1.48%), occupied 2585
```

走完一圈回到出发点后保存：

```bash
cd ~/uav_explor_LTAOM
source source.sh
rosservice call /map_reuse/save_map "{}"
```

返回 `success: True` 即已写盘：

```text
message: "occupancy map saved to /home/<user>/fuel_sessions/occ_map.bin"
```

然后 `Ctrl+C` 结束本圈全部进程。

### 5.3 第二圈：加载地图继续走

```bash
cd ~/uav_explor_LTAOM
bash handheld_map.sh session:=load
```

启动时日志应出现：

```text
SDFMap: loaded occupancy grid from /home/<user>/fuel_sessions/occ_map.bin (1600000 voxels, 23706 observed, saved inflation 0.30 m, active 0.30 m)
```

**在雷达还没开始扫描时，RViz 里就已经能看到上一圈的地图**——这是判断加载成功最直接的方法。随后继续走动，新区域叠加到旧图上；结束时同样执行 `rosservice call /map_reuse/save_map "{}"`，文件会被更新。

第三圈、第四圈……都重复 `session:=load`。

### 5.4 关键前提：每圈起点必须一致

栅格原点是 `(-map_size_x/2, -map_size_y/2, ground_height)`，锚定在 **LIO 启动时的位姿**上。LIO 每次上电都把自己的起始位姿定为原点，所以：

- 每圈必须让无人机停在**同一位置**、朝**同一朝向**再启动 `handheld_map.sh`；
- 地面高度应基本一致（手持时把无人机放在同一高度或同一张桌上）。

否则旧图会整体平移/旋转错位，表现为“地图和实际环境对不上”。这不是算法错误，而是没有全局重定位的必然限制。

### 5.5 常用操作

| 操作 | 命令 |
| --- | --- |
| 保存地图 | `rosservice call /map_reuse/save_map "{}"` |
| 清空当前内存中的地图（不影响已保存文件） | `rosservice call /map_reuse/clear "{}"` |
| 看进度 | 终端每 5 秒自动打印 `[MAP] observed ...` |
| 换地图目录 | `bash handheld_map.sh session:=save map_dir:=$HOME/maps/lab1` |
| 不开 RViz | `bash handheld_map.sh session:=save rviz:=false` |
| 调整探测距离等建图参数 | 编辑 `src/fuel_bridge/config/map_reuse.yaml`，或用 `mapping_config:=` 指向自己的 yaml |

建图参数集中在 [map_reuse.yaml](../../src/fuel_bridge/config/map_reuse.yaml)，常用项：

```yaml
sdf_map:
  resolution: 0.2        # 0.1 更精细但更耗内存/CPU
  map_size_x: 40.0       # 场地更大就调大（注意旧地图将无法加载）
  map_size_y: 40.0
  map_size_z: 8.0
  ground_height: -2.0    # 决定栅格原点的 z
  max_ray_length: 15.0   # 雷达建图截断距离
  obstacles_inflation: 0.3
```

想在不改仓库文件的前提下另存一套参数，复制一份再传进去：

```bash
cp src/fuel_bridge/config/map_reuse.yaml ~/my_map.yaml
# 编辑 ~/my_map.yaml
bash handheld_map.sh session:=fresh mapping_config:=$HOME/my_map.yaml
```

地图尺寸、分辨率、`ground_height` 一旦改变，之前保存的 `occ_map.bin` 会因校验不通过而被拒绝加载。

### 5.6 手持测试的性能说明

- 手持模式默认 `map_ros/compute_esdf: false`，跳过规划才需要的 ESDF 计算，明显降低 CPU 占用。
- 地图 `40 × 40 × 8 m @ 0.2 m` = 160 万体素，`occ_map.bin` 约 12.8 MB；保存耗时约 0.1–0.3 秒，期间会丢几帧点云，属正常现象，不要频繁触发。
- 若雷达数据量大导致卡顿，可降低 `point_filter_num`（在 `mid360.yaml` 中，默认 3）或增大 `sdf_map/resolution`。

---

## 6. 参数一览

| 参数 | 默认 | 作用 |
| --- | --- | --- |
| `session` | `fresh` | `fresh` 不载入不保存；`save` 只在结束时保存；`load` 先载入、结束时更新保存 |
| `map_dir` | `$(env HOME)/fuel_sessions` | 地图目录，文件名固定 `occ_map.bin` |
| `map_save_topic` | `/save_occ_map` | 保存触发话题（`std_msgs/Empty`） |
| `save_on_return` | `true` | 开始返航时保存（仅飞行模式） |
| `save_on_stop` | `true` | `/fuel/stop` 时保存（仅飞行模式） |
| `save_on_landing` | `true` | 落地确认后保存（仅飞行模式） |
| `ground_confirm_time` | `1.5` | 落地判定需要持续的秒数（仅飞行模式） |

参数可写在 `src/fuel_bridge/launch/fuel.launch`（飞行）或 `config/map_reuse.yaml`（手持）中固化，也可在命令行覆盖：

```bash
bash start_realflight.sh session:=save save_on_landing:=false
bash handheld_map.sh session:=save sdf_map/max_ray_length:=20.0   # 手持模式用 sdf_map/... 覆盖 yaml
```

相关服务：

| 服务 | 模式 | 作用 |
| --- | --- | --- |
| `/fuel/save_map` | 飞行 | 请求保存一次（任务管理器转发到 `/save_occ_map`） |
| `/exploration_node/save_map` | 飞行 | 直接让规划器保存，返回是否成功与路径 |
| `/map_reuse/save_map` | 手持 | 保存栅格并返回路径 |
| `/map_reuse/clear` | 手持 | 清空内存中的地图（不动已保存文件） |

---

## 7. 重要约束

1. **坐标系必须可复现。** 飞行模式用的是 MAVROS 局部 ENU（`/mavros/local_position/odom`），手持模式用的是 LIO 原点；两者都必须保证第二次启动的原点与朝向与第一次一致。原点漂移会让整张旧图整体错位，规划器看到的是“错位的墙”。手持模式的具体要求见 5.4。
2. **地图尺寸、分辨率、地面高度不能变。** 校验会拦截，但拦截后是从空图开始，等于没有复用。
3. **不要删除 `occ_map.bin` 之前重启探索流程。** `session:=fresh` 不写盘，但 `session:=load` 的文件缺失只会给出警告。
4. **保存发生在任务结束点**，不是实时增量落盘。飞行中意外断电会丢失本架次新增的探索结果；手持模式请走完一圈后主动调用 `/map_reuse/save_map`。
5. **膨胀层不从磁盘读取。** 加载时按当前 `sdf_map/obstacles_inflation` 重新生成，所以调整该参数后旧图仍可用。
6. **ESDF 在加载时重建一次**，启动大约多花 0.5–2 秒（3.75 M 体素），属正常现象。手持模式其后不再计算 ESDF。

---

## 8. 排查

| 现象 | 处理 |
| --- | --- |
| 日志 `session map reuse disabled` | `session:=fresh`，或 `map_save_path` 为空。检查是否把参数传给了正确的 launch |
| `no prior map at ...` | 首次使用属正常；否则检查 `map_dir` 与文件是否存在 |
| `dims/resolution/origin do not match` | 两次启动的地图尺寸、`resolution`、`ground_height` 不一致，恢复相同参数 |
| 加载成功但探索仍从原点开始扫 | 定位原点/航向与上一架次不同，旧图错位；改用可达的重定位方式或退回 `session:=save` 重新建图 |
| 结束任务后没有 `saved occupancy grid` | 检查 `[MAP] save requested` 是否出现；确认 `save_on_*` 未被关闭；确认地图确有观测体素 |
| `cannot create directory` / `cannot open ... for writing` | 目标目录无写权限，改 `map_dir:=` 到可写路径 |
| 文件大小明显偏小 | 保存时地图几乎未探索，或地图尺寸比预期小；先用 `session:=save` 正常飞一次 |

### 手持测试专项

| 现象 | 处理 |
| --- | --- |
| RViz 里 Occupancy grid 一直空 | 先确认 `rostopic hz /fuel/cloud_world` 有 10 Hz；没有则是雷达/LIO 未起来（看 `handheld_map.sh` 终端） |
| 加载成功但地图整体偏一边 | 本圈起点位置/朝向与上一圈不同，见 5.4；用 `/map_reuse/clear` 清空后重新走 |
| `FUEL frame mismatch` | LIO 输出的帧名不是 `imu`/`world`。检查 `mid360.yaml` 的 `publish/tf_imu_frame`，或用 `cloud_frame:=` 覆盖 |
| 保存时明显卡顿 | 正常，整图写盘 12.8 MB 约 0.1–0.3 秒；不要连续触发 |
| 想确认加载的是旧图 | 雷达未扫描时 RViz 已有内容，或看 `observed` 计数是否等于上次保存值 |

---

## 9. 与仿真工程的行为差异

`LTAOM（2）/fuelgazebo-master` 的 `session` 语义完全一致，但有两处不同：

1. 本工程没有 LTAOM 先验（描述子/平面）的保存与加载，只有栅格图。
2. 本工程 `/fuel/stop` 是**原地悬停**而不是立即返航，保存触发因此挂在“停止”而不是“开始返航”上；返航路径的保存由 `save_on_return` 负责。
