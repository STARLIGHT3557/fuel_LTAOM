# FUEL 实地飞行与手持静态测试指南

本文按实际操作顺序介绍两种测试：

1. **实地飞行**：无人机正常起飞并执行探索，返航后由操作者降落；确认落地后自动保存地图。
2. **手持静态测试**：不解锁、不起飞，只拿着无人机移动，验证建图、PCD 保存和跨次加载融合。

FUEL 的栅格地图与 FUEL 点云使用同一份 `session` 流程。第一次运行用 `session:=save` 创建地图；之后用 `session:=load` 读取旧地图并继续累积。输出默认位于 `~/fuel_sim_logs/map_prior/`：

| 文件 | 内容 |
| --- | --- |
| `occ_map.bin` | FUEL 占据栅格地图，供后续规划和建图续用 |
| `merged_map.pcd` | 世界坐标系点云；后续运行加载旧点云并追加新扫描 |

PCD 的“增量”实现为**加载旧点云后追加新点**，不做点云配准、去重或体素降采样。两次运行须使用一致的世界坐标系和定位原点；若坐标系未对齐，旧、新点云会错位。点云逐帧追加会使文件持续增大，请定期检查磁盘空间。

LTAOM 的长期先验是另一套数据，默认位于 `~/fuel_sim_logs/ltaom/map_prior/`：

| 文件 | 用途 |
| --- | --- |
| `clouds_corrected.pcd` | LTAOM 位姿图校正后的关键帧点云 |
| `descriptors.pcd`、`plane_all.pcd` | STD 回环检索描述子及平面先验 |

LTAOM 多会话模式中，`session:=save` 对应生成并保存先验，`session:=load` 对应加载先验用于当前运行；后者不会将新一轮 LTAOM 点云写回旧 prior。FUEL 的 `session:=load` 则会将本轮新点追加到 `merged_map.pcd` 并保存更新后的占据图。

不要把 LTAOM 的 `clouds_corrected.pcd` 与 FUEL 的 `merged_map.pcd` 混为一份地图：两者由不同节点生成，服务于不同的复用机制。当前 `start_realflight.sh` 默认启动机载 Faster-LIO/EKF 与 FUEL，**不会启动 `ltaom_multisession.launch`**。因此普通实飞/手持流程只保证 FUEL 的 `occ_map.bin` 与 `merged_map.pcd`；只有实际启动并确认 LTAOM、STD 节点运行时，才会生成 LTAOM 先验。不要在默认启动栈上再直接启动第二个 FAST-LIO/LTAOM 实例，两个定位实例可能竞争相同 ROS 话题。LTAOM 实机应用还必须先确认定位、点云和控制器使用同一坐标系；仿真结果不代表已完成真实飞行验证。

> 实飞前必须由有经验的操作者在合法、空旷且受控的场地操作，并遵守机体、飞控和场地安全规程。始终保持遥控器接管能力；本指南不能代替实际的失控保护和安全检查。

## 1. 编译和配置检查（首次使用或代码更新后）

### 1.1 软件环境

本工程按 Ubuntu 20.04 / ROS Noetic 配置。安装工程依赖后，在工程根目录编译：

```bash
cd ~/uav_explore
bash compile_fuel.sh
```

完整依赖和 FAST-LIO 集成说明见[根目录 README](../README.md)及[FASTER-LIO 集成说明](./FASTER_LIO_INTEGRATION.md)。不要在飞行现场第一次编译或升级依赖。

### 1.2 确认硬件与参数

启动前核对并按当前机体/场地修改：

- MID360 网口和雷达 IP 与 `src/realflight_modules/faster-lio` 的配置相符。
- MAVROS 串口、飞控连接以及 `/mavros/local_position/odom` 坐标系正常。
- 雷达外参、机体到 IMU 外参已标定；`imu_body_T` 等配置不能未经确认地沿用占位零值。
- `world_frame` 与里程计的 `header.frame_id` 一致。默认实飞为 `map`；使用别的定位源时，按该源调整 `world_frame` 和 `odom_topic`。
- `home_position` 是局部 ENU 坐标中的返航点，必须是安全、可到达的空闲位置；探索边界必须符合实际场地，且完全位于 FUEL 地图范围内。
- `max_vel`、`max_acc`、地图尺寸和探索高度适合当前飞行场地。首轮实飞应使用经验证的保守参数。
- 电池阈值、悬停油门、PX4 OFFBOARD 丢失保护以及返航/人工接管流程已由操作者确认。

参数所在位置及含义见[实飞操作与参数说明](./REALFLIGHT_PARAMETERS.md)。不要直接把示例返航点、地图范围或飞控参数当作适用于当前场地的配置。

## 2. 通用启动与状态检查

### 2.1 启动完整系统

在机载电脑工程根目录打开终端：

```bash
cd ~/uav_explore
bash start_realflight.sh
```

`start_realflight.sh` 会启动 MAVROS、MID360/Faster-LIO、EKF 和 FUEL/控制器。需要无图形界面运行时可加 `rviz:=false`。其余 `realflight.launch` 参数可在命令行临时覆盖，例如：

```bash
bash start_realflight.sh rviz:=false max_vel:=0.5 max_acc:=0.25
```

多次运行地图时需明确指定 `session` 和地图目录；默认目录为 `$HOME/fuel_sim_logs`。`session` 与 `map_prior_dir` 会传给 FUEL；如果另行配置了 LTAOM，多会话 launch 也必须使用相同的会话值和地图根目录：

```bash
bash start_realflight.sh session:=save
bash start_realflight.sh session:=load
```

`save` / `load` 会话的保存逻辑在停止任务、返航到家稳定或确认落地时触发。对于正在运行的 LTAOM 节点，任务管理器也会分别置位 `/save_prior_info` 和 `/save_prior_descriptor`。手持静态测试若未启动过任务、任务管理器仍为 `READY`，请按第 4 节手动触发三个保存入口。

启动后等待所有节点稳定。另开终端检查：

```bash
cd ~/uav_explore
source ./source.sh

rostopic echo -n 1 /mavros/state
rostopic echo -n 1 /mavros/local_position/odom
rostopic hz /laserMapping/cloud_registered_body
rostopic echo -n 1 /fuel/cloud_world/header
rostopic echo -n 1 /uav_controller/fsm_status
```

应确认 MAVROS 已连接、里程计持续更新、Faster-LIO 点云有数据、`/fuel/cloud_world` 的 frame 与预期世界坐标系一致。RViz 中检查无人机周边点云/地图、探索边界和高度没有明显错位。不要在定位或点云异常时开始测试。

任务状态可通过启动终端的 `[FLIGHT_STATUS]` 状态行或以下命令观察：

```bash
rostopic echo /fuel/mission_status
rostopic echo /fuel/planner_status
```

启动、任务及控制器参数详见[实飞操作与参数说明](./REALFLIGHT_PARAMETERS.md)。

## 3. 实地飞行步骤

本节假设场地、定位、参数和飞控保护均已由负责飞行的人员确认。FUEL 不负责自动解锁或自动降落；返航到点后只悬停，最终降落由操作者执行。

### 3.1 第一次飞行：建图并设置为保存

1. 确认之前没有需要保留的地图，启动本次任务：

   ```bash
   cd ~/uav_explore
   bash start_realflight.sh session:=save
   ```

   `save` 模式不加载旧地图；本次飞行形成的地图会写到默认 `map_prior` 目录。若使用自定义路径，两次启动必须用相同的 `map_prior_dir`。

2. 在 RViz 检查世界坐标系、点云、地图、探索边界和返航点位置。
3. 操作者按已验证的飞控流程起飞。可以手动起飞并稳定悬停后切换 OFFBOARD，或使用已验证的地面自动起飞流程；不要在没有确认控制器、悬停油门和 PX4 行为时尝试自动起飞。
4. 确认飞控状态为 `OFFBOARD`、`armed: True`，控制器反馈新鲜且闭环正常，任务管理器已就绪后，再开始探索：

   ```bash
   source ~/uav_explore/source.sh
   rosservice call /fuel/start "{}"
   ```

   也可使用 RViz 的 **2D Nav Goal** 启动探索。

5. 飞行期间观察状态行、电量、点云和地图。异常时优先由操作者安全接管；人工返航服务为：

   ```bash
   rosservice call /fuel/return_home "{}"
   ```

   取消任务并请求悬停的服务为：

   ```bash
   rosservice call /fuel/stop "{}"
   ```

   注意：`/fuel/stop` 是停止当前任务并请求悬停，不是返航或降落命令。

6. 探索完成、触发低电量阈值或操作者请求返航时，FUEL 会规划到 `home_position`。确认状态进入 `HOME_HOLD`，并由操作者按现场流程接管降落。系统不会自动降落、上锁或切换飞控模式。
7. 确认飞控报告落地并保持稳定。任务管理器检测到本次飞行由空中转为落地后，会自动触发保存。等待日志确认保存完成，再停止启动终端：

   ```text
   MapROS: saved merged PCD to .../merged_map.pcd (N points)
   ```

   按 `Ctrl+C` 结束系统。检查输出文件：

   ```bash
   find ~/fuel_sim_logs/map_prior -maxdepth 1 -type f -size +0c -printf '%f (%s bytes)\n' | sort
   ```

   正常应有 `occ_map.bin` 和 `merged_map.pcd`。不要在保存完成前关闭建图节点或断电。

### 3.2 后续飞行：加载首轮地图并继续累积

1. 确认首轮 `occ_map.bin` 与 `merged_map.pcd` 都存在且非空。
2. 尽可能在与第一次运行相同的位置和朝向启动定位；保持定位源、坐标系、地图尺寸、分辨率及 `map_prior_dir` 一致。
3. 启动后续任务：

   ```bash
   cd ~/uav_explore
   bash start_realflight.sh session:=load
   ```

4. 启动日志应显示加载占据栅格图和 prior PCD，PCD 日志会报告已加载点数。若旧 PCD 加载失败，MapROS 会报错并拒绝写 PCD，避免覆盖旧文件；先排查路径或文件，不要继续并覆盖数据。
5. 确认 RViz 在无人机移动前已经显示旧栅格地图；检查旧地图和当前点云在同一位置。如果明显错位，停止任务并排查定位原点/坐标系，不要把错位结果当成融合成功。
6. 按 3.1 的安全流程起飞、开始探索、返航并由操作者降落。落地保存时，当前点云会追加到已加载的 PCD，并更新 `merged_map.pcd`；占据栅格图也会保存为续建后的地图。
7. 等待 saved PCD 日志后再停止节点，并核对文件时间与大小已更新。PCD 点数通常会增加；如果本次没有新点云，点数可能不增加。

> `session:=load` 不代表重新建一个单独的 PCD；它会把旧 PCD 与本次扫描写入同一个累积结果文件。保存为二进制 PCD，手动查看可使用 PCL 工具或 RViz 的 PointCloud2/PCD 加载工具。

## 4. 手持静态测试步骤（全程不飞）

此测试验证点云/里程计建图、PCD 保存以及第二次运行的增量累积。**不解锁、不起飞、不启动电机**；仅用手拿无人机缓慢移动。

### 4.1 开始前

1. 确认无人机已固定/保护桨叶，测试过程中不会解锁或切到会使电机转动的模式。
2. 确认机载电脑、雷达、定位链路和网络正常，测试区域内有足够空间让雷达看到环境。
3. 为避免覆盖已有地图，第一次手持测试使用单独的 `map_prior_dir`。先选一个新的目录；若目录已包含要保留的数据，请改用另一个目录，不要清理原地图：

   ```bash
   export MAP_DATA="$HOME/fuel_sim_logs/handheld_test"
   mkdir -p "$MAP_DATA"
   ```

### 4.2 第一轮手持建图与保存

1. 启动 `save` 会话：

   ```bash
   cd ~/uav_explore
   bash start_realflight.sh session:=save map_prior_dir:="$MAP_DATA"
   ```

2. 另开终端加载 ROS 环境并检查里程计：

   ```bash
   cd ~/uav_explore
   source ./source.sh
   rostopic echo -n 5 /mavros/local_position/odom
   ```

   慢慢手持无人机移动，确认位置随移动而变化、数值有限且 frame 正确。如果未解锁时 MAVROS 局部里程计不更新，不要继续误判为地图融合正常；按[实飞参数说明](./REALFLIGHT_PARAMETERS.md)和实际定位配置选择合适的定位源，并确保两轮测试使用同一坐标系及启动方式。

3. 检查世界系点云：

   ```bash
   rostopic hz /fuel/cloud_world
   rostopic echo -n 1 /fuel/cloud_world/header
   ```

   逐渐改变朝向和位置，让雷达看到测试区域中的稳定物体。观察 RViz 中点云与占据栅格地图是否随移动更新。
4. 手持静态测试没有经历“起飞后落地”。若没有成功启动并停止 FUEL 任务，任务管理器不会自动触发会话保存；为同时保存 FUEL 地图与已运行的 LTAOM 先验，在建图一段时间后手动触发三个保存入口：

   ```bash
   source ~/uav_explore/source.sh
   rostopic pub -1 /save_occ_map std_msgs/Empty "{}"
   rosparam set /save_prior_info true
   rosparam set /save_prior_descriptor true
   ```

   其中 `/save_occ_map` 保存 FUEL 栅格图与 FUEL 拼接点云；两个 ROS 参数分别由运行中的 `loop_optimization` 与 `std_loop` 消费。若 LTAOM 节点未启动或不处于 `multisession_mode=2`，不会产生 LTAOM 先验文件。

5. 等待启动终端出现栅格图和 PCD 保存成功日志，然后检查文件：

   ```bash
   ls -lh "$MAP_DATA/map_prior/occ_map.bin" \
          "$MAP_DATA/map_prior/merged_map.pcd"
   ```

   文件应存在且非空。若本次也运行了 LTAOM，再核对先验目录：

   ```bash
   ls -lh "$MAP_DATA/ltaom/map_prior"
   ```

   至少应看到本次配置对应的 `clouds_corrected.pcd`、`descriptors.pcd`，以及有有效平面数据时的 `plane_all.pcd`。保存完成后，在启动终端按 `Ctrl+C` 退出。

### 4.3 第二轮加载旧图、手持继续建图并保存

1. 将无人机尽量放回第一次测试的**同一位置、同一朝向**；确保定位初始化方式也相同。
2. 启动 `load` 会话：

   ```bash
   cd ~/uav_explore
   export MAP_DATA="$HOME/fuel_sim_logs/handheld_test"
   bash start_realflight.sh session:=load map_prior_dir:="$MAP_DATA"
   ```

3. 检查启动日志：
   - SDFMap 显示已加载 `occ_map.bin`。
   - MapROS 显示已加载 `merged_map.pcd` 和 prior 点数。
   - 尚未手持移动时，RViz 中已经能看到旧占据栅格地图。
4. 在场景原有区域附近缓慢手持移动，也可加入之前未覆盖的新视角。确认新点云在正确位置出现，旧地图保持可见。
5. 再次手动保存：

   ```bash
   source ~/uav_explore/source.sh
   rostopic pub -1 /save_occ_map std_msgs/Empty "{}"
   ```

6. 等待 `saved merged PCD` 日志，检查文件已更新：

   ```bash
   ls -lh --time-style=long-iso \
     "$MAP_DATA/map_prior/occ_map.bin" \
     "$MAP_DATA/map_prior/merged_map.pcd"
   ```

   保存点数应至少包含启动时读入的旧点数；如果手持移动期间持续收到新点云，通常点数会增加。上述两个 ROS 参数只会由处于 `multisession_mode=2` 的 LTAOM/STD 节点处理；`session:=load` 的 LTAOM 模式为只加载先验（模式 1），不会把新运行的 LTAOM 数据覆盖回 prior。FUEL 的 `merged_map.pcd` 则会继续保存累积结果。结束后按 `Ctrl+C` 停止。

## 5. 离线查看 PCD 点云

保存完成并关闭建图节点后，可在 ROS Noetic 环境中用本工程的 PCL-RViz 查看器打开点云，不需要连接飞控或雷达：

```bash
cd ~/uav_explore
bash scripts/view_saved_pcd.sh
```

默认打开 FUEL 点云 `~/fuel_sim_logs/map_prior/merged_map.pcd`，发布到 `/saved_map/points`。查看 LTAOM 的校正点云时显式传入文件路径和坐标系标签：

```bash
bash scripts/view_saved_pcd.sh \
  "$HOME/fuel_sim_logs/ltaom/map_prior/clouds_corrected.pcd" world
```

PCD 文件本身不记录 ROS `frame_id`；第二个参数只设置发布消息的 frame 标签，不会变换点坐标。必须按生成该 PCD 时的真实坐标系选择 `frame_id`，不能用改标签的方法修复错位。也可直接调用 launch：

```bash
source ~/uav_explore/source.sh
roslaunch fuel_bridge saved_pcd_viewer.launch \
  pcd_file:="$HOME/fuel_sim_logs/map_prior/merged_map.pcd" frame_id:=map
```

RViz 中应看到 `/saved_map/points`。关闭 RViz 或在启动终端按 `Ctrl+C` 退出。

## 6. 验收标准

| 检查项 | 通过标准 |
| --- | --- |
| 定位与点云输入 | 里程计位置有效且更新；`/fuel/cloud_world` 持续有点云 |
| 第一轮保存 | `occ_map.bin` 和 `merged_map.pcd` 均存在且非空；日志确认保存成功 |
| 第二轮加载 | 日志确认加载首轮占据图和 PCD；加载点数大于 0 |
| 旧图复用 | 新一轮移动前 RViz 已显示首轮地图 |
| 增量累积 | 第二轮保存成功，PCD 包含首轮旧点；有新扫描时保存点数大于旧点数 |
| 坐标对齐 | 旧图与本轮点云在 RViz 中位置合理，没有整体偏移或旋转错位 |
| LTAOM 先验（仅启用 LTAOM 时） | `clouds_corrected.pcd`、`descriptors.pcd` 成功写入；第二轮以 `session:=load` 启动时日志确认先验已载入 |
| 离线 PCD 查看 | `scripts/view_saved_pcd.sh` 发布 `/saved_map/points`，RViz 无 Fixed Frame 错误且点云位置合理 |

## 7. 常见问题

### 启动时提示 prior PCD 加载失败

确认命令是 `session:=load`，并检查 `map_prior_dir` 下 `merged_map.pcd` 存在、非空且可读。加载失败时程序会拒绝保存 PCD，避免用空的新图覆盖旧文件。不要为了消除报错而删除旧文件，除非确定要重新开始。

### 第二轮 RViz 里没有旧占据地图

确认 `session:=load`，检查 `occ_map.bin` 路径和启动日志；地图大小、分辨率等参数必须与首次保存时相同。若看到 `map dims mismatch!`，恢复首次使用的地图配置或使用新的独立地图目录。

### 旧地图和当前点云错位

两轮定位原点、启动位置/朝向或 `world_frame` 不一致。停止测试，先统一定位源和坐标系；PCD 追加本身不会估计变换或自动配准。

### 没有生成 PCD

确认设置了 `session:=save` 或 `session:=load`，检查 `/fuel/cloud_world` 有数据，并确认 `/save_occ_map` 触发已到达 MapROS。手持静态测试必须手动发保存信号；只有真实经历 airborne → on-ground 的飞行才会触发自动落地保存。

### 保存文件很大

当前 PCD 累积是原始点追加，不去重、不降采样，文件会随运行时间增长。确保磁盘空间足够；需要精简时先备份原文件，再在单独的处理流程中做体素降采样，勿直接覆盖唯一的原始地图。

### LTAOM 先验没有保存

确认本轮确实运行了 LTAOM 和 STD 节点，并且当前为生成先验的 `session:=save`（`/multisession_mode` 为 `2`）。`session:=load` 是只加载 prior（模式 `1`），不会保存新 LTAOM 数据。纯手持、且任务管理器处于 `READY` 时，需在 `save` 会话手动设置 `/save_prior_info` 与 `/save_prior_descriptor`；仅发布 `/save_occ_map` 只会保存 FUEL 地图，不会触发 LTAOM 文件写入。

### RViz 显示 `No transform` 或点云未显示

确认 `scripts/view_saved_pcd.sh` 的第二个参数与 PCD 内容坐标系相符，RViz Fixed Frame 与该标签一致，并检查 `/saved_map/points` 是否有数据：

```bash
rostopic hz /saved_map/points
rostopic echo -n 1 /saved_map/points/header
```

改 frame 标签不会对点云做坐标变换；若点云与其他地图错位，应回查原始里程计/配准结果。

### 需要重新开始全新地图

优先为新测试指定另一个独立的 `map_prior_dir`。若要清理旧文件，先列出路径并备份，然后只删除明确的 `occ_map.bin`、`merged_map.pcd` 文件；不要删除整个日志目录。

## 8. 飞行数据记录（可选）

起飞前可在另一终端开始记录任务状态和控制器关键话题，降落后按 `Ctrl+C` 停止：

```bash
cd ~/uav_explore
source ./source.sh
bash scripts/record_flight.sh
```

默认记录文件写入 `~/flight_bags`；可用 `FLIGHT_BAG_DIR` 指定其他目录。更多说明见[实飞操作与参数说明](./REALFLIGHT_PARAMETERS.md)。
