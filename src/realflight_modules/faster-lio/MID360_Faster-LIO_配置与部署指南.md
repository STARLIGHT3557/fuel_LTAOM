> 本目录已复制到 uav_explor_LTAOM，随主工程统一管理，无独立 Git 仓库。构建与启动路径见 [集成说明](../../../docs/FASTER_LIO_INTEGRATION.md)。下文保留源工程历史资料。

# MID360 + Faster-LIO 配置与部署指南

本文记录本次在机载电脑上的实际部署，包含下载、依赖、文件修改、编译、启动、RViz 和测试方法。适用于 **MID360 使用内置 IMU，整体向机头下倾 15°** 的安装方式。

**当前完成的是雷达定位与显示。飞控 IMU 升频 EKF 已编译并调整输入话题，但尚未进行实际融合验证；没有向 PX4 回传定位，也没有修改飞控参数。**

## 1. 本机环境与目录

| 项目 | 本次配置 |
|---|---|
| 机载电脑 | Ubuntu 20.04.6，aarch64 / Jetson |
| ROS | ROS Noetic，ROS1 |
| SSH | `ssh ubt@192.168.10.116` |
| 电脑连接雷达的网口 | `eth0`，`192.168.1.50/24` |
| MID360 地址 | `192.168.1.199` |
| 飞控固件 | PX4 1.14，由用户提供版本信息 |
| 工作空间 | `/home/ubt/workspace` |
| 仓库目录 | `/home/ubt/workspace/src/realflight_modules/faster-lio` |
| 上游仓库 | [DifferentialRobotics/faster-lio](https://github.com/DifferentialRobotics/faster-lio) |
| 本次源码提交 | `c8d635c4689f5ebd89a9532411254c93eaa35adc` |

仓库内部还有一层 `faster-lio`，不是下载重复：

```text
/home/ubt/workspace/src/realflight_modules/faster-lio/
├── faster-lio/          # ROS 包名 faster_lio：定位与建图
├── livox_ros_driver2/   # MID360 驱动
├── ekf_pose/            # ROS 包名 ekf：可选的升频融合
├── scripts/
│   └── start_mid360.sh  # 本次新增的一键启动脚本
├── UPSTREAM_COMMIT     # 本次记录的源码提交
└── DEPLOYMENT_zh.md     # 机载电脑上的部署说明
```

后文的修改文件路径，除特别说明外，均相对于上述仓库目录。

## 2. 下载源码

**现有机载电脑已经部署完成，无需重新下载。以下步骤用于新环境复现，目标目录应尚不存在。两种下载方式选择一种。**

### 2.1 Git 下载

```bash
mkdir -p /home/ubt/workspace/src/realflight_modules
cd /home/ubt/workspace/src/realflight_modules

git clone https://github.com/DifferentialRobotics/faster-lio.git
cd faster-lio

# 固定到本次验证的版本
git checkout c8d635c4689f5ebd89a9532411254c93eaa35adc
```

### 2.2 Git 克隆失败时：下载官方源码包

本次机载电脑的 Git HTTPS 克隆出现 `Empty reply from server`，因此实际使用 GitHub 官方 codeload 下载了对应提交的源码包。

以下命令在子 shell 中执行；如果目标目录已存在，会退出，避免覆盖已配置的文件：

```bash
(
    set -e
    REV=c8d635c4689f5ebd89a9532411254c93eaa35adc
    TARGET=/home/ubt/workspace/src/realflight_modules/faster-lio
    CACHE=/home/ubt/.cache/faster-lio-deploy

    if [ -e "$TARGET" ]; then
        printf '目标目录已存在，请勿重复下载覆盖：%s\n' "$TARGET"
        exit 1
    fi

    mkdir -p "$CACHE"
    curl -fL --connect-timeout 10 --max-time 120 \
      -o "$CACHE/$REV.tar.gz" \
      "https://codeload.github.com/DifferentialRobotics/faster-lio/tar.gz/$REV"

    mkdir -p "$TARGET"
    tar -xzf "$CACHE/$REV.tar.gz" --strip-components=1 -C "$TARGET"
    printf '%s\n' "$REV" > "$TARGET/UPSTREAM_COMMIT"
)
```

源码包方式不包含 `.git`，因此不能直接在该目录执行 `git pull`。本次下载包保存在 `/home/ubt/.cache/faster-lio-deploy/`。

## 3. 检查依赖与网络

### 3.1 ROS 和编译依赖

本文以已安装 ROS Noetic 为前提。本次检查时，机载电脑已经具备主要依赖；没有重装 ROS，也没有重装 Livox SDK2。

新环境缺少依赖时，可在安装好 ROS Noetic 并配置其软件源后执行：

```bash
sudo apt update
sudo apt install -y \
  build-essential cmake git curl \
  libeigen3-dev libpcl-dev libgoogle-glog-dev libgflags-dev \
  libtbb-dev libyaml-cpp-dev libapr1-dev \
  python3-yaml python3-numpy \
  ros-noetic-pcl-ros ros-noetic-eigen-conversions ros-noetic-rviz
```

只有后续使用飞控 IMU、与 PX4 通信时，才需要 MAVROS。当前这台电脑已有 MAVROS。

### 3.2 Livox SDK2

检查：

```bash
ls /usr/local/lib/liblivox_lidar_sdk_static.a
ls /usr/local/include/livox_lidar_api.h
```

若缺失，可参考 [Livox SDK2 官方安装说明](https://github.com/Livox-SDK/Livox-SDK2#22-instruction-for-ubuntu-2004) 安装：

```bash
mkdir -p /home/ubt/software
cd /home/ubt/software
git clone https://github.com/Livox-SDK/Livox-SDK2.git
cd Livox-SDK2
mkdir -p build
cd build
cmake ..
make -j4
sudo make install
sudo ldconfig
```

本仓库已包含 `livox_ros_driver2`，不需要再向同一 catkin 工作空间添加第二份同名驱动包。

### 3.3 网络检查

```bash
ip -br addr
ping -c 3 192.168.1.199
```

本次确认 `eth0` 已是 `192.168.1.50/24`，雷达可 ping 通，所以没有修改电脑网口设置或雷达自身 IP。若换一台电脑，需要先将连接雷达的网口设为同网段的 `192.168.1.50/24`。

## 4. 修改雷达网络配置

文件：`livox_ros_driver2/config/MID360_config.json`。

**`host_net_info` 填电脑地址；`lidar_configs[].ip` 填雷达地址。上游 README 对这两者的文字说明写反了，不要照反方向填写。**

本次完整配置如下：

```json
{
  "lidar_summary_info": {
    "lidar_type": 8
  },
  "MID360": {
    "lidar_net_info": {
      "cmd_data_port": 56100,
      "push_msg_port": 56200,
      "point_data_port": 56300,
      "imu_data_port": 56400,
      "log_data_port": 56500
    },
    "host_net_info": {
      "cmd_data_ip": "192.168.1.50",
      "cmd_data_port": 56101,
      "push_msg_ip": "192.168.1.50",
      "push_msg_port": 56201,
      "point_data_ip": "192.168.1.50",
      "point_data_port": 56301,
      "imu_data_ip": "192.168.1.50",
      "imu_data_port": 56401,
      "log_data_ip": "",
      "log_data_port": 56501
    }
  },
  "lidar_configs": [
    {
      "ip": "192.168.1.199",
      "pcl_data_type": 1,
      "pattern_mode": 0,
      "extrinsic_parameter": {
        "roll": 0.0,
        "pitch": 0.0,
        "yaw": 0.0,
        "x": 0,
        "y": 0,
        "z": 0
      }
    }
  ]
}
```

驱动中的 `extrinsic_parameter` 保持零。15° 安装角在 Faster-LIO 的机体转换中处理，不在驱动里再次旋转点云。

### 4.1 修正驱动读取 IP 的位置

文件：`livox_ros_driver2/src/parse_cfg_file/parse_livox_lidar_cfg.cpp`。

上游实际通过环境变量读取 IP：

```cpp
// user_config.handle = IpStringToNum(std::string(config["ip"].GetString()));
user_config.handle = IpStringToNum(std::string(std::getenv("LIDAR_IP")));
```

改成读取上面 JSON 中的配置：

```cpp
// Read the explicit lidar IP from MID360_config.json.
user_config.handle = IpStringToNum(std::string(config["ip"].GetString()));
```

这样启动不再依赖 `LIDAR_IP` 环境变量，也避免环境变量缺失时构造字符串出错。这是 C++ 源码修改，需要重新编译。

## 5. 使用内置 IMU，并配置 15° 安装角

文件：`faster-lio/config/mid360.yaml`。

这里有两套外参，含义不同：

| 参数 | 含义 | 本次处理 |
|---|---|---|
| `extrinsic_R/T` | 雷达测量坐标系到内置 IMU 坐标系 | 使用 MID360 内部固定外参 |
| `imu_body_R/T` | 机体坐标系到内置 IMU 坐标系；用于输出机体位姿 | 在这里配置安装角与机体原点位置 |

15° 矩阵适用前提：**雷达安装前 X 向机头、Y 向左、Z 向上，然后整体向机头下倾 15°**。如果雷达还绕竖直方向转了角度或倒装，需要重新计算矩阵。

可将 YAML 替换为以下配置，其参数值与本次部署一致：

```yaml
common:
    lid_topic: "/livox/lidar"
    imu_topic: "/livox/imu"
    time_sync_en: false

preprocess:
    lidar_type: 1
    scan_line: 4
    blind: 0.4
    time_scale: 1e-9

mapping:
    acc_cov: 0.1
    gyr_cov: 0.1
    b_acc_cov: 0.0001
    b_gyr_cov: 0.0001
    fov_degree: 360
    det_range: 100.0
    extrinsic_est_en: false

    # 雷达到其内置 IMU：p_I = R_IL * p_L + t_IL
    extrinsic_T: [-0.011, -0.02329, 0.04412]
    extrinsic_R: [1, 0, 0, 0, 1, 0, 0, 0, 1]

    # 机体到 IMU 的旋转：Ry(-15°)
    imu_body_R: [0.9659258262890683, 0, -0.25881904510252074,
                 0, 1, 0,
                 0.25881904510252074, 0, 0.9659258262890683]

    # 未提供位移标定值：原点暂留在 MID360 IMU 处
    imu_body_T: [0.0, 0.0, 0.0]

publish:
    path_publish_en: true
    scan_publish_en: true
    dense_publish_en: false
    scan_bodyframe_pub_en: true

pcd_save:
    pcd_save_en: false
    interval: -1

feature_extract_enable: false
point_filter_num: 3
max_iteration: 3
filter_size_surf: 0.5
filter_size_map: 0.5
cube_side_length: 1000
ivox_grid_resolution: 0.5
ivox_nearby_type: 18
esti_plane_threshold: 0.1
```

关键变化：

- IMU 话题从 `/mavros/imu/data` 改为 `/livox/imu`。
- 激活雷达内置 IMU 的固定外参，替换上游默认的外置飞控 IMU 外参。
- `imu_body_R` 配置下倾 15° 的机体姿态转换。
- `imu_body_T` 暂时为零，不使用原作者无人机的安装位移。
- `pcd_save_en` 从 `true` 改为 `false`，避免按 `interval: -1` 无限累计完整点云。RViz 显示和定位仍然工作。
- 其他滤波、匹配和噪声参数沿用该版本已有取值，未做针对本机的精度调参。

### 5.1 位移外参后续怎么填

`imu_body_T` 是 **从 MID360 IMU 原点指向所选机体原点的向量，用 MID360 IMU 坐标系表示，单位米**。

若测得的是反向向量——机体原点到 MID360 IMU 的位置 `t_BI`，且用机体坐标表达，则转换为：

```text
imu_body_T = -imu_body_R × t_BI
```

当前零值表示：`odometry_body` 的轴向已按机体转换，但位置原点仍在 MID360 IMU，不能认为已经得到飞控中心或重心位置。

使用内置 IMU 后，点云和惯性数据来自同一台设备；`time_sync_en: false` 不代表后续与 PX4 融合时无需检查时钟同步。

## 6. 修改启动文件

文件：`faster-lio/launch/mapping_mid360.launch`。

将：

```xml
<arg name="bd_list" default="$(env BD_LIST)"/>
```

改为：

```xml
<arg name="bd_list" default="$(optenv BD_LIST)"/>
```

这样未设置 `BD_LIST` 也可以启动。已完成第 4 节的 IP 读取修正后，正常启动不需要设置 `BD_LIST` 或 `LIDAR_IP`。

该 launch 原本就包含驱动和定位两个节点；本次保留 `xfer_format=1`、点云 `publish_freq=10.0`。不要同时启动另一份 Livox 驱动。

## 7. 修正 RViz 配置

文件：`faster-lio/config/rviz/faster_lio.rviz`。

上游文件仍使用旧版话题与坐标系，导致“话题有数据，但 RViz 显示空白、Fixed Frame 报错”。本次已做如下修改：

| 位置 | 原值 | 修改后 |
|---|---|---|
| Global Options → Fixed Frame | `camera_init` | `world` |
| Views → Current View → Target Frame | `global` | `world` |
| 点云显示 Topic，两处 | `/cloud_registered` | `/laserMapping/cloud_registered` |
| Odometry 显示 Topic | `/Odometry` | `/laserMapping/odometry_body` |
| Path 显示 Topic | `/path` | `/laserMapping/path` |
| Path → Alpha | `0` | `1` |
| `/obstacle_removal/output` 显示项 | 启用 | 关闭；当前没有该输出 |
| Current View → Distance | 约 `31.55` | `10` |
| Current View → Focal Point | 原作者保存的位置 | `(0, 0, 0)` |

可在 RViz 界面修改以上项目，然后通过 **File → Save Config** 保存到该文件。

如果是在磁盘上修改文件，已经打开的 RViz 不会自动重新加载。请选择 **File → Open Config**，打开：

```text
/home/ubt/workspace/src/realflight_modules/faster-lio/faster-lio/config/rviz/faster_lio.rviz
```

若提示保存当前旧配置，选择不保存，避免覆盖刚修好的文件。无需重启雷达定位。

## 8. 编译

修改完 C++ 源码后，在工作空间根目录编译：

```bash
source /opt/ros/noetic/setup.bash
cd /home/ubt/workspace
catkin_make -j4 -l4 -DCMAKE_BUILD_TYPE=Release
source /home/ubt/workspace/devel/setup.bash
```

本次以下目标均已编译成功：

- `livox_ros_driver2_node`
- `run_mapping_online`
- `run_mapping_offline`
- `ekf`

本次没有修改 `CMakeLists.txt`。驱动 IP 解析的 C++ 修改需要编译；后续只改 YAML、launch 或 RViz 配置，一般无需重新编译，重新启动相应节点或重新加载显示配置即可。

## 9. 启动与检查

### 9.1 仅测试雷达定位

```bash
source /opt/ros/noetic/setup.bash
source /home/ubt/workspace/devel/setup.bash
roslaunch faster_lio mapping_mid360.launch
```

启动时保持设备静止，等待日志出现 `IMU Initial Done`。这一阶段不需要 MAVROS，也不需要升频 EKF。

### 9.2 同时启动 RViz

在机载电脑的图形桌面终端执行：

```bash
source /opt/ros/noetic/setup.bash
source /home/ubt/workspace/devel/setup.bash
roslaunch faster_lio mapping_mid360.launch rviz:=true
```

与上一种启动方式二选一，不要同时启动两套。普通 SSH 终端没有图形显示环境时，先运行不带 RViz 的方式。

### 9.3 本次新增的一键启动脚本

当前电脑可以直接执行：

```bash
/home/ubt/workspace/src/realflight_modules/faster-lio/scripts/start_mid360.sh
```

或在图形桌面执行：

```bash
/home/ubt/workspace/src/realflight_modules/faster-lio/scripts/start_mid360.sh rviz:=true
```

这个脚本是本次新增的，不在上游源码中。新环境若需要复现，创建 `scripts/start_mid360.sh`，内容如下，并执行 `chmod +x scripts/start_mid360.sh`：

```bash
#!/usr/bin/env bash
set -eo pipefail
repo_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
workspace_dir="$(cd -- "$repo_dir/../../.." && pwd)"
source /opt/ros/noetic/setup.bash
if [ ! -f "$workspace_dir/devel/setup.bash" ]; then
    printf 'Workspace is not built: %s\n' "$workspace_dir" >&2
    exit 1
fi
source "$workspace_dir/devel/setup.bash"
exec roslaunch faster_lio mapping_mid360.launch "$@"
```

### 9.4 检查话题

另开一个终端，先执行：

```bash
source /home/ubt/workspace/devel/setup.bash
```

然后逐项检查，每项看完按 `Ctrl+C`：

```bash
rostopic hz /livox/imu
rostopic hz /livox/lidar
rostopic hz /laserMapping/odometry_body
```

| 话题 | 内容 | 本次频率 |
|---|---|---:|
| `/livox/imu` | MID360 内置 IMU | 约 200 Hz |
| `/livox/lidar` | CustomMsg 原始点云 | 约 10 Hz |
| `/laserMapping/odometry` | MID360 IMU 位姿，`world → imu` | 约 10 Hz |
| `/laserMapping/odometry_body` | 经过安装外参转换的机体位姿，`world → body` | 约 10 Hz |
| `/laserMapping/cloud_registered` | 世界坐标系点云，供 RViz 显示 | 约 10 Hz |
| `/laserMapping/path` | 位姿轨迹 | 不作为固定频率指标 |

注意：上游 `/laserMapping/cloud_registered_body` 实际使用 IMU 坐标系，不要仅凭话题名字认为它已经转换到机体坐标系。

## 10. 可选：飞控 IMU 升频 EKF

这一部分是机载电脑上的独立程序，与 PX4 内部 EKF2 不同。Faster-LIO 使用雷达内置 IMU，不代表这层升频 EKF 也自动使用雷达 IMU。

```text
MID360 点云 + MID360 IMU → Faster-LIO → odometry_body
                                           ↓
飞控 IMU → MAVROS → /mavros/imu/data_raw → 升频 EKF → /ekf/ekf_odom
```

### 10.1 本次已修改的内容

文件：`ekf_pose/launch/ekf_lidar.launch`。

```xml
<remap from="~imu" to="/mavros/imu/data_raw"/>
<remap from="~bodyodometry" to="/laserMapping/odometry_body"/>
<remap from="~ekf_odom" to="/ekf/ekf_odom"/>
```

原来的两个输入分别是 `/mavros/imu/data`、`/laserMapping/odometry`。还增加了注释，说明位移外参需要标定。

文件：`ekf_pose/launch/PX4_vio_drone.yaml`。

该文件的数值保持原值；本次主要更新了话题/单位注释和“位移未标定”的说明：

```yaml
Rr_i: [1, 0, 0, 0]       # 四元数 w,x,y,z
tr_i: [0.0, 0.0, 0.0]   # 尚未标定
scale_g: 1.0             # MAVROS 加速度已经是 m/s²
rotation_imu: [1, 0, 0,
               0, 1, 0,
               0, 0, 1]
```

### 10.2 尚需完成的事项

- 飞控 IMU 和 LIO 时间戳必须处于共同时间基准。源码的缓存对齐功能不等于设备时钟同步。
- 测量并配置 `tr_i` 及 launch 中的 `imu_trans_x/y/z`。在机体与飞控 IMU 轴向一致时，两组平移均应对应“飞控 IMU 到所选输出机体原点”的向量，以飞控 IMU / FLU 坐标表达。
- 当前机体原点暂留在 MID360 IMU 处，不能直接假定它与飞控 IMU 原点重合。
- 这个 EKF 只消费加速度和角速度，所以选用 `/mavros/imu/data_raw`；缺少姿态四元数并不妨碍这里的输入。

完成以上工作后，启动有效的 MAVROS 连接，再启动 `roslaunch ekf ekf_lidar.launch`。MAVROS 的串口设备及波特率需要按实际接线设置，本次未确认这些参数。

如需请求飞控以 200 Hz 发送 HIGHRES_IMU，可在 MAVROS 已连接、对应服务可用后执行：

```bash
rosservice call /mavros/set_message_interval \
  "{message_id: 105, message_rate: 200.0}"

rostopic hz /mavros/imu/data_raw
```

这是后续操作说明，本次部署没有执行该升频请求。实际频率受链路带宽和消息源更新频率影响，不能只看服务返回值。

上游 EKF 的位置滤波器按 200 Hz 编写，前 1000 次发布调用会跳过输出。200 Hz 输入时，开始输出前约需 5 秒。改变输入频率时，需要同时检查这些假设。

**本次没有把 `/ekf/ekf_odom` 接入 PX4。**如后续用于飞控外部定位，还需处理输出坐标、消息格式、协方差、定位超时以及 PX4 1.14 参数，不能仅凭 EKF 能输出话题就认为飞行接入已完成。接口可参考 [PX4 1.14 外部定位说明](https://docs.px4.io/v1.14/en/ros/external_position_estimation)。

## 11. 本次实际修改清单

| 文件 | 修改内容 |
|---|---|
| `livox_ros_driver2/config/MID360_config.json` | 电脑地址改为 `.50`，雷达地址写为 `.199`；端口与驱动外参保持原设置 |
| `livox_ros_driver2/src/parse_cfg_file/parse_livox_lidar_cfg.cpp` | 从 JSON 读取 IP，移除对 `LIDAR_IP` 环境变量的强制依赖 |
| `faster-lio/config/mid360.yaml` | 内置 IMU、内部外参、15° 机体转换、零位移占位、关闭 PCD 累积保存 |
| `faster-lio/launch/mapping_mid360.launch` | `BD_LIST` 改为可选环境变量 |
| `faster-lio/config/rviz/faster_lio.rviz` | 修正坐标系和话题；恢复路径可见度；关闭无用显示项；调整初始视角 |
| `ekf_pose/launch/ekf_lidar.launch` | 输入改为 `data_raw` 和 `odometry_body`，增加标定说明 |
| `ekf_pose/launch/PX4_vio_drone.yaml` | 增加未标定位移及输入单位说明；参数数值未改 |
| `scripts/start_mid360.sh` | 新增加载环境并启动定位的脚本 |
| `DEPLOYMENT_zh.md`、版本/备份记录文件 | 新增本机部署说明与追溯信息 |

以下内容没有修改：

- Faster-LIO 的重力初始化和匹配算法。
- 此前讨论的 `mean_gyr_ = rotation * mean_gyr_;` 代码仍保留上游实现。
- 升频 EKF 的 C++ 算法实现。
- PX4 固件与参数。
- 无人机运动控制、解锁和自动飞行配置。

## 12. 已完成的验证与记录位置

编译完成后，在独立 ROS master `11321` 上采样约 25 秒；测试结束后关闭了测试节点。用户正常启动时无需改成这个端口。

| 检查项 | 结果 |
|---|---|
| 雷达可达性 | `192.168.1.199` ping 正常 |
| 编译 | 驱动、在线/离线 Faster-LIO、EKF 全部成功 |
| 雷达 IMU | 200.01 Hz |
| 原始点云 | 10.01 Hz，平均每帧 20000 点 |
| 两路里程计 | 均约 10.00 Hz |
| 注册点云 | 约 10.00 Hz |
| 时间戳 | 采样区间内递增 |
| IMU、位置、姿态 | 未出现 NaN / Inf |
| 安装角转换示例 | 采样末尾 IMU pitch 15.98°，机体 pitch 0.98° |
| RViz 故障复查 | 实际点云 frame 为 `world`，`world → body` TF 可查询；显示配置已修正 |

上述结果是数据通路与短时运行验证，不是定位精度评估、飞控融合验证或飞行验证。RViz 修正后需要重新加载配置；没有把配置文件检查等同于屏幕渲染验收。

机载电脑上的记录：

```text
/home/ubt/workspace/logs/faster-lio-build.log
/home/ubt/workspace/logs/mid360-live-check.json
/home/ubt/workspace/logs/mid360-live-run.log
/home/ubt/workspace/logs/faster-lio-deployment.json
/home/ubt/workspace/logs/faster-lio-deployment.patch
/home/ubt/workspace/logs/rviz-display-fix.patch
```

原文件备份：

```text
/home/ubt/workspace/.deployment-backups/faster-lio-20260923T062616/
/home/ubt/workspace/.deployment-backups/rviz-20260923T064424/faster_lio.rviz
```

首次部署 JSON/patch 记录初次修改；后续 RViz 修正单独记录在 `rviz-display-fix.patch` 中。

## 13. 常见问题

| 现象 | 检查方法 |
|---|---|
| 找不到 `faster_lio` 包 | 先 `source /home/ubt/workspace/devel/setup.bash`，再执行 `rospack find faster_lio` |
| 雷达没有数据 | 检查供电、网口 `.50`、雷达 `.199`、JSON 的 host/lidar 地址是否填反；确认未重复启动驱动 |
| 提示缺少 `BD_LIST` | 检查 launch 是否已经从 `env` 改为 `optenv` |
| 修改 JSON 的 IP 无效或启动异常 | 检查驱动是否仍读取 `getenv("LIDAR_IP")`；修改源码后重新编译 |
| RViz 显示 Unknown frame | Fixed Frame 和 Target Frame 都设为 `world`，确认 `world → body` TF 存在 |
| RViz 无点云但话题有数据 | PointCloud2 应订阅 `/laserMapping/cloud_registered`；原始 `/livox/lidar` 是 CustomMsg |
| RViz 无轨迹 | Topic 设为 `/laserMapping/path`，Alpha 设为 `1` |
| SSH 启动 RViz 提示无法连接显示器 | 使用机载电脑图形桌面；普通 SSH 先仅运行定位 |
| EKF 刚启动没有输出 | 检查 IMU/LIO 输入、时间戳及上游前 1000 次调用的启动等待逻辑 |
| 请求 200 Hz 后实测不足 | 检查 MAVLink 带宽、串口波特率、消息源频率与丢包，避免只依据请求值判断 |
