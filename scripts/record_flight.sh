#!/usr/bin/env bash
# 录制 FUEL 状态机与控制跟踪数据，用于飞后分析。
# 用法：bash scripts/record_flight.sh [额外话题 ...]
#   起飞前启动，降落后按 Ctrl+C 停止。
#   bag 默认保存到 ~/flight_bags，文件名 track_<时间>.bag；可用 FLIGHT_BAG_DIR 修改目录。
#   例：bash scripts/record_flight.sh /mavros/setpoint_raw/attitude   # 额外录姿态和油门指令
set -eo pipefail
fuel_root=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
source /opt/ros/noetic/setup.bash
source "$fuel_root/source.sh"

if ! rosnode list >/dev/null 2>&1; then
  echo "ROS master 未运行，请先执行 bash start_realflight.sh。" >&2
  exit 1
fi

bag_dir="${FLIGHT_BAG_DIR:-$HOME/flight_bags}"
mkdir -p "$bag_dir"

topics=(
  # FUEL 状态机：规划器状态（state、连续失败次数、失败原因）和任务状态
  /fuel/planner_status
  /fuel/mission_status
  # 控制阶段及切换原因
  /uav_controller/fsm_status
  # 跟踪效果：控制器参考值（数值为 ENU，与 local_position 相同）与实际位姿、速度
  /uav_controller/active_setpoint
  /mavros/local_position/pose
  /mavros/local_position/velocity_local
)

echo "录制到 $bag_dir，按 Ctrl+C 停止。"
exec rosbag record -o "$bag_dir/track" "${topics[@]}" "$@"
