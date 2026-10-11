#!/usr/bin/env bash
set -e

cd "$(dirname "${BASH_SOURCE[0]}")"
source /opt/ros/noetic/setup.bash
source ./source.sh

# 退出时只停止本脚本启动的进程。
trap 'kill $(jobs -pr) 2>/dev/null || true; wait' EXIT
trap 'exit 0' INT TERM

roslaunch mavros px4.launch &
mavros_pid=$!
while ! rosparam list >/dev/null 2>&1; do
  kill -0 "$mavros_pid" 2>/dev/null || exit 1
  sleep 1
done

roslaunch faster_lio mapping_mid360.launch &
roslaunch ekf ekf_lidar.launch &
bash ./fuel_realflight.sh "$@" &

wait -n
