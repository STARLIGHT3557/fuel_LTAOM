#!/usr/bin/env bash
# One local catkin workspace; no downloads or patch application at build time.
set -eo pipefail
fuel_root=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
unset CMAKE_PREFIX_PATH ROS_PACKAGE_PATH LD_LIBRARY_PATH PYTHONPATH
source /opt/ros/noetic/setup.bash
catkin_make -C "$fuel_root" --source "$fuel_root/src" --build "$fuel_root/build/fuel" \
  -j"${FUEL_BUILD_JOBS:-2}" -l"${FUEL_BUILD_JOBS:-2}" \
  -DCATKIN_DEVEL_PREFIX="$fuel_root/devel/fuel" \
  -DCMAKE_BUILD_TYPE=Release -DPYTHON_EXECUTABLE=/usr/bin/python3 "$@"
