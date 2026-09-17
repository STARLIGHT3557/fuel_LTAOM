#!/usr/bin/env bash
# FUEL and EGO contain incompatible packages with the same names.
# Build FUEL against /opt/ros only, sharing this project's message sources.
set -eo pipefail
fuel_root=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
fuel_upstream="$fuel_root/build/fuel_upstream"
fuel_ws="$fuel_root/build/fuel_ws"
fuel_revision=662dd23c7b52b258d3c4a0155ff6632118e8984f
mkdir -p "$fuel_ws/src"
if [[ ! -d "$fuel_upstream/.git" ]]; then
  git clone https://github.com/HKUST-Aerial-Robotics/FUEL.git "$fuel_upstream"
  git -C "$fuel_upstream" checkout --detach "$fuel_revision"
fi
if [[ $(git -C "$fuel_upstream" rev-parse HEAD) != "$fuel_revision" ]]; then
  echo "Unexpected FUEL revision in $fuel_upstream; expected $fuel_revision" >&2
  exit 1
fi
fuel_patch="$fuel_root/Modules/fuel_bridge/patches/noetic_mid360.patch"
if git -C "$fuel_upstream" apply --reverse --check "$fuel_patch" 2>/dev/null; then
  : # Already patched.
else
  git -C "$fuel_upstream" apply --check "$fuel_patch"
  git -C "$fuel_upstream" apply "$fuel_patch"
fi
ln -sfn "$fuel_upstream/fuel_planner" "$fuel_ws/src/fuel_planner"
ln -sfn "$fuel_root/Modules/common/prometheus_msgs" "$fuel_ws/src/prometheus_msgs"
ln -sfn "$fuel_root/Modules/common/quadrotor_msgs" "$fuel_ws/src/quadrotor_msgs"
ln -sfn "$fuel_root/Modules/fuel_bridge" "$fuel_ws/src/prometheus_fuel"
unset CMAKE_PREFIX_PATH ROS_PACKAGE_PATH LD_LIBRARY_PATH PYTHONPATH
source /opt/ros/noetic/setup.bash
catkin_make -C "$fuel_ws" -j"${FUEL_BUILD_JOBS:-2}" -l"${FUEL_BUILD_JOBS:-2}" \
  -DCMAKE_BUILD_TYPE=Release -DPYTHON_EXECUTABLE=/usr/bin/python3 "$@"
