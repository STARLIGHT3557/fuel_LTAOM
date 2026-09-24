#!/usr/bin/env bash
set -eo pipefail
fuel_root=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
source "$fuel_root/source.sh"
for fuel_node in prometheus_fuel/fuel_bridge uav_controller/geometric_controller_node exploration_manager/exploration_node plan_manage/traj_server; do
  if [[ ! -x "$fuel_root/devel/fuel/lib/$fuel_node" ]]; then
    echo "Build incomplete ($fuel_node missing); run bash compile_fuel.sh." >&2
    exit 1
  fi
done
if [[ ! -x "$PX4_BUILD_DIR/bin/px4" || ! -f "$PX4_BUILD_DIR/build_gazebo/libgazebo_mavlink_interface.so" ]]; then
  echo "Build external PX4 SITL first; check PX4_ROOT and PX4_BUILD_DIR ($PX4_BUILD_DIR)." >&2
  exit 1
fi
exec roslaunch prometheus_fuel simulation.launch "$@"
