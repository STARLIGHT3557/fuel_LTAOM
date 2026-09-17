#!/usr/bin/env bash
# Run only FUEL processes with the isolated workspace's libraries and packages.
set -e
# This script can also be reached through the isolated catkin workspace symlink.
fuel_script=$(readlink -f -- "${BASH_SOURCE[0]}")
fuel_root=$(cd -- "$(dirname -- "$fuel_script")/../../.." && pwd -P)
fuel_setup="$fuel_root/build/fuel_ws/devel/setup.bash"
if [[ ! -f "$fuel_setup" ]]; then
  echo "FUEL has not been built. Run: bash $fuel_root/compile_fuel.sh" >&2
  exit 1
fi
for fuel_node in prometheus_fuel/fuel_bridge exploration_manager/exploration_node plan_manage/traj_server; do
  if [[ ! -x "$fuel_root/build/fuel_ws/devel/lib/$fuel_node" ]]; then
    echo "FUEL build is incomplete ($fuel_node missing). Run: bash $fuel_root/compile_fuel.sh" >&2
    exit 1
  fi
done
unset CMAKE_PREFIX_PATH ROS_PACKAGE_PATH LD_LIBRARY_PATH PYTHONPATH
source "$fuel_setup"
export FUEL_TSP_DIR="${FUEL_TSP_DIR:-$fuel_root/build/fuel_runtime}"
mkdir -p "$FUEL_TSP_DIR"
exec "$@"
