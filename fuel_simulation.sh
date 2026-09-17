#!/usr/bin/env bash
set -eo pipefail
fuel_root=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
if [[ ! -f "$fuel_root/build/fuel_ws/devel/setup.bash" ]]; then
  echo "Run bash $fuel_root/compile_fuel.sh first." >&2
  exit 1
fi
"$fuel_root/Modules/fuel_bridge/scripts/fuel_env.sh" true
source "$fuel_root/source.sh"
export ROS_PACKAGE_PATH="$fuel_root/Modules/fuel_bridge:$ROS_PACKAGE_PATH"
# The outer launch uses the original Gazebo/controller environment. Only the
# nested FUEL launch switches to its isolated environment.
exec roslaunch prometheus_fuel simulation.launch "$@"
