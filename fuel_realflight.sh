#!/usr/bin/env bash
set -eo pipefail
fuel_root=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
source "$fuel_root/source.sh"
fuel_lio_config="${FAST_LIO_CONFIG:-${FAST_LIO_ROOT:-$fuel_root/src/realflight_modules/faster-lio}/faster-lio/config/mid360.yaml}"
fuel_launch_args=()
for fuel_arg in "$@"; do
  case "$fuel_arg" in
    lio_config:=*) fuel_lio_config="${fuel_arg#lio_config:=}" ;;
    *) fuel_launch_args+=("$fuel_arg") ;;
  esac
done
for fuel_node in fuel_bridge/fuel_bridge fuel_bridge/mission_manager_node uav_controller/geometric_controller_node exploration_manager/exploration_node plan_manage/traj_server; do
  if [[ ! -x "$fuel_root/devel/fuel/lib/$fuel_node" ]]; then
    echo "Build incomplete ($fuel_node missing); run bash compile_fuel.sh." >&2
    exit 1
  fi
done
fuel_sensor_config=$(mktemp "$FUEL_TSP_DIR/sensor.XXXXXX.yaml")
python3 "$fuel_root/src/fuel_bridge/scripts/prepare_realflight.py" "$fuel_lio_config" "$fuel_sensor_config"
echo "Reusing onboard MAVROS and FAST-LIO. Start both before exploration."
exec roslaunch fuel_bridge realflight.launch sensor_config:="$fuel_sensor_config" "${fuel_launch_args[@]}"
