#!/usr/bin/env bash
# Handheld map-reuse test: LiDAR + LIO + occupancy grid + RViz.
# No MAVROS, no flight controller, no exploration. Pose comes from LIO itself,
# so the drone can be carried by hand.
set -eo pipefail
fuel_root=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
source "$fuel_root/source.sh"

fuel_lio_config="${FAST_LIO_CONFIG:-${FAST_LIO_ROOT:-$fuel_root/src/realflight_modules/faster-lio}/faster-lio/config/mid360.yaml}"
fuel_launch_args=()
fuel_lio_args=()
for fuel_arg in "$@"; do
  case "$fuel_arg" in
    lio_config:=*) fuel_lio_config="${fuel_arg#lio_config:=}" ;;
    lio_arg:=*) fuel_lio_args+=("${fuel_arg#lio_arg:=}") ;;
    *) fuel_launch_args+=("$fuel_arg") ;;
  esac
done

for fuel_node in prometheus_fuel/fuel_bridge prometheus_fuel/map_reuse_node \
                 faster_lio/run_mapping_online; do
  if [[ ! -x "$fuel_root/devel/fuel/lib/$fuel_node" ]]; then
    echo "Build incomplete ($fuel_node missing); run bash compile_fuel.sh." >&2
    exit 1
  fi
done

fuel_sensor_config=$(mktemp "$FUEL_TSP_DIR/sensor.XXXXXX.yaml")
python3 "$fuel_root/src/fuel_bridge/scripts/prepare_realflight.py" \
  "$fuel_lio_config" "$fuel_sensor_config"

# Stop every process this script started, in one go.
trap 'kill $(jobs -pr) 2>/dev/null || true; wait' EXIT
trap 'exit 0' INT TERM

# The LIO launch also starts the MID360 driver.
roslaunch faster_lio mapping_mid360.launch "${fuel_lio_args[@]}" &

# Give LIO a moment to publish its first odometry before the map node needs a
# pose, so the grid starts filling immediately instead of warning.
(
  sleep 3
  roslaunch prometheus_fuel map_reuse.launch sensor_config:="$fuel_sensor_config" \
    "${fuel_launch_args[@]}"
) &

wait -n
