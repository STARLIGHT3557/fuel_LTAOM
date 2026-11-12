#!/usr/bin/env bash
set -eo pipefail
fuel_root=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
python3 "$fuel_root/src/fuel_bridge/test/test_realflight_config.py"
bash "$fuel_root/compile_fuel.sh" all fuel_frontier_boundary flight_controller_test
source "$fuel_root/source.sh"
"$fuel_root/devel/fuel/lib/uav_controller/flight_controller_test"
fuel_test_dir=$(mktemp -d /tmp/fuel-tests.XXXXXX)
export ROS_HOME="$fuel_test_dir/home"
export ROS_LOG_DIR="$fuel_test_dir/log"
export FUEL_TSP_DIR="$fuel_test_dir/tsp"
mkdir -p "$ROS_LOG_DIR" "$FUEL_TSP_DIR" "$ROS_HOME/test_results/fuel_bridge" "$ROS_HOME/test_results/uav_controller"
# rostest runs these directly, and a checkout from Windows does not mark them executable.
chmod +x "$fuel_root"/src/fuel_bridge/test/test_*.py
rostest uav_controller position_step_ros.test
rostest fuel_bridge frontier_boundary.test
rostest fuel_bridge bridge.test
rostest fuel_bridge realflight_bridge.test
rostest fuel_bridge planner.test
rostest fuel_bridge planner_overrun.test
rostest fuel_bridge exploration_timeouts.test
rostest fuel_bridge return_recovery.test
rostest fuel_bridge return_planner.test
rostest fuel_bridge return_takeover.test
rostest fuel_bridge return_observe.test
echo "Test results: $ROS_HOME/test_results"
