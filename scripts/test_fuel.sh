#!/usr/bin/env bash
set -eo pipefail
fuel_root=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
bash "$fuel_root/compile_fuel.sh" all fuel_frontier_boundary
source "$fuel_root/source.sh"
fuel_test_dir=$(mktemp -d /tmp/fuel-tests.XXXXXX)
export ROS_HOME="$fuel_test_dir/home"
export ROS_LOG_DIR="$fuel_test_dir/log"
export FUEL_TSP_DIR="$fuel_test_dir/tsp"
mkdir -p "$ROS_LOG_DIR" "$FUEL_TSP_DIR" "$ROS_HOME/test_results/prometheus_fuel" "$ROS_HOME/test_results/uav_controller"
rostest uav_controller position_step_ros.test
rostest prometheus_fuel frontier_boundary.test
rostest prometheus_fuel bridge.test
rostest prometheus_fuel planner.test
echo "Test results: $ROS_HOME/test_results"
