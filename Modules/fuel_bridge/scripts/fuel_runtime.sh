#!/usr/bin/env bash
set -e
fuel_script=$(readlink -f -- "${BASH_SOURCE[0]}")
fuel_scripts=$(dirname -- "$fuel_script")
# roslaunch appends __name/__log arguments intended for ROS nodes; do not pass
# those to the nested roslaunch CLI.
fuel_args=()
for fuel_arg in "$@"; do
  [[ "$fuel_arg" == __* ]] || fuel_args+=("$fuel_arg")
done
exec "$fuel_scripts/fuel_env.sh" roslaunch prometheus_fuel fuel.launch "${fuel_args[@]}"
