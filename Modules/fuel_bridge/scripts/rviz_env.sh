#!/usr/bin/env bash
# RViz needs the original workspace's mesh resources. FUEL nodes keep using
# their separate environment so EGO and FUEL libraries are never mixed.
set -e
# roslaunch finds this package through build/fuel_ws/src/prometheus_fuel.
# Resolve that symlink before walking up to the original workspace.
fuel_script=$(readlink -f -- "${BASH_SOURCE[0]}")
fuel_root=$(cd -- "$(dirname -- "$fuel_script")/../../.." && pwd -P)
unset CMAKE_PREFIX_PATH ROS_PACKAGE_PATH LD_LIBRARY_PATH PYTHONPATH
source "$fuel_root/devel/setup.bash"
exec "$@"
