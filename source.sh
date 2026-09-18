# Source this file from any working directory: source /path/to/project/source.sh
fuel_root=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
if [[ ! -f "$fuel_root/devel/fuel/setup.bash" ]]; then
  echo "Run bash $fuel_root/compile_fuel.sh first." >&2
  return 1
fi
# Do not inherit the old multi-workspace package/library paths.
unset CMAKE_PREFIX_PATH ROS_PACKAGE_PATH LD_LIBRARY_PATH PYTHONPATH
source "$fuel_root/devel/fuel/setup.bash"
export FUEL_TSP_DIR="${FUEL_TSP_DIR:-$fuel_root/build/fuel_runtime}"
mkdir -p "$FUEL_TSP_DIR" || return 1
export PX4_ROOT="${PX4_ROOT:-$HOME/prometheus_px4}"
export PX4_BUILD_DIR="${PX4_BUILD_DIR:-$PX4_ROOT/build/amovlab_sitl_default}"
export ROS_PACKAGE_PATH="$ROS_PACKAGE_PATH:$PX4_ROOT:$PX4_ROOT/Tools/sitl_gazebo"
export GAZEBO_PLUGIN_PATH="$fuel_root/devel/fuel/lib:$PX4_BUILD_DIR/build_gazebo:/opt/ros/noetic/lib"
export LD_LIBRARY_PATH="$LD_LIBRARY_PATH:$PX4_BUILD_DIR/build_gazebo"
fuel_models="$fuel_root/src/gazebo_simulator/gazebo_models"
export GAZEBO_MODEL_PATH="$fuel_models/uav_models:$fuel_models/sensor_models:$fuel_models/texture:/usr/share/gazebo-11/models"
unset fuel_models
