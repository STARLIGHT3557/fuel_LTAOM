# Source this file from any working directory: source /path/to/project/source.sh
fuel_root=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
if [[ ! -f "$fuel_root/devel/fuel/setup.bash" ]]; then
  echo "Run bash $fuel_root/compile_fuel.sh first." >&2
  return 1
fi
# Preserve the onboard FAST-LIO and MAVROS workspace overlays.
fuel_existing_ros_package_path="${ROS_PACKAGE_PATH:-}"
source "$fuel_root/devel/fuel/setup.bash" --extend
# Catkin's ROS hook rebuilds this variable even when using --extend.
IFS=: read -r -a fuel_existing_package_paths <<< "$fuel_existing_ros_package_path"
for fuel_package_path in "${fuel_existing_package_paths[@]}"; do
  [[ -z "$fuel_package_path" ]] && continue
  case ":${ROS_PACKAGE_PATH}:" in
    *":${fuel_package_path}:"*) ;;
    *) export ROS_PACKAGE_PATH="${ROS_PACKAGE_PATH}:$fuel_package_path" ;;
  esac
done
unset fuel_existing_ros_package_path fuel_existing_package_paths fuel_package_path
export FUEL_TSP_DIR="${FUEL_TSP_DIR:-$fuel_root/build/fuel_runtime}"
mkdir -p "$FUEL_TSP_DIR" || return 1
