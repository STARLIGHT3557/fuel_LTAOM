#!/usr/bin/env bash
set -euo pipefail

fuel_root=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
pcd_file="${1:-$HOME/fuel_sim_logs/map_prior/merged_map.pcd}"
frame_id="${2:-map}"

if [[ $# -gt 2 ]]; then
  echo "Usage: bash $0 [path/to/map.pcd] [frame_id]" >&2
  exit 2
fi
if [[ ! -f /opt/ros/noetic/setup.bash ]]; then
  echo "ROS Noetic not found at /opt/ros/noetic/setup.bash" >&2
  exit 1
fi
if [[ ! -f "$fuel_root/devel/fuel/setup.bash" ]]; then
  echo "Workspace is not built; run: bash $fuel_root/compile_fuel.sh" >&2
  exit 1
fi
if [[ ! -f "$pcd_file" ]]; then
  echo "PCD file not found: $pcd_file" >&2
  echo "Usage: bash $0 [path/to/map.pcd] [frame_id]" >&2
  exit 1
fi
pcd_file=$(realpath -- "$pcd_file")
if [[ -z "$frame_id" ]]; then
  echo "frame_id must not be empty" >&2
  exit 2
fi

source /opt/ros/noetic/setup.bash
source "$fuel_root/source.sh"
exec roslaunch fuel_bridge saved_pcd_viewer.launch \
  "pcd_file:=$pcd_file" "frame_id:=$frame_id"
