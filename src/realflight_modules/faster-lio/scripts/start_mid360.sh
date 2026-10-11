#!/usr/bin/env bash
set -eo pipefail
repo_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
workspace_dir="$(cd -- "$repo_dir/../../.." && pwd)"
source /opt/ros/noetic/setup.bash
if [ ! -f "$workspace_dir/devel/fuel/setup.bash" ]; then
    printf 'Workspace is not built: %s\n' "$workspace_dir" >&2
    exit 1
fi
source "$workspace_dir/source.sh"
exec roslaunch faster_lio mapping_mid360.launch "$@"
