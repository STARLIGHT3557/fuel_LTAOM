#!/usr/bin/env bash
# Convenience wrapper for tests and planner-only launches in the same workspace.
set -e
fuel_script=$(readlink -f -- "${BASH_SOURCE[0]}")
fuel_root=$(cd -- "$(dirname -- "$fuel_script")/../../.." && pwd -P)
source "$fuel_root/source.sh"
exec "$@"
