#!/usr/bin/env bash
set -e
# roslaunch appends __name/__log remappings, which are not PX4 arguments.
exec "${PX4_BUILD_DIR:?Source source.sh}/bin/px4" "${PX4_ROOT:?}/ROMFS/px4fmu_common" \
  -s etc/init.d-posix/rcS -i "$1" -w "sitl_fuel_$1" -d
