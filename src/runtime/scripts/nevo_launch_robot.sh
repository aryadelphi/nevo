#!/bin/bash
# Launcher for the full NEVO robot stack (launch_robot.launch.py).
# Used by nevo.service on boot, and can also be run manually.
# Usage: bash /home/nevo/nevo_launch_robot.sh [stack_mode] [robot_id] [ui_id]

set -e

STACK_MODE=${1:-nav}
ROBOT_ID=${2:-TFRB2}
UI_ID=${3:-TFUI2}

source /opt/ros/jazzy/setup.bash
source /home/nevo/nevo_ws/install/setup.bash

# Wait up to 30s for USB hardware to enumerate after boot.
for dev in /dev/ttyLIDAR /dev/ttyMOTOR /dev/ttyBATERAI /dev/ttyUSB0 /dev/video0; do
    for i in $(seq 1 30); do
        [ -e "$dev" ] && break
        sleep 1
    done
    if [ ! -e "$dev" ]; then
        echo "WARNING: $dev not found after 30s" >&2
    fi
done

exec ros2 launch nevo_bot launch_robot.launch.py \
    stack_mode:="$STACK_MODE" \
    robot_id:="$ROBOT_ID" \
    ui_id:="$UI_ID"
