#!/bin/bash
# IMU calibration + drop-test recording on nevo. Start the robot stack first and
# close teleop_twist_keyboard (drive.py and teleop would fight over /cmd_vel).
#
#   ~/imu_cal/imu_cal_run.sh            # bag name imu_cal_<date_time>
#   ~/imu_cal/imu_cal_run.sh my_run
#
# 1. preflight: /imu/data_raw rate, /cmd_vel subscribers, power, confirmation
# 2. starts the bag (all topics except Kinect/camera) and a 1 Hz power log
# 3. spin sequence via ~/drive.py, each segment labelled on /imu_cal/phase:
#    still 10 s, spin CCW 3 turns, still 5 s, spin CW 3 turns, still 5 s
# 4. free-drive phase: drive with teleop, press Enter to finish
# Output: ~/bags/<name>/ (bag), ~/bags/<name>.power.log, ~/bags/<name>.bag.log
# Ctrl+C during a segment stops the robot (drive.py) and ends the run; the bag is
# always closed cleanly.

source /opt/ros/jazzy/setup.bash
source ~/nevo_ws/install/setup.bash 2>/dev/null
set -u   # after sourcing: the ROS setup scripts reference unset variables

DRIVE=${DRIVE:-$HOME/drive.py}
WZ=${WZ:-0.35}                                                 # rad/s
TURNS=${TURNS:-3}
SPIN_S=$(python3 -c "import math; print(round($TURNS * 2 * math.pi / $WZ, 2))")

NAME=${1:-imu_cal_$(date +%Y-%m-%d_%H%M%S)}
OUT=~/bags/$NAME
mkdir -p ~/bags
[ -e "$OUT" ] && { echo "$OUT already exists"; exit 1; }
[ -f "$DRIVE" ] || { echo "drive script not found: $DRIVE"; exit 1; }

power() { echo "$(vcgencmd get_throttled 2>/dev/null) $(vcgencmd pmic_read_adc EXT5V_V 2>/dev/null | awk '{print $2}')"; }
phase() { timeout 5 ros2 topic pub --once /imu_cal/phase std_msgs/msg/String "{data: $1}" >/dev/null 2>&1; }

# One drive.py segment. drive.py exits 0 on Ctrl+C too, so an early return means interrupted.
seg() {  # label vx wz seconds
  echo "[$(date +%T)] $1: vx=$2 wz=$3 for $4 s"
  phase "$1"
  local t0=$(date +%s.%N)
  python3 "$DRIVE" "$2" "$3" "$4" || return 1
  python3 -c "import sys; sys.exit(0 if $(date +%s.%N) - $t0 >= $4 else 1)"
}

echo "-- preflight"
# --no-daemon: under rmw_zenoh the ros2 daemon can answer with an empty graph
ros2 topic list --no-daemon | grep -qx /imu/data_raw || { echo "no /imu/data_raw: is the stack up?"; exit 1; }
echo "IMU rate:"; timeout 5 ros2 topic hz /imu/data_raw --no-daemon --window 100 2>/dev/null | grep average | tail -1
ros2 topic info /cmd_vel --no-daemon
echo "power now: $(power)"
echo "plan: still 10 s, CCW ${TURNS} turns @ ${WZ} rad/s (${SPIN_S} s), still 5 s, CW same, still 5 s, then free drive"
read -rp "Teleop closed, robot on level floor, clear space for in-place spins? [y/N] " ok
[ "$ok" = y ] || exit 1

set -m   # background jobs in their own process groups: Ctrl+C hits only the foreground step
ros2 bag record -o "$OUT" --all-topics --exclude-regex '.*(image|depth|camera|points|kinect).*' \
  </dev/null >"$OUT.bag.log" 2>&1 &
BAG=$!
( while :; do echo "$(date +%T) $(power)"; sleep 1; done ) </dev/null >"$OUT.power.log" 2>&1 &
PWR=$!

cleanup() {
  trap - EXIT INT
  set +m   # no job-status chatter while shutting down
  phase end
  echo "-- stopping bag"
  kill -INT $BAG 2>/dev/null; wait $BAG 2>/dev/null
  kill -- -$PWR 2>/dev/null
  echo "saved: $OUT"
  echo "power samples: $(grep -c . "$OUT.power.log"), not 0x0: $(grep -vc 'throttled=0x0' "$OUT.power.log")"
}
trap cleanup EXIT
trap 'exit 130' INT

sleep 4   # let the recorder discover topics
kill -0 $BAG 2>/dev/null || { echo "bag recorder died, see $OUT.bag.log"; exit 1; }

echo "-- spin sequence (Ctrl+C stops the robot and ends the run)"
seg still     0 0      10         &&
seg spin_ccw  0 $WZ    $SPIN_S    &&
seg still     0 0      5          &&
seg spin_cw   0 -$WZ   $SPIN_S    &&
seg still     0 0      5          || { echo "sequence interrupted"; exit 1; }

echo
echo "-- free drive: start teleop, ~60 s of straight runs, turns, hard stops, a bump"
phase free_drive
read -rp "Press Enter when done... " _
