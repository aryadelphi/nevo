#!/usr/bin/env bash
# Record a Tel-U Cafe survey bag on nevo: lidar + odometry + TF + full-res Kinect depth.
#
# Run in a FOREGROUND terminal on the robot (zsh suspends `ros2 bag record &`),
# with the stack already up, e.g.:
#   ros2 launch nevo_bot launch_robot.launch.py stack_mode:=mapping cloud:=false
# then:
#   bash record_cafe_survey.sh            # -> ~/bags/cafe_survey_<date_time>
#   bash record_cafe_survey.sh myname     # -> ~/bags/myname
# Stop with Ctrl+C.
set -eo pipefail

source /opt/ros/jazzy/setup.bash
[ -f ~/nevo_ws/install/setup.bash ] && source ~/nevo_ws/install/setup.bash

NAME="${1:-cafe_survey_$(date +%Y-%m-%d_%H%M)}"
OUT=~/bags/"$NAME"
MIN_FREE_GB=5

mkdir -p ~/bags
if [ -e "$OUT" ]; then
  echo "ERROR: $OUT already exists" >&2
  exit 1
fi

free_gb=$(df -BG --output=avail ~/bags | tail -1 | tr -dc '0-9')
echo "free disk: ${free_gb} GB"
if [ "$free_gb" -lt "$MIN_FREE_GB" ]; then
  echo "ERROR: need at least ${MIN_FREE_GB} GB free (about 2 GB per 10 min)" >&2
  exit 1
fi

topics_now=$(ros2 topic list)
has() { grep -qx "$1" <<<"$topics_now"; }

TOPICS=(/scan /scan_filtered /diff_cont/odom /odom_rf2o /odometry/filtered
        /imu/data_raw /tf /tf_static /depth/camera_info)
missing=()
for t in /scan /tf /depth/camera_info; do
  has "$t" || missing+=("$t")
done
if [ ${#missing[@]} -gt 0 ]; then
  echo "ERROR: required topics missing: ${missing[*]} (is the stack + Kinect up?)" >&2
  exit 1
fi

# The topics existing is not enough: on 2026-10-26 the Kinect driver hung at
# start-up and a 14 min building run recorded 0 depth frames. Check that frames
# actually flow (/points is always published while the Kinect streams).
kinect_ok() { timeout "${1:-10}" ros2 topic echo --once /points --field header.stamp.sec >/dev/null 2>&1; }
if [ "${ALLOW_NO_KINECT:-0}" != 1 ]; then
  echo -n "waiting for Kinect frames... "
  if kinect_ok 10; then echo ok; else
    echo
    echo "ERROR: Kinect topics exist but no frames arrive (driver hung)." >&2
    echo "  fix: restart the launch (Ctrl+C it, start again), or replug the Kinect USB." >&2
    echo "  2D-only on purpose: ALLOW_NO_KINECT=1 bash $0 $NAME" >&2
    exit 1
  fi
fi

# Full-res depth: PNG-compressed if the compressedDepth plugin is installed,
# otherwise raw 16UC1 with zstd bag compression (about 4x bigger on disk).
EXTRA=()
if has /depth/image_raw/compressedDepth; then
  TOPICS+=(/depth/image_raw/compressedDepth)
  echo "depth: /depth/image_raw/compressedDepth"
else
  TOPICS+=(/depth/image_raw)
  EXTRA=(--compression-mode message --compression-format zstd)
  echo "WARN: compressedDepth transport not available, recording raw depth (zstd)"
  echo "      fix: sudo apt install ros-jazzy-compressed-depth-image-transport"
fi

# Optional topics: record if present, say so if not
REC=()
for t in "${TOPICS[@]}"; do
  if has "$t"; then REC+=("$t"); else echo "note: $t not published, skipped"; fi
done

echo
echo "recording to $OUT"
printf '  %s\n' "${REC[@]}"
echo "drive slowly (0.1-0.2 m/s), pass every table at 1-2 m from both sides. Ctrl+C to stop."
echo
# watchdog: one persistent subscriber (cheap, no CLI start-up per check) that
# shouts if the Kinect really stops during the run. Ctrl+C stops it too.
if [ "${ALLOW_NO_KINECT:-0}" != 1 ]; then
  python3 - <<'PY' &
import time, rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2
rclpy.init(); n = rclpy.create_node('survey_kinect_watchdog')
last = [time.monotonic()]; warned = [False]
n.create_subscription(PointCloud2, '/points', lambda m: last.__setitem__(0, time.monotonic()),
                      qos_profile_sensor_data)
def check():
    gap = time.monotonic() - last[0]
    if gap > 5 and not warned[0]:
        print('\n\033[1;31m!!! KINECT STOPPED (no frames for %.0f s) at %s - 3D data is being lost\033[0m'
              % (gap, time.strftime('%H:%M:%S')), flush=True); warned[0] = True
    elif gap < 1 and warned[0]:
        print('\n\033[1;32mKinect frames are back at %s\033[0m' % time.strftime('%H:%M:%S'), flush=True)
        warned[0] = False
n.create_timer(1.0, check)
try:
    rclpy.spin(n)
except (KeyboardInterrupt, Exception):
    pass
PY
  WATCHDOG=$!
  trap 'kill $WATCHDOG 2>/dev/null' EXIT
fi
ros2 bag record -o "$OUT" "${EXTRA[@]}" "${REC[@]}"
