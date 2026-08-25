#!/bin/bash
# NEVO stack auto-restart - spawned by robot_api_node when the cloud UI toggles
# between navigation and mapping. Kills ANY running NEVO stack (launch_robot
# or hw_* / systemd) and relaunches launch_robot in the requested mode inside a
# detached tmux session. All output is logged to ~/nevo_restart.log so a failed
# swap leaves a trace instead of silently hanging the UI.
MODE="$1"
[ -z "$MODE" ] && MODE="nav"

LOCKFILE="/tmp/nevo_restart.lock"
LOGFILE="$HOME/nevo_restart.log"
SESSION="nevo_robot"
LOG() { echo "[$(date '+%F %T')] $*" >> "$LOGFILE"; }

# --- stale lock cleanup: only honor a lock owned by a LIVE process ----------
if [ -f "$LOCKFILE" ]; then
  OLD_PID="$(cat "$LOCKFILE" 2>/dev/null)"
  if [ -n "$OLD_PID" ] && kill -0 "$OLD_PID" 2>/dev/null; then
    LOG "another restart already in progress (pid=$OLD_PID), skipping"
    exit 0
  fi
  LOG "removing stale lock from dead pid=$OLD_PID"
  rm -f "$LOCKFILE"
fi
echo $$ > "$LOCKFILE"
trap 'rm -f "$LOCKFILE"' EXIT

LOG "=== NEVO stack restart -> mode=$MODE ==="

# Kill a process AND all of its descendants (depth-first), so that killing
# `ros2 launch` cannot leave orphaned nodes (bt_navigator, smoother_server,
# rplidar holding /dev/ttyLIDAR, etc.) running behind a new stack.
killtree() {
  local pid=$1 sig=${2:-TERM}
  local -a kids
  mapfile -t kids < <(pgrep -P "$pid" 2>/dev/null)
  local k
  for k in "${kids[@]}"; do
    killtree "$k" "$sig"
  done
  kill -"$sig" "$pid" 2>/dev/null
}

# --- stop systemd-launched stacks too (services are disabled but may be used) ---
for svc in nevo.service nevo-mapping.service; do
  if systemctl is-active --quiet "$svc" 2>/dev/null; then
    LOG "stopping systemd service: $svc"
    sudo -n systemctl stop "$svc" 2>>"$LOGFILE" \
      || LOG "WARN: could not stop $svc (ret=$?)"
  fi
done

# Let the requesting robot_api_node flush its ACK to the cloud first
sleep 3

# 1) Cancel any pending navigation goals before killing processes.
#    Without this, the planner_server keeps trying to plan to a stale goal
#    from the previous session, causing TF extrapolation errors.
LOG "cancelling pending navigation goals"
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTRTPS_DEFAULT_PROFILES_FILE="file://$HOME/fastdds.xml"
export ROS_DOMAIN_ID=7
source /opt/ros/jazzy/setup.bash 2>/dev/null || true
source /home/rspx1/nevo_ws/install/setup.bash 2>/dev/null || true
timeout 5 python3 -c "
import rclpy
from rclpy.node import Node
from action_msgs.srv import CancelGoal
rclpy.init(args=None)
node = rclpy.create_node('goal_canceller')
for action in ['/navigate_to_pose', '/navigate_through_poses', '/follow_path', '/follow_waypoints']:
    cli = node.create_client(CancelGoal, action + '/_action/cancel_goal')
    if cli.wait_for_service(timeout_sec=1.0):
        req = CancelGoal.Request()
        future = cli.call_async(req)
        rclpy.spin_until_future_complete(node, future, timeout_sec=2.0)
        print(f'cancelled {action}')
    else:
        print(f'{action} not available')
node.destroy_node()
rclpy.shutdown()
" 2>&1 && LOG "goals cancelled" || LOG "WARN: goal cancellation skipped"
unset RMW_IMPLEMENTATION FASTRTPS_DEFAULT_PROFILES_FILE ROS_DOMAIN_ID

# 1) Graceful stop of ANY running NEVO launch AND its whole process tree
#    (matches both launch_robot.launch.py and hw_mapping/hw_nav variants)
for old_launch in $(pgrep -f "ros2 launch nevo_bot" 2>/dev/null); do
  LOG "terminating launch tree pid=$old_launch"
  killtree "$old_launch" TERM
done
sleep 4

# 3b) Hard-kill anything that survived (escalate whole tree, then sweep patterns)
for old_launch in $(pgrep -f "ros2 launch nevo_bot" 2>/dev/null); do
  killtree "$old_launch" KILL
done
sleep 2
for pat in robot_api_node ws_bridge_node sensor_manager_node slam_supervisor_node \
           position_sensor_node slam_toolbox map_server amcl controller_server \
           planner_server controller_manager ros2_control_node safety_fail_node \
           explore docking_server v4l2_camera apriltag_node kinect_ros2_node \
           ekf_node twist_mux scan_to_scan_filter diagnostic_node \
           bt_navigator behavior_server smoother_server waypoint_follower \
           velocity_smoother ina226_bridge tag_pose_bridge rectify_node \
           robot_state_publisher joint_state_broadcaster diff_cont \
           kinect_tf_publisher static_transform_publisher cloud_upload \
           rplidar_node rviz teleop_twist_keyboard spawner; do
  pkill -9 -f "$pat" 2>/dev/null
done

# 3) Wait (bounded) until the old stack's session holders are really gone.
#    Without this, the new ws_bridge connects while the old TFRB2 session is
#    still bound -> cloud rejects it with DUPLICATE_ROBOT_ID and the swap hangs.
for i in $(seq 1 10); do
  if ! pgrep -f "ws_bridge_node" >/dev/null 2>&1 &&
     ! pgrep -f "robot_api_node" >/dev/null 2>&1; then
    break
  fi
  sleep 2
done
if pgrep -f "ws_bridge_node" >/dev/null 2>&1; then
  LOG "WARN: old ws_bridge_node still alive after teardown, continuing anyway"
fi
# DDS / cloud session cleanup grace
sleep 5

# 3b) Reset serial ports so the new rplidar/motor nodes can open them cleanly.
#     The old rplidar process may leave the port in a non-reset state.
for port in /dev/ttyLIDAR /dev/ttyMOTOR /dev/ttyBATERAI; do
  if [ -e "$port" ]; then
    stty -F "$port" raw speed 115200 -echo 2>/dev/null && LOG "serial $port reset OK" || LOG "WARN: serial $port reset failed"
  fi
done
sleep 3

# 4) Relaunch detached in a tmux session (fall back to nohup if tmux missing)
LOG "relaunching: mode=$MODE"
if command -v tmux >/dev/null 2>&1; then
  tmux kill-session -t "$SESSION" 2>/dev/null
  tmux new-session -d -s "$SESSION" -x 200 -y 50 "bash -c 'export RMW_IMPLEMENTATION=rmw_fastrtps_cpp && export FASTRTPS_DEFAULT_PROFILES_FILE=file://$HOME/fastdds.xml && export ROS_DOMAIN_ID=7 && source /opt/ros/jazzy/setup.bash && source /home/rspx1/nevo_ws/install/setup.bash && exec ros2 launch nevo_bot launch_robot.launch.py stack_mode:=$MODE robot_id:=TFRB2 ui_id:=TFUI2'"
  LOG "launch running in tmux session '$SESSION' (attach: tmux attach -t $SESSION)"
else
  nohup bash -c "export RMW_IMPLEMENTATION=rmw_fastrtps_cpp && export FASTRTPS_DEFAULT_PROFILES_FILE=file://\$HOME/fastdds.xml && export ROS_DOMAIN_ID=7 && source /opt/ros/jazzy/setup.bash && source /home/rspx1/nevo_ws/install/setup.bash && exec ros2 launch nevo_bot launch_robot.launch.py stack_mode:=$MODE robot_id:=TFRB2 ui_id:=TFUI2" >> "$LOGFILE" 2>&1 &
  LOG "launch running detached (log: $LOGFILE)"
fi

# 5) Background health check: report to the log whether the new stack came up.
#    (robot_api_node publishing READY to the cloud is the real completion signal;
#     this just gives a local trace so a failed swap is debuggable.)
(
  sleep 50
  if pgrep -f "robot_api_node" >/dev/null 2>&1; then
    LOG "health check: robot_api_node is UP (mode=$MODE)"
  else
    LOG "health check: FAILURE - robot_api_node NOT running 50s after relaunch (mode=$MODE)"
  fi
) >> "$LOGFILE" 2>&1 &

LOG "=== nevo_restart: done ==="
exit 0
