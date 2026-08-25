#!/usr/bin/env bash
# NEVO: load a new map YAML into the running Nav2 map_server.
# Called by nevo_robot_api (handleMapSelected) as:
#   /home/rspx1/restart_map.sh '<yaml_path>'
# Exit 0 = success, non-zero = failure.
# NOTE: no `set -u` — ROS setup.bash scripts reference unbound vars.

YAML="${1:-}"
if [ -z "$YAML" ] || [ ! -f "$YAML" ]; then
    echo "restart_map: yaml not found or empty arg: '$YAML'" >&2
    exit 2
fi

source /opt/ros/jazzy/setup.bash
source "$HOME/nevo_ws/install/setup.bash"
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTRTPS_DEFAULT_PROFILES_FILE="file://$HOME/fastdds.xml"
export ROS_DOMAIN_ID=7

# Use an rclpy client: `ros2 service call` prints a truncated repr of the
# response (huge OccupancyGrid), so its success field can't be parsed reliably.
python3 - "$YAML" <<'PYEOF'
import sys
import rclpy
from rclpy.node import Node
from nav2_msgs.srv import LoadMap

yaml_path = sys.argv[1]
rclpy.init()
node = Node('nevo_restart_map_client')
cli = node.create_client(LoadMap, '/map_server/load_map')
if not cli.wait_for_service(timeout_sec=20.0):
    print('restart_map: /map_server/load_map service unavailable', file=sys.stderr)
    sys.exit(4)

req = LoadMap.Request()
req.map_url = yaml_path
future = cli.call_async(req)
rclpy.spin_until_future_complete(node, future, timeout_sec=30.0)
res = future.result()
if res is None:
    print('restart_map: no response from map_server', file=sys.stderr)
    sys.exit(5)
if res.result != LoadMap.Response.RESULT_SUCCESS:
    print(f'restart_map: load_map failed with result={res.result}', file=sys.stderr)
    sys.exit(3)
info = res.map.info
print(f'restart_map: map loaded OK: {yaml_path} '
      f'({info.width}x{info.height} @ {info.resolution:.3f} m/cell)')
sys.exit(0)
PYEOF
RC=$?
[ $RC -ne 0 ] && exit $RC

# Stale costmap data from the previous map must go
timeout 15 ros2 service call /local_costmap/clear_local_costmap \
    nav2_msgs/srv/ClearEntireCostmap "{}" >/dev/null 2>&1
timeout 15 ros2 service call /global_costmap/clear_global_costmap \
    nav2_msgs/srv/ClearEntireCostmap "{}" >/dev/null 2>&1

exit 0
