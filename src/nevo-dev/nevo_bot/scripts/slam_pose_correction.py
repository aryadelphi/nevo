#!/usr/bin/env python3
"""RViz "2D Pose Estimate" for slam_toolbox in MAPPING mode.

slam_toolbox's mapping node ignores /initialpose. When it loses track while
mapping, this node lets you re-anchor it without losing the map built so far:

  /initialpose (map frame)
    -> /slam_toolbox/serialize_map   (current pose graph to a temp file)
    -> /slam_toolbox/deserialize_map (same graph, START_AT_GIVEN_POSE = clicked pose)

slam_toolbox then matches the next scan against the map near the clicked pose
(so a rough click is refined) and keeps mapping from there.

Only start it in mapping mode: in nav mode /initialpose belongs to AMCL.
"""
import math
import os
import tempfile
import time

import rclpy
from geometry_msgs.msg import Pose2D, PoseWithCovarianceStamped
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from slam_toolbox.srv import DeserializePoseGraph, SerializePoseGraph


class SlamPoseCorrection(Node):
    def __init__(self):
        super().__init__('slam_pose_correction')
        self.slam_ns = self.declare_parameter('slam_node', 'slam_toolbox').value
        self.min_interval = self.declare_parameter('min_interval', 3.0).value
        self.workdir = tempfile.mkdtemp(prefix='slam_pose_fix_')
        cb = ReentrantCallbackGroup()
        self.ser = self.create_client(SerializePoseGraph, f'/{self.slam_ns}/serialize_map',
                                      callback_group=cb)
        self.des = self.create_client(DeserializePoseGraph, f'/{self.slam_ns}/deserialize_map',
                                      callback_group=cb)
        self.create_subscription(PoseWithCovarianceStamped, '/initialpose', self.on_pose, 1,
                                 callback_group=cb)
        self.busy = False
        self.last = 0.0
        self.count = 0
        self.get_logger().info('ready: use RViz "2D Pose Estimate" to re-anchor slam_toolbox '
                               f'(graph snapshots in {self.workdir})')

    def call(self, client, req, timeout):
        if not client.wait_for_service(timeout_sec=2.0):
            raise RuntimeError(f'{client.srv_name} not available (is slam_toolbox in mapping mode?)')
        fut = client.call_async(req)
        t0 = time.monotonic()
        while not fut.done():
            if time.monotonic() - t0 > timeout:
                raise RuntimeError(f'{client.srv_name} timed out after {timeout:.0f} s')
            time.sleep(0.05)
        return fut.result()

    def on_pose(self, msg):
        if msg.header.frame_id.lstrip('/') not in ('map', ''):
            self.get_logger().warn(f'ignoring pose in frame "{msg.header.frame_id}", need map')
            return
        if self.busy or time.monotonic() - self.last < self.min_interval:
            self.get_logger().warn('previous correction still running, click again in a moment')
            return
        self.busy = True
        try:
            p, q = msg.pose.pose.position, msg.pose.pose.orientation
            theta = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
            self.count += 1
            # slam_toolbox appends .posegraph/.data to this name
            name = os.path.join(self.workdir, f'fix_{self.count:03d}')
            self.get_logger().info(f'correction #{self.count}: x {p.x:.2f} y {p.y:.2f} '
                                   f'yaw {math.degrees(theta):.1f} deg -> saving pose graph')
            r = self.call(self.ser, SerializePoseGraph.Request(filename=name), 60.0)
            if r.result != SerializePoseGraph.Response.RESULT_SUCCESS:
                raise RuntimeError(f'serialize_map failed (result {r.result})')
            req = DeserializePoseGraph.Request(
                filename=name, match_type=DeserializePoseGraph.Request.START_AT_GIVEN_POSE,
                initial_pose=Pose2D(x=p.x, y=p.y, theta=theta))
            self.call(self.des, req, 120.0)
            self.get_logger().info(f'correction #{self.count} done: mapping continues from the '
                                   'clicked pose (next scan is matched against the map there)')
        except Exception as e:
            self.get_logger().error(f'correction failed: {e}. Mapping continues unchanged.')
        finally:
            self.busy = False
            self.last = time.monotonic()


def main():
    rclpy.init()
    node = SlamPoseCorrection()
    ex = MultiThreadedExecutor()
    ex.add_node(node)
    try:
        ex.spin()
    except KeyboardInterrupt:
        pass
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
