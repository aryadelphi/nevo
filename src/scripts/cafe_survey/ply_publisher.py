#!/usr/bin/env python3
"""Publish a survey cloud.ply once as a latched PointCloud2 (for RViz).

  python3 ply_publisher.py cloud.ply [--frame odom] [--topic /survey_cloud]

The cloud is in RTAB-Map's map frame, which starts equal to the robot's odom
frame, so `odom` lines it up with a replayed bag (off by the SLAM corrections,
a few cm). Uses the PLY's height colours if present.
"""
import argparse

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2, PointField


def load_ply(path):
    d = open(path, 'rb').read()
    i = d.index(b'end_header\n') + len(b'end_header\n')
    dt = [('x', '<f4'), ('y', '<f4'), ('z', '<f4')]
    rgb = b'property uchar red' in d[:i]
    if rgb:
        dt += [('r', 'u1'), ('g', 'u1'), ('b', 'u1')]
    return np.frombuffer(d[i:], dtype=dt), rgb


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('ply')
    ap.add_argument('--frame', default='odom')
    ap.add_argument('--topic', default='/survey_cloud')
    a, ros_args = ap.parse_known_args()

    v, has_rgb = load_ply(a.ply)
    out = np.zeros(len(v), dtype=[('x', '<f4'), ('y', '<f4'), ('z', '<f4'), ('rgb', '<u4')])
    out['x'], out['y'], out['z'] = v['x'], v['y'], v['z']
    if has_rgb:
        out['rgb'] = (v['r'].astype(np.uint32) << 16) | (v['g'].astype(np.uint32) << 8) | v['b']
    else:
        out['rgb'] = 0xFFFFFF

    rclpy.init(args=ros_args)
    node = Node('survey_cloud_publisher')
    qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                     durability=DurabilityPolicy.TRANSIENT_LOCAL)
    pub = node.create_publisher(PointCloud2, a.topic, qos)
    msg = PointCloud2()
    msg.header.frame_id = a.frame
    msg.header.stamp = node.get_clock().now().to_msg()
    msg.height, msg.width = 1, len(out)
    msg.fields = [PointField(name=n, offset=o, datatype=t, count=1) for n, o, t in
                  [('x', 0, PointField.FLOAT32), ('y', 4, PointField.FLOAT32),
                   ('z', 8, PointField.FLOAT32), ('rgb', 12, PointField.UINT32)]]
    msg.is_bigendian, msg.point_step = False, 16
    msg.row_step, msg.is_dense = 16 * len(out), True
    msg.data = out.tobytes()
    pub.publish(msg)
    node.get_logger().info(f'published {len(out)} points on {a.topic} in {a.frame} (latched)')
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
