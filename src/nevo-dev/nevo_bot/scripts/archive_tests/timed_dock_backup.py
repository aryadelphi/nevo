#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry
import math

class OdomDockBackup(Node):
    def __init__(self):
        super().__init__('timed_dock_backup')
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.latest_odom = None
        self.odom_sub = self.create_subscription(Odometry, '/diff_cont/odom', self.odom_cb, 10)

    def odom_cb(self, msg):
        self.latest_odom = msg

    def run(self):
        self.get_logger().info('Waiting for /detected_dock_pose...')
        msg = self.wait_for_pose()
        if msg is None:
            return

        z = msg.pose.position.z
        distance = abs(z) if z != 0 else 0.28
        stop_before = 0.05
        travel = max(distance - stop_before, 0.05)

        self.get_logger().info(f'Tag at z={z:.3f}m, need to travel {travel:.2f}m')

        odom_start = self.wait_for_odom()
        if odom_start is None:
            return

        cmd = Twist()
        cmd.linear.x = -0.15

        traveled = 0.0
        last_print = 0.0
        while rclpy.ok() and traveled < travel:
            self.cmd_pub.publish(cmd)
            rclpy.spin_once(self, timeout_sec=0.05)
            current = self.latest_odom
            if current is None:
                continue
            dx = current.pose.pose.position.x - odom_start.pose.pose.position.x
            dy = current.pose.pose.position.y - odom_start.pose.pose.position.y
            traveled = math.sqrt(dx*dx + dy*dy)
            if traveled - last_print > 0.05:
                self.get_logger().info(f'Traveled {traveled:.2f}m so far')
                last_print = traveled

        self.stop()

    def wait_for_pose(self, timeout=10.0):
        msg = None
        start = self.get_clock().now()

        def cb(m):
            nonlocal msg
            if msg is None:
                msg = m

        sub = self.create_subscription(PoseStamped, '/detected_dock_pose', cb, 1)
        while rclpy.ok() and msg is None and (self.get_clock().now() - start).nanoseconds / 1e9 < timeout:
            rclpy.spin_once(self, timeout_sec=0.1)
        self.destroy_subscription(sub)
        if msg is None:
            self.get_logger().error('No /detected_dock_pose received within timeout!')
        return msg

    def wait_for_odom(self, timeout=5.0):
        start = self.get_clock().now()
        while rclpy.ok() and self.latest_odom is None and (self.get_clock().now() - start).nanoseconds / 1e9 < timeout:
            rclpy.spin_once(self, timeout_sec=0.1)
        if self.latest_odom is None:
            self.get_logger().error('No /diff_cont/odom received!')
        return self.latest_odom

    def stop(self):
        self.cmd_pub.publish(Twist())
        self.get_logger().info('Stopped')
        rclpy.shutdown()

def main():
    rclpy.init()
    node = OdomDockBackup()
    node.run()

if __name__ == '__main__':
    main()
