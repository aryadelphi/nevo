#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry
from nav2_msgs.action import NavigateToPose
import math, time, sys

class FullDock(Node):
    def __init__(self):
        super().__init__('full_dock')
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.latest_odom = None
        self.odom_sub = self.create_subscription(Odometry, '/diff_cont/odom', self.odom_cb, 10)
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.nav_done = False
        self.nav_ok = False

    def odom_cb(self, msg):
        self.latest_odom = msg

    def nav_result_cb(self, future):
        status = future.result().status
        self.nav_ok = (status == 4)
        self.nav_done = True
        self.get_logger().info(f'Navigation {"succeeded" if self.nav_ok else f"failed (status {status})"}')

    def goal_response_cb(self, future):
        gh = future.result()
        if not gh.accepted:
            self.get_logger().error('Nav2 goal rejected!')
            self.nav_done = True
            return
        self.get_logger().info('Navigation accepted, waiting for result...')
        gh.get_result_async().add_done_callback(self.nav_result_cb)

    def run(self):
        dock_pose = (3.31854, -1.03164, -1.64472)
        xo = -0.5 * math.cos(dock_pose[2])
        yo = -0.5 * math.sin(dock_pose[2])
        sx = dock_pose[0] + xo
        sy = dock_pose[1] + yo
        syaw = dock_pose[2]  # same orientation as dock; dock_backwards reverses control direction

        self.get_logger().info(f'Navigating to staging ({sx:.2f}, {sy:.2f}, {syaw:.2f})')

        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = sx
        goal.pose.pose.position.y = sy
        goal.pose.pose.orientation.z = math.sin(syaw / 2)
        goal.pose.pose.orientation.w = math.cos(syaw / 2)

        if not self.nav_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('Nav2 not available! Is hw_nav.launch.py running?')
            return

        self.nav_done = False
        send_future = self.nav_client.send_goal_async(goal)
        send_future.add_done_callback(self.goal_response_cb)

        while rclpy.ok() and not self.nav_done:
            rclpy.spin_once(self, timeout_sec=0.1)

        if not self.nav_ok:
            return

        self.get_logger().info('At staging pose, waiting for AprilTag...')
        msg = self.wait_for_pose()
        if msg is None:
            return

        z = abs(msg.pose.position.z)
        travel = max(z - 0.05, 0.05)
        self.get_logger().info(f'Tag at z={z:.3f}m, reversing {travel:.2f}m')

        odom_start = self.wait_for_odom()
        if odom_start is None:
            return

        cmd = Twist()
        cmd.linear.x = -0.15

        traveled = 0.0
        while rclpy.ok() and traveled < travel:
            self.cmd_pub.publish(cmd)
            rclpy.spin_once(self, timeout_sec=0.05)
            if self.latest_odom is None:
                continue
            dx = self.latest_odom.pose.pose.position.x - odom_start.pose.pose.position.x
            dy = self.latest_odom.pose.pose.position.y - odom_start.pose.pose.position.y
            traveled = math.sqrt(dx*dx + dy*dy)

        self.cmd_pub.publish(Twist())
        self.get_logger().info('Docking complete!')

    def wait_for_pose(self, timeout=15.0):
        msg = None
        start = time.time()
        def cb(m):
            nonlocal msg
            if msg is None:
                msg = m
        sub = self.create_subscription(PoseStamped, '/detected_dock_pose', cb, 1)
        while rclpy.ok() and msg is None and time.time() - start < timeout:
            rclpy.spin_once(self, timeout_sec=0.1)
        self.destroy_subscription(sub)
        if msg is None:
            self.get_logger().error('No /detected_dock_pose received!')
        return msg

    def wait_for_odom(self, timeout=5.0):
        start = time.time()
        while rclpy.ok() and self.latest_odom is None and time.time() - start < timeout:
            rclpy.spin_once(self, timeout_sec=0.1)
        if self.latest_odom is None:
            self.get_logger().error('No /diff_cont/odom received!')
        return self.latest_odom

def main():
    rclpy.init()
    node = FullDock()
    node.run()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
