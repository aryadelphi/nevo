#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import Twist
from nav2_msgs.action import NavigateToPose
import math, time

class SimpleDock(Node):
    def __init__(self):
        super().__init__('simple_dock')
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.nav_done = False
        self.nav_ok = False

    def nav_result_cb(self, future):
        self.nav_ok = (future.result().status == 4)
        self.nav_done = True

    def goal_response_cb(self, future):
        gh = future.result()
        if not gh.accepted:
            self.get_logger().error('Nav2 goal rejected!')
            self.nav_done = True
            return
        self.get_logger().info('Navigating to staging...')
        gh.get_result_async().add_done_callback(self.nav_result_cb)

    def run(self):
        dock_pose = (3.31854, -1.03164, -1.64472)
        xo = -0.5 * math.cos(dock_pose[2])
        yo = -0.5 * math.sin(dock_pose[2])

        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = dock_pose[0] + xo
        goal.pose.pose.position.y = dock_pose[1] + yo
        goal.pose.pose.orientation.z = math.sin(dock_pose[2] / 2)
        goal.pose.pose.orientation.w = math.cos(dock_pose[2] / 2)

        for i in range(40):
            rclpy.spin_once(self, timeout_sec=0.5)
            if self.nav_client.server_is_ready():
                self.get_logger().info(f'Nav2 ready after {i+1} tries')
                break
        else:
            self.get_logger().error('Nav2 not found!')
            return

        self.nav_done = False
        f = self.nav_client.send_goal_async(goal)
        f.add_done_callback(self.goal_response_cb)
        while rclpy.ok() and not self.nav_done:
            rclpy.spin_once(self, timeout_sec=0.1)
        if not self.nav_ok:
            return

        self.get_logger().info('Reversing for 2.5 seconds...')
        cmd = Twist()
        cmd.linear.x = -0.15
        start = time.time()
        while rclpy.ok() and time.time() - start < 2.5:
            self.cmd_pub.publish(cmd)
            rclpy.spin_once(self, timeout_sec=0.05)
        self.cmd_pub.publish(Twist())
        self.get_logger().info('Docking complete!')

def main():
    rclpy.init()
    node = SimpleDock()
    node.run()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
