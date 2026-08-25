#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from opennav_docking_msgs.action import DockRobot
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist  # <-- Added for manual velocity commands

class Nav2DockCoordinator(Node):
    def __init__(self):
        super().__init__('nav2_dock_coordinator')
        
        self.robot_state = 'IDLE'
        self.goal_handle = None
        
        # Keep the Nav2 client for the complex visual approach
        self.dock_client = ActionClient(self, DockRobot, 'dock_robot')
        
        # Replace the undock client with a simple direct motor command
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)

        self.state_sub = self.create_subscription(
            Bool,
            'dock_state',
            self.dock_state_callback,
            10
        )

    def send_dock_goal(self, dock_id="home_dock"):
        self.get_logger().info("Waiting for 'DockRobot' action server...")
        self.dock_client.wait_for_server()

        goal_msg = DockRobot.Goal()
        goal_msg.use_dock_id = True
        goal_msg.dock_id = dock_id
        goal_msg.navigate_to_staging_pose = True

        self.get_logger().info(f"Sending request to dock at: {dock_id}")
        self.robot_state = 'NAVIGATING_TO_DOCK'
        
        send_goal_future = self.dock_client.send_goal_async(
            goal_msg, feedback_callback=self.dock_feedback_callback
        )
        send_goal_future.add_done_callback(self.dock_response_callback)

    def dock_feedback_callback(self, feedback_msg):
        self.get_logger().info(f"Docking Inbound... State: {feedback_msg.feedback.state}")

    def dock_response_callback(self, future):
        self.goal_handle = future.result()
        if not self.goal_handle.accepted:
            self.get_logger().error("Docking request was rejected!")
            self.robot_state = 'IDLE'
            return
        
        result_future = self.goal_handle.get_result_async()
        result_future.add_done_callback(self.dock_result_callback)

    def dock_state_callback(self, msg):
        if self.robot_state == 'NAVIGATING_TO_DOCK' and msg.data is True:
            self.get_logger().info("💥 Limit switch triggered! Canceling Nav2 goal to freeze motors...")
            self.robot_state = 'DOCKED'
            
            if self.goal_handle is not None:
                self.goal_handle.cancel_goal_async()
            
            self.get_logger().info("Successfully docked. Standing by for 20s test delay...")

        elif self.robot_state == 'DOCKED' and msg.data is False:
            self.get_logger().info("🔄 Dock state turned False! Commencing manual forward push...")
            self.robot_state = 'UNDOCKING'
            self.send_undock_goal()

    def dock_result_callback(self, future):
        if self.robot_state == 'DOCKED':
            return
        result = future.result().result
        if not result.success:
            self.get_logger().error(f"Docking failed: {result.error_code}")
            self.robot_state = 'IDLE'
            rclpy.shutdown()

    def send_undock_goal(self):
        # Instead of a framework action, use a timer to drive straight forward
        self.undock_ticks = 0
        self.undock_timer = self.create_timer(0.1, self.undock_timer_callback)

    def undock_timer_callback(self):
        msg = Twist()
        # 30 ticks at 0.1s interval = 3.0 seconds of driving straight forward
        if self.undock_ticks < 30:
            msg.linear.x = 0.15   # Push forward slowly (adjust if needed)
            msg.angular.z = 0.0   # Zero turning guaranteed
            self.cmd_vel_pub.publish(msg)
            self.undock_ticks += 1
        else:
            # Time is up, stop the motors
            msg.linear.x = 0.0
            self.cmd_vel_pub.publish(msg)
            self.undock_timer.cancel()
            self.get_logger().info("SUCCESS: Robot cleanly cleared the dock manually.")
            self.robot_state = 'IDLE'
            rclpy.shutdown()

def main():
    rclpy.init()
    coordinator = Nav2DockCoordinator()
    coordinator.send_dock_goal("home_dock")
    rclpy.spin(coordinator)

if __name__ == '__main__':
    main()
