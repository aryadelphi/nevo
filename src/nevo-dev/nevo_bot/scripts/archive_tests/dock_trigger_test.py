#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from opennav_docking_msgs.action import DockRobot

class DockTriggerTest(Node):
    def __init__(self):
        super().__init__('dock_trigger_test')
        self.client = ActionClient(self, DockRobot, 'dock_robot')

    def send_goal(self):
        self.get_logger().info('Waiting for docking action server...')
        if not self.client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error('Docking server not available!')
            return

        goal = DockRobot.Goal()
        goal.use_dock_id = True
        goal.dock_id = 'home_dock'
        goal.navigate_to_staging_pose = True

        self.get_logger().info('Sending dock goal...')
        future = self.client.send_goal_async(goal, feedback_callback=self.feedback)
        future.add_done_callback(self.goal_response)

    def feedback(self, msg):
        fb = msg.feedback
        self.get_logger().info(f'Docking state: {fb.state}  retries: {fb.num_retries}')

    def goal_response(self, future):
        gh = future.result()
        if not gh.accepted:
            self.get_logger().error('Goal rejected!')
            rclpy.shutdown()
            return
        self.get_logger().info('Goal accepted!')
        gh.get_result_async().add_done_callback(self.result)

    def result(self, future):
        status = future.result().status
        self.get_logger().info(f'Docking finished with status: {status}')
        rclpy.shutdown()

def main():
    rclpy.init()
    node = DockTriggerTest()
    node.send_goal()
    rclpy.spin(node)

if __name__ == '__main__':
    main()
