import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from action_msgs.msg import GoalStatus

from sensor_msgs.msg import BatteryState
from nav2_msgs.action import NavigateToPose
from opennav_docking_msgs.action import DockRobot

class AutoDockingMockup(Node):
    def __init__(self):
        super().__init__('auto_docking_mockup')

        self.battery_sub = self.create_subscription(
            BatteryState, 'power_state', self.battery_callback, 10)

        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.dock_client = ActionClient(self, DockRobot, 'dock_robot')

        self.docking_phase = 0  # 0=Idle, 1=Clearing, 2=Docking, 3=Done

        # Phase 1: Safe clearance point (away from obstacles)
        self.home_x = 0.602491
        self.home_y = 0.354647
        self.home_z_orient = 0.0112424
        self.home_w_orient = 0.999937

    def battery_callback(self, msg):
        if msg.percentage <= 0.20 and self.docking_phase == 0:
            self.get_logger().warn(
                f"Battery Critical: {msg.percentage * 100:.1f}%. Moving to safe clearance point.")
            self.docking_phase = 1
            self.send_nav_goal()

    def send_nav_goal(self):
        self.get_logger().info('Waiting for Nav2 action server...')
        self.nav_client.wait_for_server()

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.pose.position.x = self.home_x
        goal_msg.pose.pose.position.y = self.home_y
        goal_msg.pose.pose.orientation.z = self.home_z_orient
        goal_msg.pose.pose.orientation.w = self.home_w_orient

        self.get_logger().info('Navigating to safe clearance point...')
        future = self.nav_client.send_goal_async(goal_msg)
        future.add_done_callback(self.nav_response_callback)

    def nav_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Clearance nav goal rejected!')
            return
        self.get_logger().info('Clearance goal accepted, traveling...')
        goal_handle.get_result_async().add_done_callback(self.nav_result_callback)

    def nav_result_callback(self, future):
        status = future.result().status
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info('Reached clearance point. Handing off to docking server...')
            self.docking_phase = 2
            # Small stabilization delay then trigger dock
            self.create_timer(1.0, self.trigger_docking_server)
        else:
            self.get_logger().error(f'Clearance navigation failed! Status: {status}')
            self.docking_phase = 0  # reset so it can retry

    def trigger_docking_server(self):
        # Timer fires once then we cancel it
        self.send_dock_goal()

    def send_dock_goal(self):
        self.get_logger().info('Waiting for docking action server...')
        self.dock_client.wait_for_server()

        goal_msg = DockRobot.Goal()
        goal_msg.use_dock_id = True                  # use registered dock from yaml
        goal_msg.dock_id = 'home_dock'               # matches docking.yaml
        goal_msg.navigate_to_staging_pose = True     # server drives to staging, THEN aligns

        self.get_logger().info('Sending dock goal: navigating to staging then aligning...')
        future = self.dock_client.send_goal_async(
            goal_msg,
            feedback_callback=self.dock_feedback_callback)
        future.add_done_callback(self.dock_response_callback)

    def dock_feedback_callback(self, feedback_msg):
        fb = feedback_msg.feedback
        self.get_logger().info(f'Docking state: {fb.state}  |  Retries: {fb.num_retries}')

    def dock_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Docking server rejected the goal!')
            self.docking_phase = 0
            return
        self.get_logger().info('Docking goal accepted. Precision sequence running...')
        goal_handle.get_result_async().add_done_callback(self.dock_result_callback)

    def dock_result_callback(self, future):
        status = future.result().status
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info('NEVO Bot successfully docked! Charging active.')
            self.docking_phase = 3
        else:
            self.get_logger().error(f'Docking failed! Status: {status}. Will retry on next battery check.')
            self.docking_phase = 0  # allow retry

def main(args=None):
    rclpy.init(args=args)
    node = AutoDockingMockup()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("Shutting down...")
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()

if __name__ == '__main__':
    main()