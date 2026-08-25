#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist
import math

class AprilTagAlignTest(Node):
    def __init__(self):
        super().__init__('apriltag_align_test')
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.sub = self.create_subscription(
            PoseStamped, 'detected_dock_pose', self.pose_callback, 10)
        self.get_logger().info('Align test running — driving toward detected tag')

    def pose_callback(self, msg):
        twist = Twist()
        x = msg.pose.position.x
        y = msg.pose.position.y
        _, _, yaw = self._euler_from_quaternion(msg.pose.orientation)

        linear_gain = 0.3
        lateral_gain = 0.4
        angular_gain = 0.5
        min_speed = 0.05

        twist.linear.x = -abs(x) * linear_gain
        if abs(twist.linear.x) < min_speed and abs(x) > 0.01:
            twist.linear.x = -min_speed

        twist.linear.y = -y * lateral_gain

        twist.angular.z = -yaw * angular_gain

        self.cmd_pub.publish(twist)
        self.get_logger().info(
            f'tag: x={x:.3f} y={y:.3f} yaw={yaw:.3f}  cmd: v={twist.linear.x:.3f} w={twist.angular.z:.3f}')

    @staticmethod
    def _euler_from_quaternion(q):
        sinr_cosp = 2.0 * (q.w * q.x + q.y * q.z)
        cosr_cosp = 1.0 - 2.0 * (q.x * q.x + q.y * q.y)
        roll = math.atan2(sinr_cosp, cosr_cosp)
        sinp = 2.0 * (q.w * q.y - q.z * q.x)
        pitch = math.asin(sinp)
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        return roll, pitch, yaw

def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(AprilTagAlignTest())
    rclpy.shutdown()

if __name__ == '__main__':
    main()
