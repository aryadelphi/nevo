#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import tf2_ros
import math, time

class TagTracker(Node):
    def __init__(self):
        super().__init__('tag_tracker')
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

    def get_tag(self):
        try:
            t = self.tf_buffer.lookup_transform(
                'c310_camera_link_optical', 'tag36h11:5', rclpy.time.Time())
            return t.transform.translation
        except Exception:
            return None

    def run(self):
        self.get_logger().info('Tag tracker running — rotate to face tag (Ctrl+C to stop)')
        k_angular = 0.8
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.05)
            tag = self.get_tag()
            if tag is None:
                print('\rno tag              ', end='', flush=True)
                self.cmd_pub.publish(Twist())
                continue
            x = tag.x
            z = abs(tag.z)
            print(f'\rz={z:.3f}  x={x:.3f}  ang={-x*k_angular:.3f}  ', end='', flush=True)
            cmd = Twist()
            cmd.angular.z = -x * k_angular
            self.cmd_pub.publish(cmd)
            time.sleep(0.05)

def main():
    rclpy.init()
    node = TagTracker()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    node.cmd_pub.publish(Twist())
    rclpy.shutdown()

if __name__ == '__main__':
    main()
