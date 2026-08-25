#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import tf2_ros
import math

class TagMonitor(Node):
    def __init__(self):
        super().__init__('tag_monitor')
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

    def run(self):
        self.get_logger().info('Monitoring AprilTag distance (Ctrl+C to stop)')
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)
            try:
                t = self.tf_buffer.lookup_transform(
                    'c310_camera_link_optical', 'tag36h11:6', rclpy.time.Time())
                x = t.transform.translation.x
                y = t.transform.translation.y
                z = t.transform.translation.z
                dist = math.sqrt(x*x + y*y + z*z)
                print(f'\rtag dist: {dist:.3f}m  (x:{x:.3f} y:{y:.3f} z:{z:.3f})  ', end='', flush=True)
            except Exception:
                print('\rno tag detected           ', end='', flush=True)

def main():
    rclpy.init()
    node = TagMonitor()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()

if __name__ == '__main__':
    main()
