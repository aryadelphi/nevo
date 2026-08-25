#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
import tf2_ros

class TagPoseBridge(Node):
    def __init__(self):
        super().__init__('tag_pose_bridge')
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.pub = self.create_publisher(PoseStamped, 'detected_dock_pose', 10)
        self.create_timer(0.1, self.publish_pose)
        self.get_logger().info('Tag pose bridge running — publishing tag36h11:5 as detected_dock_pose')

    def publish_pose(self):
        try:
            t = self.tf_buffer.lookup_transform(
                'c310_camera_link_optical', 'tag36h11:5', rclpy.time.Time())
            pose = PoseStamped()
            pose.header.stamp = self.get_clock().now().to_msg()
            pose.header.frame_id = 'c310_camera_link_optical'
            pose.pose.position.x = t.transform.translation.x
            pose.pose.position.y = t.transform.translation.y
            pose.pose.position.z = t.transform.translation.z
            pose.pose.orientation = t.transform.rotation
            self.pub.publish(pose)
        except Exception:
            pass

def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(TagPoseBridge())

if __name__ == '__main__':
    main()
