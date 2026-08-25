#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from tf2_ros import Buffer, TransformListener
from tf2_geometry_msgs import do_transform_pose
import tf2_ros

class DetectDock(Node):
    def __init__(self):
        super().__init__('detect_dock')
        self.publisher_ = self.create_publisher(PoseStamped, 'detected_dock_pose', 10)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # Change 'camera_link' to your actual optical frame if different
        self.camera_frame = "camera_link" 
        self.tag_frame = "tag36h11:0"
        
        self.timer = self.create_timer(0.1, self.timer_callback)

    def timer_callback(self):
        try:
            # Look up transform from Camera -> Tag
            t = self.tf_buffer.lookup_transform(
                self.camera_frame,
                self.tag_frame,
                rclpy.time.Time())

            dock_pose = PoseStamped()
            dock_pose.header.stamp = self.get_clock().now().to_msg()
            dock_pose.header.frame_id = self.camera_frame
            
            # The transform IS the pose of the tag relative to camera
            dock_pose.pose.position.x = t.transform.translation.x
            dock_pose.pose.position.y = t.transform.translation.y
            dock_pose.pose.position.z = t.transform.translation.z
            dock_pose.pose.orientation = t.transform.rotation

            self.publisher_.publish(dock_pose)
            
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            pass

def main(args=None):
    rclpy.init(args=args)
    node = DetectDock()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()