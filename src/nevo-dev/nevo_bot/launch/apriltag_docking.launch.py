import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    
    # Paths
    # IMPORTANT: Update 'nevo_bot' to whatever package name holds your config folder
    config_dir = os.path.join(get_package_share_directory('nevo_bot'), 'config')
    tag_params_file = os.path.join(config_dir, 'tags.yaml')
    
    # Path to the calibration file we generated earlier
    # Make sure this matches where you scp'd it on the Pi
    camera_info_url = 'file:///home/rspx1/.ros/camera_info/c310_calibration.yaml'

    # Camera Node
    camera_node = Node(
        package='v4l2_camera',
        executable='v4l2_camera_node',
        name='v4l2_camera',
        parameters=[{
            'video_device': '/dev/video0',
            'image_size': [640, 480],
            'camera_info_url': camera_info_url,
            'camera_frame_id': 'c310_camera_link_optical' # Must match your URDF!
        }]
    )

    # AprilTag Node
    apriltag_node = Node(
        package='apriltag_ros',
        executable='apriltag_node',
        name='apriltag',
        parameters=[tag_params_file],
        remappings=[
            ('image_rect', '/image_raw'), # AprilTag expects a rectified image topic by default
            ('camera_info', '/camera_info')
        ]
    )

    return LaunchDescription([
        camera_node,
        apriltag_node
    ])
