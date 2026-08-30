import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    pkg_dir = get_package_share_directory('nevo_bot')

    camera_node = Node(
        package='v4l2_camera',
        executable='v4l2_camera_node',
        remappings=[
            ('image_raw', 'rear_camera/image_raw'),
            ('camera_info', 'rear_camera/camera_info')
        ],
        parameters=[{
            'video_device': '/dev/video0',
            'image_size': [320, 240],
            'camera_info_url': 'file:///home/nevo/.ros/camera_info/c310_calibration_320x240.yaml',
            'camera_frame_id': 'c310_camera_link_optical'
        }]
    )

    rectify_node = Node(
        package='image_proc',
        executable='rectify_node',
        name='rectify_node',
        remappings=[
            ('image', 'rear_camera/image_raw'),         # Pulls from the new camera namespace
            ('camera_info', 'rear_camera/camera_info'),
            ('image_rect', 'rear_camera/image_rect')    # Outputs to the new namespace
        ]
    )

    apriltag_node = Node(
        package='apriltag_ros',
        executable='apriltag_node',
        parameters=[os.path.join(pkg_dir, 'config', 'tags.yaml')],
        remappings=[
            ('image_rect', 'rear_camera/image_rect'),   # Pulls the rectified image
            ('camera_info', 'rear_camera/camera_info')
        ]
    )
    
    # Bridges tag36h11:5 TF → /detected_dock_pose PoseStamped
    tag_pose_bridge = Node(
        package='nevo_bot',
        executable='tag_pose_bridge.py',
        name='tag_pose_bridge',
        output='screen'
    )

    docking_server = Node(
        package='opennav_docking',
        executable='opennav_docking',
        name='docking_server',
        output='screen',
        parameters=[os.path.join(pkg_dir, 'config', 'docking.yaml')]
    )

    return LaunchDescription([
        camera_node, rectify_node, apriltag_node, tag_pose_bridge, docking_server
    ])
