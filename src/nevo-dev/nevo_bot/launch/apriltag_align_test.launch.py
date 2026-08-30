import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    pkg_dir = get_package_share_directory('nevo_bot')

    camera_node = Node(
        package='v4l2_camera',
        executable='v4l2_camera_node',
        parameters=[{
            'video_device': '/dev/video0',
            'image_size': [640, 480],
            'camera_info_url': 'file:///home/nevo/.ros/camera_info/c310_calibration.yaml',
            'camera_frame_id': 'c310_camera_link_optical'
        }]
    )

    rectify_node = Node(
        package='image_proc',
        executable='rectify_node',
        name='rectify_node',
        remappings=[
            ('image', '/image_raw'),
            ('camera_info', '/camera_info'),
            ('image_rect', '/image_rect')
        ]
    )

    apriltag_node = Node(
        package='apriltag_ros',
        executable='apriltag_node',
        parameters=[os.path.join(pkg_dir, 'config', 'tags.yaml')],
        remappings=[
            ('image_rect', '/image_rect'),
            ('camera_info', '/camera_info')
        ]
    )

    tag_pose_bridge = Node(
        package='nevo_bot',
        executable='tag_pose_bridge.py',
        name='tag_pose_bridge',
        output='screen'
    )

    align_test = Node(
        package='nevo_bot',
        executable='apriltag_align_test.py',
        name='apriltag_align_test',
        output='screen'
    )

    return LaunchDescription([
        camera_node, rectify_node, apriltag_node, tag_pose_bridge, align_test
    ])
