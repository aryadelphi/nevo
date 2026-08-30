import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    pkg_dir = get_package_share_directory('nevo_bot')

    # launch_robot now includes: RSP, rplidar, ws_bridge, diagnostic_node,
    # laser_filter, controller_manager, EKF, twist_mux, safety_fail_node,
    # initial_pose, localization, navigation, robot_api, sensor_manager
    launch_robot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(pkg_dir, 'launch', 'launch_robot.launch.py')
        ]),
    )

    ina226_bridge = Node(
        package='nevo_bot',
        executable='ina226_bridge_node.py',
        name='ina226_bridge',
        output='screen'
    )

    # --- Rear Docking Camera Stack ---
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
            ('image', 'rear_camera/image_raw'),
            ('camera_info', 'rear_camera/camera_info'),
            ('image_rect', 'rear_camera/image_rect')
        ]
    )

    apriltag_node = Node(
        package='apriltag_ros',
        executable='apriltag_node',
        parameters=[os.path.join(pkg_dir, 'config', 'tags.yaml')],
        remappings=[
            ('image_rect', 'rear_camera/image_rect'),
            ('camera_info', 'rear_camera/camera_info')
        ]
    )

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

    # Kinect depth camera
    kinect_activate = Node(
        package='kinect_ros2',
        executable='kinect_ros2_node',
        output='screen'
    )

    kinect_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='kinect_tf_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'camera_link_optical', 'kinect_depth']
    )

    delayed_docking_stack = TimerAction(
        period=5.0,
        actions=[
            camera_node,
            rectify_node,
            apriltag_node,
            tag_pose_bridge,
            docking_server,
            kinect_activate,
            kinect_tf,
        ]
    )

    return LaunchDescription([
        launch_robot,
        ina226_bridge,
        delayed_docking_stack,
    ])
