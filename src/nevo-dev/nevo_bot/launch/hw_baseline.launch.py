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

    delayed_kinect_stack = TimerAction(
        period=5.0,
        actions=[
            kinect_activate,
            kinect_tf,
        ]
    )

    return LaunchDescription([
        launch_robot,
        ina226_bridge,
        delayed_kinect_stack,
    ])
