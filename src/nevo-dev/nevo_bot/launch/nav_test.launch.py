import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    pkg_dir = get_package_share_directory('nevo_bot')
    params_file = os.path.join(pkg_dir, 'config', 'controller_mppi_params_test.yaml')

    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        parameters=[{
            'use_sim_time': False,
            'yaml_filename': os.path.join(pkg_dir, 'maps', 'docking_save_map.yaml'),
        }]
    )

    map_server_lifecycle = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_map',
        parameters=[{
            'use_sim_time': False,
            'autostart': True,
            'node_names': ['map_server'],
        }]
    )

    # Position robot at the dock staging area: staging ~= (4.18, 1.48, yaw=1.54)
    # This way the staging navigation goal succeeds immediately.
    static_map_odom = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_map_to_odom',
        arguments=['2.0', '0.0', '0', '0', '0', '0', 'map', 'odom']
    )

    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_dir, 'launch', 'navigation_launch.py')),
        launch_arguments={
            'params_file': params_file,
            'use_sim_time': 'false',
            'autostart': 'true',
        }.items()
    )

    return LaunchDescription([
        map_server,
        map_server_lifecycle,
        static_map_odom,
        TimerAction(period=3.0, actions=[navigation]),
    ])
