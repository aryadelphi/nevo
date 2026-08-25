import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    package_name = 'nevo_bot'
    use_sim_time = 'true'

    declare_map_arg = DeclareLaunchArgument(
        name="map",
        default_value=os.path.join(
            get_package_share_directory(package_name), 'maps', 'turtlebot3_save_map.yaml'
        ),
        description="Full path to map yaml file"
    )
    map_file = LaunchConfiguration("map")
    declare_robot_id = DeclareLaunchArgument('robot_id', default_value='TFRB2')
    declare_ui_id = DeclareLaunchArgument('ui_id', default_value='TFUI2')
    declare_ws_uri = DeclareLaunchArgument('ws_uri_primary', default_value='wss://tifa-ws.forgixrobotic.com')

    pkg_dir = get_package_share_directory(package_name)
    nav2_params_file = os.path.join(pkg_dir, 'config', 'stvl_sim_params.yaml')
    lidar_filter_config = os.path.join(pkg_dir, 'config', 'lidar_filter.yaml')

    # Layer 0 — immediate
    ws_bridge = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('nevo_ws_bridge'), 'launch', 'ws_bridge.launch.py')
        ),
        launch_arguments={'ws_uri_primary': LaunchConfiguration('ws_uri_primary'), 'robot_id': LaunchConfiguration('robot_id'), 'network_mode': 'auto'}.items()
    )
    diagnostic_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('nevo_diagnostic_node'), 'launch', 'diagnostic.launch.py')
        ),
        launch_arguments={'mode': 'navigation', 'robot_id': 'TFRB2', 'grace_period_sec': '10', 'global_timeout_sec': '120'}.items()
    )

    laser_filter_node = Node(
        package="laser_filters",
        executable="scan_to_scan_filter_chain",
        parameters=[lidar_filter_config, {'use_sim_time': True}],
        remappings=[('scan', '/scan'), ('scan_filtered', '/scan_filtered')]
    )

    localization_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_dir, 'launch', 'localization_launch.py')
        ),
        launch_arguments={'map': map_file, 'params_file': nav2_params_file, 'use_sim_time': use_sim_time}.items()
    )

    initial_pose_node = Node(
        package='nevo_bot', executable='publish_initial_pose',
        name='initial_pose_pub', output='log',
        parameters=[{'use_sim_time': True}],
        arguments=['0.691', '-0.788', '-0.418']
    )

    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_dir, 'launch', 'navigation_launch.py')
        ),
        launch_arguments={'map_subscribe_transient_local': 'true', 'params_file': nav2_params_file, 'use_sim_time': use_sim_time}.items()
    )

    safety_fail_node = Node(
        package='nevo_bot', executable='safety_fail_node', name='safety_fail_node', output='log'
    )

    robot_api = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('nevo_robot_api'), 'launch', 'robot_api.launch.py')
        ),
        launch_arguments={'robot_id': LaunchConfiguration('robot_id'), 'ui_id': LaunchConfiguration('ui_id')}.items()
    )
    sensor_manager = Node(
        package='nevo_robot_api', executable='sensor_manager_node', name='sensor_manager_node'
    )

    return LaunchDescription([
        declare_map_arg, declare_robot_id, declare_ui_id, declare_ws_uri,
        ws_bridge, diagnostic_node,
        TimerAction(period=1.0, actions=[laser_filter_node]),
        TimerAction(period=2.0, actions=[localization_launch]),
        TimerAction(period=5.0, actions=[initial_pose_node]),
        TimerAction(period=10.0, actions=[navigation_launch]),
        TimerAction(period=12.0, actions=[safety_fail_node]),
        TimerAction(period=14.0, actions=[robot_api, sensor_manager]),
    ])
