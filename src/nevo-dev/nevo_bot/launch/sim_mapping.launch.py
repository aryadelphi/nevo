import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    package_name = 'nevo_bot'
    use_sim_time = 'true'

    pkg_dir = get_package_share_directory(package_name)
    slam_toolbox_dir = get_package_share_directory('slam_toolbox')

    slam_params_file = os.path.join(pkg_dir, 'config', 'mapper_params_online_async.yaml')
    lidar_filter_config = os.path.join(pkg_dir, 'config', 'lidar_filter.yaml')
    nav2_params_file = os.path.join(pkg_dir, 'config', 'stvl_sim_params.yaml')
    explore_config = os.path.join(pkg_dir, 'config', 'explore.yaml')

    declare_explore_arg = DeclareLaunchArgument(
        'auto', default_value='false',
        description='Run autonomous exploration (m-explore)'
    )
    declare_robot_id = DeclareLaunchArgument('robot_id', default_value='TFRB2')
    declare_ui_id = DeclareLaunchArgument('ui_id', default_value='TFUI2')
    declare_ws_uri = DeclareLaunchArgument('ws_uri_primary', default_value='wss://tifa-ws.forgixrobotic.com')

    explore = LaunchConfiguration('auto')

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
        launch_arguments={'mode': 'mapping', 'robot_id': 'TFRB2', 'grace_period_sec': '10', 'global_timeout_sec': '60'}.items()
    )

    laser_filter_node = Node(
        package="laser_filters",
        executable="scan_to_scan_filter_chain",
        parameters=[lidar_filter_config, {'use_sim_time': True}],
        remappings=[('scan', '/scan'), ('scan_filtered', '/scan_filtered')]
    )

    slam_activate = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(slam_toolbox_dir, 'launch', 'online_async_launch.py')
        ),
        launch_arguments={'slam_params_file': slam_params_file, 'use_sim_time': use_sim_time}.items()
    )

    nav2_activate = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('nav2_bringup'), 'launch', 'navigation_launch.py')
        ),
        launch_arguments={'use_sim_time': use_sim_time, 'params_file': nav2_params_file}.items()
    )

    explore_node = Node(
        package='explore_lite', executable='explore', name='explore_node',
        output='screen', parameters=[explore_config, {'use_sim_time': True}],
        condition=IfCondition(explore)
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
    slam_supervisor = Node(
        package='nevo_robot_api', executable='slam_supervisor_node', name='slam_supervisor_node'
    )
    position_sensor_node = Node(
        package='nevo_robot_api', executable='position_sensor_node', name='position_sensor_node'
    )

    return LaunchDescription([
        declare_explore_arg, declare_robot_id, declare_ui_id, declare_ws_uri,
        ws_bridge, diagnostic_node,
        TimerAction(period=2.0, actions=[laser_filter_node]),
        TimerAction(period=5.0, actions=[slam_activate]),
        TimerAction(period=8.0, actions=[nav2_activate]),
        TimerAction(period=10.0, actions=[safety_fail_node]),
        TimerAction(period=15.0, actions=[explore_node, robot_api, sensor_manager, slam_supervisor, position_sensor_node]),
    ])
