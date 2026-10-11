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
    declare_core_arg = DeclareLaunchArgument(
        'core', default_value='false',
        description='Also start Layer 0 (ws_bridge, diagnostic, safety_fail). '
                    'launch_sim.launch.py already provides all three, so the sim '
                    'layers default to false — otherwise two ws_bridge nodes fight '
                    'over the same cloud session.'
    )
    declare_robot_id = DeclareLaunchArgument('robot_id', default_value='TFRB2')
    declare_ui_id = DeclareLaunchArgument('ui_id', default_value='TFUI2')
    declare_ws_uri = DeclareLaunchArgument('ws_uri_primary', default_value='wss://tifa-ws.forgixrobotic.com')
    declare_nevo_extras_arg = DeclareLaunchArgument(
        'nevo_extras', default_value='false',
        description='Also start the nevo robot_api layer (robot_api, '
                    'sensor_manager, slam_supervisor, position_sensor). '
                    'Default false = pure nav2 + slam_toolbox + ros2_control.'
    )
    nevo_extras = LaunchConfiguration('nevo_extras')

    explore = LaunchConfiguration('auto')
    core = LaunchConfiguration('core')

    # Layer 0 — immediate (only when launch_sim is not already providing it)
    ws_bridge = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('nevo_ws_bridge'), 'launch', 'ws_bridge.launch.py')
        ),
        launch_arguments={'ws_uri_primary': LaunchConfiguration('ws_uri_primary'), 'robot_id': LaunchConfiguration('robot_id'), 'network_mode': 'auto'}.items(),
        condition=IfCondition(core)
    )
    diagnostic_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('nevo_diagnostic_node'), 'launch', 'diagnostic.launch.py')
        ),
        launch_arguments={'mode': 'mapping', 'robot_id': 'TFRB2', 'grace_period_sec': '10', 'global_timeout_sec': '60', 'enable_stack_lock': 'false'}.items(),
        condition=IfCondition(core)
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
        package='nevo_bot', executable='safety_fail_node', name='safety_fail_node', output='log',
        # sim time stamps, so diff_cont's cmd_vel_timeout works (see launch_sim)
        parameters=[{"use_sim_time": True}],
        condition=IfCondition(core)
    )

    robot_api = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('nevo_robot_api'), 'launch', 'robot_api.launch.py')
        ),
        launch_arguments={'robot_id': LaunchConfiguration('robot_id'), 'ui_id': LaunchConfiguration('ui_id')}.items(),
        condition=IfCondition(nevo_extras)
    )
    sensor_manager = Node(
        package='nevo_robot_api', executable='sensor_manager_node', name='sensor_manager_node',
        condition=IfCondition(nevo_extras)
    )
    slam_supervisor = Node(
        package='nevo_robot_api', executable='slam_supervisor_node', name='slam_supervisor_node',
        condition=IfCondition(nevo_extras)
    )
    position_sensor_node = Node(
        package='nevo_robot_api', executable='position_sensor_node', name='position_sensor_node',
        condition=IfCondition(nevo_extras)
    )

    return LaunchDescription([
        declare_explore_arg, declare_core_arg, declare_robot_id, declare_ui_id, declare_ws_uri,
        declare_nevo_extras_arg,
        ws_bridge, diagnostic_node,
        TimerAction(period=2.0, actions=[laser_filter_node]),
        TimerAction(period=5.0, actions=[slam_activate]),
        TimerAction(period=8.0, actions=[nav2_activate]),
        TimerAction(period=10.0, actions=[safety_fail_node]),
        TimerAction(period=15.0, actions=[explore_node, robot_api, sensor_manager, slam_supervisor, position_sensor_node]),
    ])
