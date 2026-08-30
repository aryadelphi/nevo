import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription, TimerAction,
    RegisterEventHandler, DeclareLaunchArgument,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.event_handlers import OnProcessStart
from launch_ros.actions import Node


def get_selected_map():
    config_file = os.path.expanduser("~/nevo_selected_map.txt")
    default_map = "/home/nevo/nevo_ws/src/nevo-dev/nevo_bot/maps/abie_lab_save.yaml"
    if os.path.exists(config_file):
        with open(config_file, 'r') as f:
            map_path = f.read().strip()
        if map_path and os.path.exists(map_path):
            return map_path
    return default_map


def generate_launch_description():
    package_name = 'nevo_bot'
    use_sim_time = 'false'

    robot_id       = LaunchConfiguration('robot_id')
    ui_id          = LaunchConfiguration('ui_id')
    ws_uri_primary = LaunchConfiguration('ws_uri_primary')

    declare_robot_id = DeclareLaunchArgument('robot_id',       default_value='TFRB2')
    declare_ui_id    = DeclareLaunchArgument('ui_id',          default_value='TFUI2')
    declare_ws_uri   = DeclareLaunchArgument('ws_uri_primary', default_value='wss://tifa-ws.forgixrobotic.com')
    declare_map_arg = DeclareLaunchArgument(
        name="map",
        default_value=get_selected_map(),
        description="Full path to map yaml file"
    )
    map_file = LaunchConfiguration("map")

    pkg_share = get_package_share_directory(package_name)
    nav2_params_file = os.path.join(pkg_share, 'config', 'controller_mppi_params.yaml')

    # =========================================================================
    # LAYER 0 — immediate
    # =========================================================================
    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_share, 'launch', 'rsp.launch.py')),
        launch_arguments={'use_sim_time': use_sim_time, 'sim_mode': 'false'}.items()
    )
    rplidar_activate = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('rplidar_ros'), 'launch', 'rplidar_a1_launch.py')
        ),
        launch_arguments={'serial_port': '/dev/ttyLIDAR'}.items()
    )
    ws_bridge = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('nevo_ws_bridge'), 'launch', 'ws_bridge.launch.py')
        ),
        launch_arguments={'ws_uri_primary': ws_uri_primary, 'robot_id': robot_id, 'network_mode': 'auto'}.items()
    )
    diagnostic_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('nevo_diagnostic_node'), 'launch', 'diagnostic.launch.py')
        ),
        launch_arguments={'mode': 'navigation', 'robot_id': 'TFRB2', 'grace_period_sec': '10', 'global_timeout_sec': '120'}.items()
    )

    # =========================================================================
    # LAYER 1 — laser + controller_manager +5s
    # =========================================================================
    lidar_filter_config = os.path.join(pkg_share, 'config', 'lidar_filter.yaml')
    laser_filter_node = Node(
        package='laser_filters',
        executable='scan_to_scan_filter_chain',
        parameters=[lidar_filter_config, {'use_sim_time': False}],
        remappings=[('scan', '/scan'), ('scan_filtered', '/scan_filtered')]
    )

    # =========================================================================
    # LAYER 2 — OnProcessStart: localization + navigation + safety_fail + initial_pose + robot_api + sensor_manager
    # =========================================================================
    localization_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'localization_launch.py')
        ),
        launch_arguments={
            'map': map_file,
            'params_file': nav2_params_file,
            'use_sim_time': use_sim_time,
        }.items()
    )
    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'navigation_launch.py')
        ),
        launch_arguments={
            'map_subscribe_transient_local': 'true',
            'params_file': nav2_params_file,
            'use_sim_time': use_sim_time,
        }.items()
    )
    initial_pose_node = Node(
        package='nevo_bot',
        executable='publish_initial_pose',
        name='initial_pose_pub',
        output='log',
        arguments=['0.0', '0.0', '0.0']
    )
    safety_fail_node = Node(
        package='nevo_bot',
        executable='safety_fail_node',
        name='safety_fail_node',
        output='log'
    )
    robot_api = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('nevo_robot_api'), 'launch', 'robot_api.launch.py')
        ),
        launch_arguments={'robot_id': robot_id, 'ui_id': ui_id, 'mode': 'nav'}.items()
    )
    sensor_manager = Node(
        package='nevo_robot_api',
        executable='sensor_manager_node',
        name='sensor_manager_node'
    )

    # =========================================================================
    # TIMERS — staggered startup
    # =========================================================================
    return LaunchDescription([
        declare_robot_id, declare_ui_id, declare_ws_uri, declare_map_arg,

        # Layer 0
        rsp, rplidar_activate, ws_bridge, diagnostic_node,

        # Layer 1
        laser_filter_node,

        # Layer 2 — delayed timers for navigation stack
        TimerAction(period=2.0, actions=[localization_launch]),
        TimerAction(period=10.0, actions=[navigation_launch]),
        TimerAction(period=12.0, actions=[initial_pose_node]),
        TimerAction(period=14.0, actions=[safety_fail_node]),
        TimerAction(period=16.0, actions=[robot_api]),
        TimerAction(period=18.0, actions=[sensor_manager]),
    ])
