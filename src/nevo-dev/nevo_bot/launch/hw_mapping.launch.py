import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription, TimerAction,
    RegisterEventHandler, DeclareLaunchArgument,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch.event_handlers import OnProcessStart, OnProcessExit
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.conditions import IfCondition


def generate_launch_description():
    package_name    = 'nevo_bot'
    slam_toolbox_pkg = 'slam_toolbox'
    lidar_pkg       = 'rplidar_ros'

    robot_id       = LaunchConfiguration('robot_id')
    ui_id          = LaunchConfiguration('ui_id')
    ws_uri_primary = LaunchConfiguration('ws_uri_primary')

    declare_robot_id = DeclareLaunchArgument('robot_id',       default_value='TFRB2')
    declare_ui_id    = DeclareLaunchArgument('ui_id',          default_value='TFUI2')
    declare_ws_uri   = DeclareLaunchArgument('ws_uri_primary', default_value='wss://tifa-ws.forgixrobotic.com')

    declare_explore_arg = DeclareLaunchArgument(
        'auto', default_value='false',
        description='Run autonomous exploration (m-explore)'
    )

    pkg_share           = get_package_share_directory(package_name)
    xacro_file          = os.path.join(pkg_share, 'description', 'robot.urdf.xacro')
    controller_params   = os.path.join(pkg_share, 'config', 'my_controllers.yaml')
    ekf_config          = os.path.join(pkg_share, 'config', 'ekf.yaml')
    lidar_filter_config = os.path.join(pkg_share, 'config', 'lidar_filter.yaml')
    twist_mux_config    = os.path.join(pkg_share, 'config', 'twist_mux.yaml')
    slam_params_file    = os.path.join(pkg_share, 'config', 'mapper_params_online_async.yaml')
    nav2_params_file    = os.path.join(pkg_share, 'config', 'controller_mppi_params.yaml')
    explore_config      = os.path.join(pkg_share, 'config', 'explore.yaml')

    robot_description = ParameterValue(Command(['xacro ', xacro_file, ' sim_mode:=false']), value_type=str)

    # =========================================================================
    # LAYER 0 — immediate
    # =========================================================================
    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_share, 'launch', 'rsp.launch.py')),
        launch_arguments={'use_sim_time': 'false', 'sim_mode': 'false'}.items()
    )
    rplidar_activate = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory(lidar_pkg), 'launch', 'rplidar_a1_launch.py')
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
        launch_arguments={'mode': 'mapping', 'robot_id': 'TFRB2', 'grace_period_sec': '10', 'global_timeout_sec': '60'}.items()
    )

    # =========================================================================
    # LAYER 1 — laser/rf2o immediate, controller_manager +3s
    # =========================================================================
    laser_filter_node = Node(
        package='laser_filters',
        executable='scan_to_scan_filter_chain',
        parameters=[lidar_filter_config, {'use_sim_time': False}],
        remappings=[('scan', '/scan'), ('scan_filtered', '/scan_filtered')]
    )
    # RF2O disabled for now
    # rf2o_node = Node(...)
    controller_manager = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[{'robot_description': robot_description}, controller_params]
    )
    delayed_controller_manager = TimerAction(period=3.0, actions=[controller_manager])

    # =========================================================================
    # LAYER 2 — OnProcessStart(controller_manager)
    # =========================================================================
    diff_drive_spawner = Node(
        package='controller_manager', executable='spawner', arguments=['diff_cont']
    )
    joint_state_broadcaster_spawner = Node(
        package='controller_manager', executable='spawner', arguments=['joint_state_broadcaster']
    )
    ekf_node = Node(
        package='robot_localization', executable='ekf_node',
        name='ekf_filter_node', output='log', parameters=[ekf_config]
    )
    twist_mux = Node(
        package='twist_mux', executable='twist_mux', name='twist_mux', output='log',
        arguments=['--ros-args', '--log-level', 'warn'],
        parameters=[twist_mux_config],
        remappings=[('cmd_vel_out', '/diff_cont/cmd_vel_unstamped_raw')]
    )

    layer2_on_controller_start = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=controller_manager,
            on_start=[diff_drive_spawner, joint_state_broadcaster_spawner, ekf_node, twist_mux]
        )
    )

    # =========================================================================
    # LAYER 3 — OnProcessExit(diff_drive_spawner): SLAM + safety
    # =========================================================================
    slam_activate = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory(slam_toolbox_pkg), 'launch', 'online_async_launch.py')
        ),
        launch_arguments={
            'slam_params_file': slam_params_file,
            'use_sim_time': 'false'
        }.items()
    )
    safety_fail_node = Node(
        package='nevo_bot', executable='safety_fail_node',
        name='safety_fail_node', output='log'
    )
    explore = LaunchConfiguration('auto')
    explore_node = Node(
        package='explore_lite', executable='explore', name='explore_node',
        output='screen', parameters=[explore_config, {'use_sim_time': False}],
        condition=IfCondition(explore)
    )

    layer3_on_diff_cont_done = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=diff_drive_spawner,
            on_exit=[slam_activate, safety_fail_node, explore_node]
        )
    )

    # =========================================================================
    # LAYER 4 — robot_api, sensor_manager, slam_supervisor, position_sensor
    # =========================================================================
    robot_api = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('nevo_robot_api'), 'launch', 'robot_api.launch.py')
        ),
        launch_arguments={'robot_id': robot_id, 'ui_id': ui_id, 'mode': 'mapping'}.items()
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

    layer4_after_slam_init = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=diff_drive_spawner,
            on_exit=[
                TimerAction(period=8.0, actions=[
                    robot_api, sensor_manager, slam_supervisor, position_sensor_node,
                ])
            ]
        )
    )

    # =========================================================================
    # RETURN
    # =========================================================================
    return LaunchDescription([
        declare_robot_id, declare_ui_id, declare_ws_uri, declare_explore_arg,

        # Layer 0
        rsp, rplidar_activate, ws_bridge, diagnostic_node,

        # Layer 1
        laser_filter_node, delayed_controller_manager,

        # Layer 2
        layer2_on_controller_start,

        # Layer 3
        layer3_on_diff_cont_done,

        # Layer 4
        layer4_after_slam_init,
    ])
