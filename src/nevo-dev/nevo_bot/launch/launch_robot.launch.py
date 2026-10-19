import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription, TimerAction,
    RegisterEventHandler, DeclareLaunchArgument,
    SetEnvironmentVariable, GroupAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command, LaunchConfiguration, EqualsSubstitution, IfElseSubstitution
from launch.event_handlers import OnProcessStart
from launch_ros.actions import Node


def get_selected_map():
    config_file = os.path.expanduser("~/nevo_selected_map.txt")
    default_map = os.path.join(
        get_package_share_directory('nevo_bot'), 'maps', 'abie_lab_save.yaml')
    if os.path.exists(config_file):
        with open(config_file, 'r') as f:
            map_path = f.read().strip()
        if map_path and os.path.exists(map_path):
            return map_path
    return default_map


def generate_launch_description():
    package_name = 'nevo_bot'
    lidar_pkg    = 'rplidar_ros'

    robot_id       = LaunchConfiguration('robot_id')
    ui_id          = LaunchConfiguration('ui_id')
    ws_uri_primary = LaunchConfiguration('ws_uri_primary')
    # NOTE: top-level mode arg is named `stack_mode` on purpose. Nested
    # IncludeLaunchDescription actions leak their launch_arguments into the
    # shared context via SetLaunchConfiguration, so a plain `mode` would get
    # overwritten (e.g. by diagnostic_node's mode:=navigation) before the
    # timed layer-3 conditions are evaluated.
    stack_mode     = LaunchConfiguration('stack_mode')

    declare_robot_id = DeclareLaunchArgument('robot_id', default_value='TFRB2')
    declare_ui_id    = DeclareLaunchArgument('ui_id',    default_value='TFUI2')
    declare_ws_uri   = DeclareLaunchArgument('ws_uri_primary', default_value='wss://tifa-ws.forgixrobotic.com')
    # cloud:=false skips ws_bridge + robot_api (dashboard link, goals, map selection) for
    # manual runs, e.g. testing another map while nevo.service is stopped
    declare_cloud    = DeclareLaunchArgument('cloud', default_value='true')
    cloud_enabled    = IfCondition(LaunchConfiguration('cloud'))
    declare_mode_arg = DeclareLaunchArgument(
        name='stack_mode',
        default_value='nav',
        description='Stack mode: nav (localization+navigation+docking) or mapping (slam_toolbox)'
    )
    is_nav     = EqualsSubstitution(stack_mode, 'nav')
    is_mapping = EqualsSubstitution(stack_mode, 'mapping')
    declare_map_arg  = DeclareLaunchArgument(
        name='map',
        default_value=get_selected_map(),
        description='Full path to map yaml file'
    )
    map_file = LaunchConfiguration('map')
    declare_docking_arg = DeclareLaunchArgument(
        'docking', default_value='false',
        description='Start the rear camera + AprilTag + docking stack (needs the C310 on /dev/video0)')
    docking_enabled = LaunchConfiguration('docking')
    declare_nav_params_arg = DeclareLaunchArgument(
        'nav_params',
        default_value=os.path.join(get_package_share_directory(package_name), 'config', 'controller_mppi_params.yaml'),
        description='Nav2 + AMCL params file (thesis runs: config/hw_nav_mppi.yaml or hw_nav_dwb.yaml)')
    nav_params = LaunchConfiguration('nav_params')
    declare_enforce_map_id_arg = DeclareLaunchArgument(
        'enforce_map_id', default_value='false',
        description='robot_api: reject dashboard goals whose map_id differs from the loaded map')

    pkg_share           = get_package_share_directory(package_name)
    xacro_file          = os.path.join(pkg_share, 'description', 'robot.urdf.xacro')
    controller_params   = os.path.join(pkg_share, 'config', 'my_controllers.yaml')
    ekf_config          = os.path.join(pkg_share, 'config', 'ekf.yaml')
    lidar_filter_config = os.path.join(pkg_share, 'config', 'lidar_filter.yaml')
    twist_mux_config    = os.path.join(pkg_share, 'config', 'twist_mux.yaml')

    robot_description = ParameterValue(Command(['xacro ', xacro_file, ' sim_mode:=false']), value_type=str)

    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_share, 'launch', 'rsp.launch.py')),
        launch_arguments={'use_sim_time': 'false', 'sim_mode': 'false'}.items()
    )

    controller_manager = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[{'robot_description': robot_description}, controller_params]
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
        launch_arguments={'ws_uri_primary': ws_uri_primary, 'robot_id': robot_id, 'network_mode': 'auto'}.items(),
        condition=cloud_enabled,
    )
    diag_mode = IfElseSubstitution(is_mapping, 'mapping', 'navigation')
    diagnostic_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('nevo_diagnostic_node'), 'launch', 'diagnostic.launch.py')
        ),
        launch_arguments={'mode': diag_mode, 'robot_id': robot_id, 'grace_period_sec': '30', 'global_timeout_sec': '300'}.items()
    )

    # =========================================================================
    # LAYER 1 — laser/rf2o immediate, controller_manager +5s
    # =========================================================================
    laser_filter_node = Node(
        package='laser_filters',
        executable='scan_to_scan_filter_chain',
        parameters=[lidar_filter_config, {'use_sim_time': False}],
        remappings=[('scan', '/scan'), ('scan_filtered', '/scan_filtered')]
    )
    imu_node = Node(
        package='nevo_bot',
        executable='imu_publisher.py',
        name='imu_publisher',
        output='screen',
        # Arduino Nano + MPU6050 (2026-10-18). by-id path until the ttyIMU udev rule is updated.
        # gyro_cov z 1e-5: validated 2026-10-18 (3+3 turns at 0.35 rad/s, IMU within 0.3 deg
        # of lidar ICP vs wheels 3.5 deg), so the EKF takes yaw rate from the IMU, not the wheels
        parameters=[{'serial_port': '/dev/serial/by-id/usb-1a86_USB2.0-Ser_-if00-port0',
                     'tilt_correction': 'fixed',
                     'gyro_cov': [1e-4, 1e-4, 1e-5]}],
    )
    # rf2o: lidar odometry (twist only, no TF) fused by the EKF next to the wheels.
    # Wheels slip (esp. reversing); lidar does not. Needs the patched rf2o that
    # publishes twist_cov_* (stock rf2o publishes zero covariance).
    rf2o_node = Node(
        package='rf2o_laser_odometry',
        executable='rf2o_laser_odometry_node',
        name='rf2o_laser_odometry',
        output='log',
        ros_arguments=['--log-level', 'rf2o_laser_odometry:=warn'],
        parameters=[{
            'laser_scan_topic': '/scan_filtered', 'odom_topic': '/odom_rf2o',
            'publish_tf': False, 'base_frame_id': 'base_link',
            'odom_frame_id': 'odom', 'init_pose_from_topic': '', 'freq': 10.0,
            # lower = more weight in the EKF. vyaw below the wheel 0.001 so lidar
            # wins on rotation; vx equal to the wheels (rf2o vx is weaker in long corridors).
            'twist_cov_vx': 0.001, 'twist_cov_vyaw': 0.0005,
        }],
    )

    delayed_controller_manager = TimerAction(period=5.0, actions=[controller_manager])

    # =========================================================================
    # LAYER 2 — OnProcessStart(controller_manager)
    # =========================================================================
    # One spawner, in order: joint_state_broadcaster first (state interfaces),
    # then diff_cont (command interfaces). Long timeouts because the Pi is busy
    # during bringup; two separate spawners on timers raced and one died.
    controllers_spawner = Node(
        package='controller_manager', executable='spawner',
        arguments=['joint_state_broadcaster', 'diff_cont',
                   '--controller-manager-timeout', '60',
                   '--service-call-timeout', '30']
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
            on_start=[controllers_spawner, ekf_node, twist_mux]
        )
    )

    # =========================================================================
    # LAYER 3 — TimerAction from OnProcessStart(controller_manager)
    # =========================================================================
    localization_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_share, 'launch', 'localization_launch.py')),
        launch_arguments={'map': map_file, 'params_file': nav_params}.items(),
        condition=IfCondition(is_nav)
    )
    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_share, 'launch', 'navigation_launch.py')),
        launch_arguments={'use_sim_time': 'false', 'map_subscribe_transient_local': 'true',
                          'params_file': nav_params}.items(),
        condition=IfCondition(is_nav)
    )
    safety_fail_node = Node(
        package='nevo_bot', executable='safety_fail_node', name='safety_fail_node', output='log'
    )
    slam_params_file = os.path.join(pkg_share, 'config', 'mapper_params_online_async.yaml')
    mapping_slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('slam_toolbox'), 'launch', 'online_async_launch.py')
        ),
        launch_arguments={'slam_params_file': slam_params_file, 'use_sim_time': 'false'}.items(),
        condition=IfCondition(is_mapping)
    )
    layer3_timed = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=controller_manager,
            on_start=[
                TimerAction(period=20.0, actions=[localization_launch]),
                TimerAction(period=25.0, actions=[navigation_launch]),
                TimerAction(period=22.0, actions=[safety_fail_node]),
                TimerAction(period=8.0, actions=[mapping_slam]),
            ]
        )
    )

    # =========================================================================
    # LAYER 4 — TimerAction from OnProcessStart(controller_manager)
    # =========================================================================
    robot_api = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('nevo_robot_api'), 'launch', 'robot_api.launch.py')
        ),
        launch_arguments={'robot_id': robot_id, 'ui_id': ui_id, 'mode': stack_mode,
                          'enforce_map_id': LaunchConfiguration('enforce_map_id')}.items(),
        condition=cloud_enabled,
    )
    sensor_manager = Node(
        package='nevo_robot_api', executable='sensor_manager_node', name='sensor_manager_node'
    )
    mapping_position_sensor = Node(
        package='nevo_robot_api', executable='position_sensor_node', name='position_sensor_node',
        condition=IfCondition(is_mapping)
    )
    mapping_slam_supervisor = Node(
        package='nevo_robot_api', executable='slam_supervisor_node', name='slam_supervisor_node',
        condition=IfCondition(is_mapping)
    )

    layer4_timed = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=controller_manager,
            on_start=[
                TimerAction(period=22.0, actions=[robot_api]),
                TimerAction(period=25.0, actions=[sensor_manager]),
                TimerAction(period=12.0, actions=[mapping_position_sensor]),
                TimerAction(period=14.0, actions=[mapping_slam_supervisor]),
            ]
        )
    )

    # =========================================================================
    # LAYER 2b — INA226 battery bridge (on controller_manager start +5s)
    # =========================================================================
    ina226_bridge = Node(
        package='nevo_bot',
        executable='ina226_bridge_node.py',
        name='ina226_bridge',
        output='screen'
    )

    # =========================================================================
    # LAYER 2c — Docking + Kinect stack (on controller_manager start +5s)
    # =========================================================================
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
        parameters=[os.path.join(pkg_share, 'config', 'tags.yaml')],
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
        parameters=[os.path.join(pkg_share, 'config', 'docking.yaml')],
        # Same path as Nav2: velocity_smoother -> twist_mux "navigation".
        remappings=[('cmd_vel', 'cmd_vel_nav')],
        condition=IfCondition(is_nav)
    )
    # navigation_launch's lifecycle manager only manages the core Nav2 servers,
    # so docking_server needs its own (it stayed unconfigured before).
    docking_lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_docking',
        output='log',
        parameters=[{'autostart': True, 'node_names': ['docking_server'], 'bond_timeout': 10.0}],
        condition=IfCondition(is_nav)
    )
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

    layer2b_on_controller_start = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=controller_manager,
            on_start=[
                TimerAction(period=5.0, actions=[
                    #ina226_bridge,  # DISABLED — Arduino unplugged for rewiring
                    kinect_activate, kinect_tf,
                ]),
                # Docking stack (rear camera, AprilTag, docking_server) only on request.
                TimerAction(period=5.0, actions=[
                    GroupAction(
                        condition=IfCondition(docking_enabled),
                        actions=[camera_node, rectify_node, apriltag_node,
                                 tag_pose_bridge, docking_server, docking_lifecycle_manager],
                    ),
                ]),
            ]
        )
    )

    # =========================================================================
    # RETURN
    # =========================================================================
    fix_rmw_env = SetEnvironmentVariable(
        'RMW_IMPLEMENTATION',
        'rmw_fastrtps_cpp'
    )
    fix_fastdds_env = SetEnvironmentVariable(
        'FASTRTPS_DEFAULT_PROFILES_FILE',
        'file:///home/nevo/fastdds.xml'
    )

    return LaunchDescription([
        fix_rmw_env, fix_fastdds_env,
        declare_map_arg, declare_robot_id, declare_ui_id, declare_ws_uri, declare_cloud, declare_mode_arg,
        declare_docking_arg, declare_nav_params_arg, declare_enforce_map_id_arg,

        # Layer 0
        rsp, rplidar_activate, ws_bridge, diagnostic_node,

        # Layer 1
        laser_filter_node, rf2o_node, imu_node,
        delayed_controller_manager,

        # Layer 2
        layer2_on_controller_start,

        # Layer 2b — INA226 + Docking + Kinect
        layer2b_on_controller_start,

        # Layer 3
        layer3_timed,

        # Layer 4
        layer4_timed,
    ])
