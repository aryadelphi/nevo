import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription,
    DeclareLaunchArgument,
    TimerAction,
    RegisterEventHandler,
    SetEnvironmentVariable,
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, EnvironmentVariable
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    package_name = 'nevo_bot'
    use_sim_time = LaunchConfiguration('use_sim_time')
    world = LaunchConfiguration('world')
    world_name = LaunchConfiguration('world_name')

    declare_use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time', default_value='true',
        description='Use simulation clock'
    )

    default_world = PathJoinSubstitution([
        FindPackageShare(package_name), 'worlds', 'docking_station.world'
    ])

    declare_world_arg = DeclareLaunchArgument(
        'world', default_value=default_world,
        description='Path to the gz-sim world file'
    )

    declare_world_name_arg = DeclareLaunchArgument(
        'world_name', default_value='default',
        description='<world name="..."> inside the world file, used by ros_gz_sim create'
    )

    declare_robot_id = DeclareLaunchArgument('robot_id', default_value='TFRB2')
    declare_ui_id = DeclareLaunchArgument('ui_id', default_value='TFUI2')
    declare_ws_uri = DeclareLaunchArgument('ws_uri_primary', default_value='wss://tifa-ws.forgixrobotic.com')

    # Bind gz-transport discovery to loopback: this host filters multicast on
    # the physical NIC, otherwise /world/default/create never appears
    # (README_amcl_health_sim.md §6). An exported GZ_IP overrides this.
    gz_discovery_ip = SetEnvironmentVariable(
        name='GZ_IP',
        value=EnvironmentVariable('GZ_IP', default_value='127.0.0.1'),
    )

    # Layer 0 — Gazebo Harmonic (gz sim). gazebo_ros does not exist on Jazzy.
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')]),
        launch_arguments={
            'gz_args': ['-r ', world],
            'on_exit_shutdown': 'true',
        }.items()
    )

    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory(package_name), 'launch', 'rsp.launch.py'
        )]),
        launch_arguments={'use_sim_time': use_sim_time, 'sim_mode': 'true'}.items()
    )

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
        launch_arguments={'mode': 'navigation', 'robot_id': 'TFRB2', 'grace_period_sec': '10', 'global_timeout_sec': '120', 'enable_stack_lock': 'false'}.items()
    )

    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-world', world_name,
                   '-name', 'my_bot',
                   '-topic', 'robot_description',
                   '-x', '0', '-y', '0.40', '-z', '0', '-Y', '0'],
        output='screen'
    )
    delayed_spawn = TimerAction(period=4.0, actions=[spawn_entity])

    # gz-sim -> ROS topic bridges (see launch_sim.launch.py for details)
    gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            '/c310/image_raw@sensor_msgs/msg/Image[gz.msgs.Image',
            '/c310/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
            '/camera/depth/image_raw@sensor_msgs/msg/Image[gz.msgs.Image',
            '/camera/depth/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
            '/camera/depth/image_raw/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked',
        ],
        remappings=[
            ('/camera/depth/image_raw/points', '/camera/points'),
        ],
        output='screen'
    )

    joint_broad_spawner = Node(
        package="controller_manager", executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager-timeout", "60"],
    )

    diff_drive_spawner = Node(
        package="controller_manager", executable="spawner",
        arguments=["diff_cont", "--controller-manager-timeout", "60"],
    )

    spawn_controllers = RegisterEventHandler(
        OnProcessExit(target_action=spawn_entity, on_exit=[joint_broad_spawner])
    )
    spawn_diff_drive = RegisterEventHandler(
        OnProcessExit(target_action=joint_broad_spawner, on_exit=[diff_drive_spawner])
    )

    apriltag_node = Node(
        package='apriltag_ros',
        executable='apriltag_node',
        name='apriltag',
        parameters=[os.path.join(
            get_package_share_directory(package_name), 'config', 'tags.yaml'
        )],
        remappings=[
            ('image_rect', '/c310/image_raw'),
            ('camera_info', '/c310/camera_info')
        ]
    )

    tag_pose_bridge = Node(
        package='nevo_bot',
        executable='tag_pose_bridge_sim.py',
        name='tag_pose_bridge_sim',
        output='screen'
    )

    docking_server = Node(
        package='opennav_docking',
        executable='opennav_docking',
        name='docking_server',
        output='screen',
        parameters=[os.path.join(
            get_package_share_directory(package_name), 'config', 'docking_sim.yaml'
        )]
    )

    return LaunchDescription([
        declare_use_sim_time_arg,
        declare_world_arg,
        declare_world_name_arg,
        declare_robot_id,
        declare_ui_id,
        declare_ws_uri,
        gz_discovery_ip,
        gazebo,
        rsp,
        ws_bridge,
        diagnostic_node,
        gz_bridge,
        delayed_spawn,
        spawn_controllers,
        spawn_diff_drive,
        apriltag_node,
        tag_pose_bridge,
        docking_server,
    ])
