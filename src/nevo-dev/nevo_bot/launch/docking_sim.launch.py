import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    package_name = 'nevo_bot'
    use_sim_time = LaunchConfiguration('use_sim_time')
    world = LaunchConfiguration('world')

    declare_use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time', default_value='true',
        description='Use simulation clock'
    )

    default_world = PathJoinSubstitution([
        FindPackageShare(package_name), 'worlds', 'docking_station.world'
    ])

    declare_world_arg = DeclareLaunchArgument(
        'world', default_value=default_world,
        description='Path to .world file'
    )

    declare_robot_id = DeclareLaunchArgument('robot_id', default_value='TFRB2')
    declare_ui_id = DeclareLaunchArgument('ui_id', default_value='TFUI2')
    declare_ws_uri = DeclareLaunchArgument('ws_uri_primary', default_value='wss://tifa-ws.forgixrobotic.com')

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py'
        )]),
        launch_arguments={'world': world, 'verbose': 'true'}.items()
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
        launch_arguments={'mode': 'navigation', 'robot_id': 'TFRB2', 'grace_period_sec': '10', 'global_timeout_sec': '120'}.items()
    )

    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description', '-entity', 'my_bot',
                    '-x', '0', '-y', '0.40', '-z', '0', '-Y', '0'],
        output='screen'
    )

    joint_broad_spawner = Node(
        package="controller_manager", executable="spawner",
        arguments=["joint_state_broadcaster"],
    )

    diff_drive_spawner = Node(
        package="controller_manager", executable="spawner",
        arguments=["diff_cont"],
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
        declare_robot_id,
        declare_ui_id,
        declare_ws_uri,
        gazebo,
        rsp,
        ws_bridge,
        diagnostic_node,
        spawn_entity,
        joint_broad_spawner,
        diff_drive_spawner,
        apriltag_node,
        tag_pose_bridge,
        docking_server,
    ])
