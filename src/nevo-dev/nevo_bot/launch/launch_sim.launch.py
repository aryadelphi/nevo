import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription,
    DeclareLaunchArgument,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    package_name = 'nevo_bot'

    use_sim_time = LaunchConfiguration('use_sim_time')
    world = LaunchConfiguration('world')
    rviz = LaunchConfiguration('rviz')
    teleop = LaunchConfiguration('teleop')

    declare_use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time', default_value='true',
        description='Use simulation clock (always true for sim)'
    )

    default_world = PathJoinSubstitution([
        FindPackageShare(package_name), 'worlds', 'empty.world'
    ])

    declare_world_arg = DeclareLaunchArgument(
        'world', default_value=default_world,
        description='Path to .world file'
    )

    declare_rviz_arg = DeclareLaunchArgument(
        'rviz', default_value='false',
        description='Launch RViz2'
    )

    declare_teleop_arg = DeclareLaunchArgument(
        'teleop', default_value='false',
        description='Launch teleop_twist_keyboard'
    )

    declare_robot_id = DeclareLaunchArgument('robot_id', default_value='TFRB2')
    declare_ui_id = DeclareLaunchArgument('ui_id', default_value='TFUI2')
    declare_ws_uri = DeclareLaunchArgument('ws_uri_primary', default_value='wss://tifa-ws.forgixrobotic.com')

    # Layer 0
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')]),
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

    launch_teleop_node = Node(
        package="teleop_twist_keyboard",
        executable="teleop_twist_keyboard",
        name="teleop_twist_keyboard",
        output="screen",
        prefix="xterm -e",
        parameters=[{"stamped": False}, {"use_sim_time": True}],
        remappings=[("/cmd_vel", "/cmd_vel_key")],
        condition=IfCondition(teleop),
    )

    delayed_teleop = TimerAction(
        period=2.0, actions=[launch_teleop_node], condition=IfCondition(teleop)
    )

    twist_mux = Node(
        package='twist_mux', executable='twist_mux', name='twist_mux', output='log',
        parameters=[
            os.path.join(get_package_share_directory(package_name), 'config', 'twist_mux.yaml'),
            {'use_sim_time': True}
        ],
        remappings=[('cmd_vel_out', '/diff_cont/cmd_vel_unstamped_raw')]
    )

    safety_fail_node = Node(
        package='nevo_bot', executable='safety_fail_node', name='safety_fail_node', output='log'
    )

    rviz_config = os.path.join(
        get_package_share_directory(package_name), 'config', 'config_nevo.rviz'
    )
    rviz_node = Node(
        package="rviz2", executable="rviz2", name="rviz2",
        arguments=["-d", rviz_config],
        parameters=[{"use_sim_time": use_sim_time}],
        output="screen",
        condition=IfCondition(rviz),
    )
    delayed_rviz = TimerAction(
        period=3.0, actions=[rviz_node], condition=IfCondition(rviz)
    )

    ld = LaunchDescription()
    ld.add_action(declare_use_sim_time_arg)
    ld.add_action(declare_world_arg)
    ld.add_action(declare_rviz_arg)
    ld.add_action(declare_teleop_arg)
    ld.add_action(declare_robot_id)
    ld.add_action(declare_ui_id)
    ld.add_action(declare_ws_uri)
    ld.add_action(gazebo)
    ld.add_action(rsp)
    ld.add_action(ws_bridge)
    ld.add_action(diagnostic_node)
    ld.add_action(spawn_entity)
    ld.add_action(joint_broad_spawner)
    ld.add_action(diff_drive_spawner)
    ld.add_action(delayed_teleop)
    ld.add_action(twist_mux)
    ld.add_action(safety_fail_node)
    ld.add_action(delayed_rviz)

    return ld
