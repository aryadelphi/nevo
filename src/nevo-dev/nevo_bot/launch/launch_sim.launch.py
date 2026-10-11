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
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, IfElseSubstitution, EnvironmentVariable
from launch.substitution import Substitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


class WorldPath(Substitution):
    """Resolve the `world` argument to an absolute gz world file path.

    Accepts all of:
      turtlebot3_house            -> <share>/nevo_bot/worlds/turtlebot3_house.world
      turtlebot3_house.world      -> <share>/nevo_bot/worlds/turtlebot3_house.world
      /abs/path/to/world.world    -> used as-is
    (relative paths with a '/' are made absolute against the launch cwd)
    """

    def perform(self, context):
        v = LaunchConfiguration('world').perform(context).strip()
        if not v.endswith('.world') and '.' not in os.path.basename(v):
            v += '.world'
        if '/' not in v:
            v = os.path.join(
                get_package_share_directory('nevo_bot'), 'worlds', v
            )
        return os.path.abspath(v)

    def describe(self):
        return 'absolute path of the gz-sim world file'


def generate_launch_description():
    package_name = 'nevo_bot'

    use_sim_time = LaunchConfiguration('use_sim_time')
    world = LaunchConfiguration('world')
    world_name = LaunchConfiguration('world_name')
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
        description='gz-sim world: a bare name from nevo_bot/worlds (e.g. '
                    "'turtlebot3_house', 'turtlebot3_world', 'empty'), a name + "
                    '.world, or a full path'
    )

    declare_world_name_arg = DeclareLaunchArgument(
        'world_name', default_value='default',
        description='<world name="..."> inside the world file, used by ros_gz_sim create'
    )

    declare_gz_headless_arg = DeclareLaunchArgument(
        'gz_headless', default_value='false',
        description='Run the gz server without its GUI (-s). Needed when the GUI '
                    'process competes for the gz-transport discovery socket, which '
                    'breaks ros_gz_sim create on hosts where UDP multicast is filtered.'
    )

    declare_rviz_arg = DeclareLaunchArgument(
        'rviz', default_value='false',
        description='Launch RViz2'
    )

    declare_mode_arg = DeclareLaunchArgument(
        'mode', default_value='navigation',
        description="diagnostic mode: 'navigation' (map+amcl+odom) or "
                    "'mapping' (odom only) — use mode:=mapping when you run "
                    "sim_mapping.launch.py in a second terminal"
    )

    declare_teleop_arg = DeclareLaunchArgument(
        'teleop', default_value='false',
        description='Launch teleop_twist_keyboard'
    )

    declare_robot_id = DeclareLaunchArgument('robot_id', default_value='TFRB2')
    declare_ui_id = DeclareLaunchArgument('ui_id', default_value='TFUI2')
    declare_ws_uri = DeclareLaunchArgument('ws_uri_primary', default_value='wss://tifa-ws.forgixrobotic.com')
    declare_nevo_extras_arg = DeclareLaunchArgument(
        'nevo_extras', default_value='false',
        description='Start the nevo cloud/diagnostic layer (nevo_ws_bridge, '
                    'nevo_diagnostic_node + cloud_upload). Default false = pure '
                    'nav2 + ros2_control sim; true needs a robot_id not held by '
                    'a live robot or another sim (else DUPLICATE_ROBOT_ID loop).'
    )
    nevo_extras = LaunchConfiguration('nevo_extras')

    # gz-sim resolves model://<name> URIs through GZ_SIM_RESOURCE_PATH only.
    # The TurtleBot3 worlds in worlds/ (turtlebot3_house, turtlebot3_world)
    # pull their scenery that way, so append this package's models/ dir
    # without clobbering a value the user already exported.
    gz_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=[
            EnvironmentVariable('GZ_SIM_RESOURCE_PATH', default_value=''),
            os.pathsep,
            os.path.join(get_package_share_directory(package_name), 'models'),
        ],
    )

    # This host filters UDP multicast on the physical NIC, so gz-transport
    # peers bound to it never discover each other: /world/<name>/create never
    # appears and `create` waits forever (README_amcl_health_sim.md §6).
    # Binding discovery to loopback keeps true multicast working. An
    # already-exported GZ_IP wins, so the workaround can be dropped once the
    # NIC multicast problem is fixed. Never use GZ_DISCOVERY_MULTICAST_IP.
    gz_discovery_ip = SetEnvironmentVariable(
        name='GZ_IP',
        value=EnvironmentVariable('GZ_IP', default_value='127.0.0.1'),
    )

    # Layer 0 — Gazebo Harmonic (gz sim).
    # ROS 2 Jazzy has no gazebo_ros: Gazebo Classic was never released for it.
    # gz_args starts the world unpaused; on_exit_shutdown tears ROS down with it.
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')]),
        launch_arguments={
            'gz_args': [
                IfElseSubstitution(
                    LaunchConfiguration('gz_headless'), '-r -s ', '-r '), WorldPath()],
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
        launch_arguments={'ws_uri_primary': LaunchConfiguration('ws_uri_primary'), 'robot_id': LaunchConfiguration('robot_id'), 'network_mode': 'auto'}.items(),
        condition=IfCondition(nevo_extras)
    )
    diagnostic_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('nevo_diagnostic_node'), 'launch', 'diagnostic.launch.py')
        ),
        launch_arguments={'mode': LaunchConfiguration('mode'), 'robot_id': 'TFRB2', 'grace_period_sec': '10', 'global_timeout_sec': '120', 'enable_stack_lock': 'false'}.items(),
        condition=IfCondition(nevo_extras)
    )

    # Replacement for gazebo_ros spawn_entity.py. Waits a few seconds so the
    # /world/default/create service is up before the first call.
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

    # gz-sim publishes on gz transport; these bridges carry it into ROS.
    # Remap keeps the Classic Gazebo point cloud name (/camera/points).
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

    # controller_manager only exists once the model is spawned and the
    # gz_ros2_control plugin has booted it, so the spawners are chained
    # behind create instead of racing it.
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

    launch_teleop_node = Node(
        package='teleop_twist_keyboard',
        executable='teleop_twist_keyboard',
        name='teleop_twist_keyboard',
        output='screen',
        prefix='xterm -e',
        parameters=[{"stamped": False}, {"use_sim_time": True}],
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
        package='nevo_bot', executable='safety_fail_node', name='safety_fail_node', output='log',
        # Stamp TwistStamped with sim time: diff_cont (use_sim_time=true via
        # controller_manager) computes cmd age as now - header.stamp; a wall-time
        # stamp makes the age permanently negative, so cmd_vel_timeout never fires.
        parameters=[{"use_sim_time": True}]
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
    ld.add_action(declare_world_name_arg)
    ld.add_action(declare_gz_headless_arg)
    ld.add_action(declare_rviz_arg)
    ld.add_action(declare_teleop_arg)
    ld.add_action(declare_mode_arg)
    ld.add_action(declare_robot_id)
    ld.add_action(declare_ui_id)
    ld.add_action(declare_ws_uri)
    ld.add_action(declare_nevo_extras_arg)
    ld.add_action(gz_resource_path)
    ld.add_action(gz_discovery_ip)
    ld.add_action(gazebo)
    ld.add_action(rsp)
    ld.add_action(ws_bridge)
    ld.add_action(diagnostic_node)
    ld.add_action(gz_bridge)
    ld.add_action(delayed_spawn)
    ld.add_action(spawn_controllers)
    ld.add_action(spawn_diff_drive)
    ld.add_action(delayed_teleop)
    ld.add_action(twist_mux)
    ld.add_action(safety_fail_node)
    ld.add_action(delayed_rviz)

    return ld
