import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.substitution import Substitution
from launch.conditions import IfCondition
from launch_ros.actions import Node


class MapPath(Substitution):
    """Resolve the `map` argument to an absolute yaml path.

    Accepts all of:
      turtlebot3_save_map          -> <share>/nevo_bot/maps/turtlebot3_save_map.yaml
      turtlebot3_save_map.yaml     -> <share>/nevo_bot/maps/turtlebot3_save_map.yaml
      /abs/path/to/map.yaml        -> used as-is
    """

    def perform(self, context):
        v = LaunchConfiguration('map').perform(context).strip()
        if not v.endswith(('.yaml', '.yml')) and '.' not in os.path.basename(v):
            v += '.yaml'
        if '/' not in v:
            v = os.path.join(
                get_package_share_directory('nevo_bot'), 'maps', v
            )
        return os.path.abspath(v)

    def describe(self):
        return 'absolute path of the occupancy map yaml'


def generate_launch_description():
    package_name = 'nevo_bot'
    use_sim_time = 'true'

    declare_map_arg = DeclareLaunchArgument(
        name="map",
        default_value=os.path.join(
            get_package_share_directory(package_name), 'maps', 'turtlebot3_save_map.yaml'
        ),
        description="Map to localize on: a bare name from nevo_bot/maps "
                    "(e.g. 'turtlebot3_save_map'), a name + .yaml, or a full path"
    )
    map_file = LaunchConfiguration("map")
    declare_robot_id = DeclareLaunchArgument('robot_id', default_value='TFRB2')
    declare_ui_id = DeclareLaunchArgument('ui_id', default_value='TFUI2')
    declare_ws_uri = DeclareLaunchArgument('ws_uri_primary', default_value='wss://tifa-ws.forgixrobotic.com')
    declare_core_arg = DeclareLaunchArgument(
        'core', default_value='false',
        description='Also start Layer 0 (ws_bridge, diagnostic, safety_fail). '
                    'launch_sim.launch.py already provides all three, so the sim '
                    'layers default to false — otherwise two ws_bridge nodes fight '
                    'over the same cloud session.'
    )
    core = LaunchConfiguration('core')
    declare_nevo_extras_arg = DeclareLaunchArgument(
        'nevo_extras', default_value='false',
        description='Also start the nevo robot_api layer (robot_api, '
                    'sensor_manager). Default false = pure nav2 + ros2_control.'
    )
    nevo_extras = LaunchConfiguration('nevo_extras')

    pkg_dir = get_package_share_directory(package_name)
    nav2_params_file = os.path.join(pkg_dir, 'config', 'stvl_sim_params.yaml')
    lidar_filter_config = os.path.join(pkg_dir, 'config', 'lidar_filter.yaml')

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
        launch_arguments={'mode': 'navigation', 'robot_id': 'TFRB2', 'grace_period_sec': '10', 'global_timeout_sec': '120', 'enable_stack_lock': 'false'}.items(),
        condition=IfCondition(core)
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
        launch_arguments={'map': MapPath(), 'params_file': nav2_params_file, 'use_sim_time': use_sim_time}.items()
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

    return LaunchDescription([
        declare_map_arg, declare_core_arg, declare_robot_id, declare_ui_id, declare_ws_uri,
        declare_nevo_extras_arg,
        ws_bridge, diagnostic_node,
        TimerAction(period=1.0, actions=[laser_filter_node]),
        TimerAction(period=2.0, actions=[localization_launch]),
        TimerAction(period=5.0, actions=[initial_pose_node]),
        TimerAction(period=10.0, actions=[navigation_launch]),
        TimerAction(period=12.0, actions=[safety_fail_node]),
        TimerAction(period=14.0, actions=[robot_api, sensor_manager]),
    ])
