import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, GroupAction, SetEnvironmentVariable,
    ExecuteProcess,
)
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import LoadComposableNodes
from launch_ros.actions import Node
from launch_ros.descriptions import ComposableNode, ParameterFile
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    bringup_dir = get_package_share_directory('nevo_bot')

    namespace = LaunchConfiguration('namespace')
    map_yaml_file = LaunchConfiguration('map')
    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file')
    use_composition = LaunchConfiguration('use_composition')
    container_name = LaunchConfiguration('container_name')
    container_name_full = (namespace, '/', container_name)
    use_respawn = LaunchConfiguration('use_respawn')
    log_level = LaunchConfiguration('log_level')

    remappings = [('/tf', 'tf'), ('/tf_static', 'tf_static')]

    param_substitutions = {
        'use_sim_time': use_sim_time,
        'yaml_filename': map_yaml_file}

    configured_params = ParameterFile(
        RewrittenYaml(
            source_file=params_file,
            root_key=namespace,
            param_rewrites=param_substitutions,
            convert_types=True),
        allow_substs=True)

    stdout_linebuf_envvar = SetEnvironmentVariable(
        'RCUTILS_LOGGING_BUFFERED_STREAM', '1')

    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace', default_value='')
    declare_map_yaml_cmd = DeclareLaunchArgument(
        'map', description='Full path to map yaml file')
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time', default_value='false')
    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(bringup_dir, 'config', 'controller_mppi_params.yaml'))
    declare_use_composition_cmd = DeclareLaunchArgument(
        'use_composition', default_value='False')
    declare_container_name_cmd = DeclareLaunchArgument(
        'container_name', default_value='nav2_container')
    declare_use_respawn_cmd = DeclareLaunchArgument(
        'use_respawn', default_value='False')
    declare_log_level_cmd = DeclareLaunchArgument(
        'log_level', default_value='info')

    load_nodes = GroupAction(
        condition=IfCondition(PythonExpression(['not ', use_composition])),
        actions=[
            Node(
                package='nav2_map_server',
                executable='map_server',
                name='map_server',
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params],
                arguments=['--ros-args', '--log-level', log_level],
                remappings=remappings),
            Node(
                package='nav2_amcl',
                executable='amcl',
                name='amcl',
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params],
                arguments=['--ros-args', '--log-level', log_level],
                remappings=remappings),
        ]
    )

    load_composable_nodes = LoadComposableNodes(
        condition=IfCondition(use_composition),
        target_container=container_name_full,
        composable_node_descriptions=[
            ComposableNode(
                package='nav2_map_server',
                plugin='nav2_map_server::MapServer',
                name='map_server',
                parameters=[configured_params],
                remappings=remappings),
            ComposableNode(
                package='nav2_amcl',
                plugin='nav2_amcl::AmclNode',
                name='amcl',
                parameters=[configured_params],
                remappings=remappings),
        ],
    )

    lifecycle_script = ExecuteProcess(
        cmd=['bash', '-c', '''
            source /opt/ros/jazzy/setup.bash
            source /home/nevo/nevo_ws/install/setup.bash 2>/dev/null || true
            export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
            export FASTRTPS_DEFAULT_PROFILES_FILE="file:///home/nevo/fastdds.xml"

            # Wait for nodes to appear on the ROS graph
            echo "=== lifecycle_activator: waiting for map_server node ==="
            for i in $(seq 1 30); do
                if ros2 node list 2>/dev/null | grep -q "/map_server"; then
                    echo "=== lifecycle_activator: map_server found after ${i}s ==="
                    break
                fi
                sleep 1
            done
            sleep 3

            # Retry helper: try a lifecycle command up to N times
            lifecycle_set() {
                local node="$1" transition="$2" label="$3"
                for attempt in 1 2 3 4 5 6 7 8; do
                    echo "=== lifecycle_activator: $label (attempt $attempt) ==="
                    OUTPUT=$(ros2 lifecycle set "$node" "$transition" 2>&1)
                    echo "$OUTPUT"
                    if echo "$OUTPUT" | grep -q "Transitioning successful"; then
                        echo "=== lifecycle_activator: $label OK ==="
                        return 0
                    fi
                    echo "=== lifecycle_activator: $label failed, retrying in 5s ==="
                    sleep 5
                done
                echo "=== lifecycle_activator: $label FAILED after 8 attempts ==="
                return 1
            }

            # Configure and activate map_server
            lifecycle_set /map_server configure "configuring map_server"
            sleep 3
            lifecycle_set /map_server activate "activating map_server"
            sleep 3

            # Configure and activate AMCL
            lifecycle_set /amcl configure "configuring amcl"
            sleep 3
            lifecycle_set /amcl activate "activating amcl"
            sleep 3

            # Publish initial pose repeatedly on a background timer
            echo "=== lifecycle_activator: publishing initial pose ==="
            (
                while true; do
                    ros2 topic pub --once /initialpose geometry_msgs/PoseWithCovarianceStamped "{
                      header: {frame_id: 'map'},
                      pose: {
                        pose: {position: {x: 0.0, y: 0.0, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}},
                        covariance: [0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0625]
                      }
                    }" 2>/dev/null
                    sleep 5
                done
            ) &
            INITIALPOSE_PID=$!

            # Wait for AMCL to actually broadcast map->odom TF
            echo "=== lifecycle_activator: waiting for map->odom TF ==="
            for i in $(seq 1 120); do
                if timeout 3 ros2 run tf2_ros tf2_echo map odom 2>/dev/null | grep -q "Translation"; then
                    echo "=== lifecycle_activator: map->odom TF confirmed after ${i}s ==="
                    break
                fi
                if [ "$i" -eq 120 ]; then
                    echo "=== lifecycle_activator: WARNING map->odom TF not confirmed after 120s ==="
                fi
                sleep 1
            done

            echo "=== lifecycle_activator: done ==="
        '''],
        output='screen'
    )

    ld = LaunchDescription()
    ld.add_action(stdout_linebuf_envvar)
    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_map_yaml_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_use_composition_cmd)
    ld.add_action(declare_container_name_cmd)
    ld.add_action(declare_use_respawn_cmd)
    ld.add_action(declare_log_level_cmd)
    ld.add_action(load_nodes)
    ld.add_action(load_composable_nodes)
    ld.add_action(lifecycle_script)

    return ld
