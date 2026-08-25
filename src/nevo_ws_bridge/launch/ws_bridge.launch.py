from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.actions import DeclareLaunchArgument, TimerAction, ExecuteProcess, LogInfo
from launch.substitutions import LaunchConfiguration, IfElseSubstitution, NotSubstitution
from launch.conditions import IfCondition
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessStart

import os
import shutil


def generate_launch_description():
    pkg_share = FindPackageShare('nevo_ws_bridge').find('nevo_ws_bridge')
    default_params = os.path.join(pkg_share, 'config', 'ws_bridge_params.yaml')

    ws_uri_arg = DeclareLaunchArgument(
        'ws_uri_primary',
        default_value='ws://localhost:3001',
        description='Primary WebSocket URI'
    )

    robot_id_arg = DeclareLaunchArgument(
        'robot_id',
        default_value='TFRB2',
        description='Robot ID'
    )

    network_mode = DeclareLaunchArgument(
        'network_mode',
        default_value='auto',
        description='Network mode'
    )

    node = Node(
        package='nevo_ws_bridge',
        executable='ws_bridge_node',
        name='nevo_ws_bridge',
        output='screen',
        parameters=[
            default_params,
            {
                'ws_uri_primary': LaunchConfiguration('ws_uri_primary'),
                'robot_id': LaunchConfiguration('robot_id'),
                'network_mode': LaunchConfiguration('network_mode'),
                'enable_local_ws': True,
                'local_ws_port': 8765
            }
        ]
    )

    node_upload_sensor = Node(
        package='nevo_ws_bridge',
        executable='cloud_upload',
        name='cloud_upload',
        output='screen',
    )

    delayed_cloud = TimerAction(
        period=5.0,
        actions=[node]
    )

    # ADB reverse proxy — only if adb is installed
    adb_available = shutil.which('adb') is not None

    adb_kill = ExecuteProcess(
        cmd=['pkill', 'adb'],
        output='screen',
        condition=IfCondition('true' if adb_available else 'false')
    )

    adb_reverse = ExecuteProcess(
        cmd=['adb', 'reverse', 'tcp:8765', 'tcp:8765'],
        output='screen',
        condition=IfCondition('true' if adb_available else 'false')
    )

    delayed_adb_reverse = TimerAction(
        period=5.0,
        actions=[adb_reverse],
        condition=IfCondition('true' if adb_available else 'false')
    )

    return LaunchDescription([
        ws_uri_arg,
        robot_id_arg,
        adb_kill,
        delayed_adb_reverse,
        delayed_cloud,
        network_mode,
        node_upload_sensor,
    ])
