"""
Launch file for nevo_diagnostic_node.

Usage:
  ros2 launch nevo_diagnostic_node diagnostic.launch.py
  ros2 launch nevo_diagnostic_node diagnostic.launch.py robot_id:=RB002
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    mode               = LaunchConfiguration('mode',               default='navigation')
    robot_id           = LaunchConfiguration('robot_id',           default='TFRB2')
    grace_period_sec   = LaunchConfiguration('grace_period_sec',   default='10')
    global_timeout_sec = LaunchConfiguration('global_timeout_sec', default='60')

    # ── Launch arguments (can be overridden on CLI) ────────────────────────
    robot_id_arg = DeclareLaunchArgument(
        "robot_id",
        default_value="TFRB2",
        description="Robot identifier published in JSON status messages",
    )

    ui_id_arg = DeclareLaunchArgument(
        "ui_id",
        default_value="TFUI2",
        description="UI identifier published in JSON status messages",
    )

    grace_period_arg = DeclareLaunchArgument(
        "grace_period_sec",
        default_value="10",
        description="Seconds to ignore all topics after node start",
    )

    global_timeout_arg = DeclareLaunchArgument(
        "global_timeout_sec",
        default_value="60",
        description="Seconds before declaring boot FAILED if not READY",
    )

    odom_timeout_arg = DeclareLaunchArgument(
        "odom_timeout_sec",
        default_value="3.0",
        description="Seconds without /odometry/filtered before stale warning/fail",
    )

    # ── Config file path ───────────────────────────────────────────────────
    config_file = PathJoinSubstitution([
        FindPackageShare("nevo_diagnostic_node"),
        "config",
        "diagnostic_params.yaml",
    ])

    # ── Node ───────────────────────────────────────────────────────────────
    diagnostic_node = Node(
        package="nevo_diagnostic_node",
        executable="diagnostic_node",
        name="nevo_diagnostic_node",
        output="screen",
        emulate_tty=True,
        # params file provides defaults; CLI arguments override individual keys
        parameters=[
            config_file,
            {
                "robot_id":           LaunchConfiguration("robot_id"),
                "ui_id":              LaunchConfiguration("ui_id"),
                'mode':               mode,
                "grace_period_sec":   LaunchConfiguration("grace_period_sec"),
                "global_timeout_sec": LaunchConfiguration("global_timeout_sec"),
                "odom_timeout_sec":   LaunchConfiguration("odom_timeout_sec"),
            },
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument('mode',               default_value='navigation'),
        DeclareLaunchArgument('robot_id',           default_value='TFRB2'),
        DeclareLaunchArgument('grace_period_sec',   default_value='10'),
        DeclareLaunchArgument('global_timeout_sec', default_value='60'),
        robot_id_arg,
        ui_id_arg,
        grace_period_arg,
        global_timeout_arg,
        odom_timeout_arg,
        diagnostic_node,
    ])
