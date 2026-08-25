import os

from ament_index_python.packages import get_package_share_directory
from launch_ros.parameter_descriptions import ParameterValue
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration , Command
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node

import xacro


def generate_launch_description():

    use_sim_time = LaunchConfiguration('use_sim_time')
    sim_mode = LaunchConfiguration('sim_mode')

    pkg_path = os.path.join(get_package_share_directory('nevo_bot'))
    xacro_file = os.path.join(pkg_path,'description','robot.urdf.xacro')

    robot_description_config = ParameterValue(
        Command(['xacro ', xacro_file, ' sim_mode:=', sim_mode]),
        value_type=str
    )
 
    params = {
        'robot_description': robot_description_config,
        'use_sim_time': use_sim_time
    }

    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='log',
        parameters=[params]
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use sim time if true'
        ),
        DeclareLaunchArgument(
            'sim_mode',
            default_value='false',
            description='Use simulation hardware (Gazebo ros2_control) if true'
        ),

        node_robot_state_publisher
    ])
