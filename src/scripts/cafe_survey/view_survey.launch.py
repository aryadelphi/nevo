"""Replay a survey bag in RViz: robot model, lidar, live Kinect depth cloud,
and (optionally) the finished survey cloud.ply as a backdrop.

Laptop, ubuntu24 distrobox (needs a built nevo_bot for the robot model, e.g.
`source ~/work/diagonal_robotics/nevo_git/install/setup.bash` first):

  ros2 launch ~/work/diagonal_robotics/nevo_git_hw/src/scripts/cafe_survey/view_survey.launch.py \
      bag:=~/bags/lab_full_1026 cloud:=~/cafe_survey/lab_full_1026/cloud.ply

  rate:=2.0     play faster
  cloud:=''     replay only, no survey cloud

Space in the terminal pauses the bag. RViz fixed frame is `odom`.
"""
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction, TimerAction
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

HERE = os.path.dirname(os.path.abspath(__file__))


def setup(context):
    bag = os.path.expanduser(LaunchConfiguration('bag').perform(context))
    cloud = os.path.expanduser(LaunchConfiguration('cloud').perform(context))
    rate = LaunchConfiguration('rate').perform(context)
    sim = {'use_sim_time': True}
    actions = [
        Node(package='robot_state_publisher', executable='robot_state_publisher', output='log',
             parameters=[sim, {'robot_description': ParameterValue(Command([
                 'xacro ', FindPackageShare('nevo_bot'), '/description/robot.urdf.xacro']),
                 value_type=str)}]),
        Node(package='rviz2', executable='rviz2', output='log',
             arguments=['-d', os.path.join(HERE, 'view_survey.rviz')], parameters=[sim]),
        TimerAction(period=3.0, actions=[ExecuteProcess(
            cmd=['ros2', 'bag', 'play', bag, '--clock', '--rate', rate], output='screen')]),
    ]
    if cloud:
        actions.append(ExecuteProcess(
            cmd=['python3', os.path.join(HERE, 'ply_publisher.py'), cloud,
                 '--ros-args', '-p', 'use_sim_time:=true'], output='screen'))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('bag', description='survey bag directory'),
        DeclareLaunchArgument('cloud', default_value='', description='cloud.ply to show'),
        DeclareLaunchArgument('rate', default_value='1.0'),
        OpaqueFunction(function=setup),
    ])
