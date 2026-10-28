"""Offline cafe survey: bag -> RTAB-Map (lidar SLAM) + 3D Kinect cloud (PLY).

Runs on the laptop (ubuntu24 distrobox, Jazzy + ros-jazzy-rtabmap-ros), no
build needed:

  ros2 launch src/scripts/cafe_survey/survey_rtabmap.launch.py \
      bag:=~/bags/cafe_survey_2026-10-26_1400 out:=~/cafe_survey/

Outputs in `out`: rtabmap.db and cloud.ply (map frame = the odom frame at the
bag's first scan, i.e. where the robot stack was launched).
The robot's recorded map->odom (AMCL / slam_toolbox) is filtered out by
survey_cloud.py so RTAB-Map owns map->odom.

No localization-mode second pass: lidar-only RTAB-Map has no visual words
to relocalize with and starts from a wrong pose (tested on a sim bag,
2026-10-25). Depth frames already wait for the loop-closed map->odom.
"""
import os
import re

from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess, OpaqueFunction,
                            TimerAction)
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

HERE = os.path.dirname(os.path.abspath(__file__))


def setup(context):
    bag = os.path.expanduser(LaunchConfiguration('bag').perform(context))
    out = os.path.expanduser(LaunchConfiguration('out').perform(context))
    rate = LaunchConfiguration('rate').perform(context)
    show = LaunchConfiguration('rviz').perform(context).lower() == 'true'
    use_bag_slam = LaunchConfiguration('slam').perform(context) == 'bag'
    scan = LaunchConfiguration('scan').perform(context)
    os.makedirs(out, exist_ok=True)
    db = os.path.join(out, 'rtabmap.db')

    params = {
        'use_sim_time': True,
        'frame_id': 'base_link',
        'odom_frame_id': 'odom',          # EKF odom->base_link from the bag
        'map_frame_id': 'map',
        'publish_tf': True,
        'subscribe_depth': False,
        'subscribe_rgb': False,
        'subscribe_rgbd': False,
        'subscribe_scan': True,
        'database_path': db,
        # lidar-only graph SLAM, 2D
        'Reg/Strategy': '1',              # ICP
        'Reg/Force3DoF': 'true',
        'RGBD/NeighborLinkRefining': 'true',
        'RGBD/ProximityBySpace': 'true',  # no camera words -> loops by proximity
        'RGBD/AngularUpdate': '0.05',
        'RGBD/LinearUpdate': '0.05',
        'Rtabmap/DetectionRate': '0',     # process every update
        'Kp/MaxFeatures': '-1',           # no visual words: lidar-only
        'Icp/VoxelSize': '0.05',
        'Icp/MaxCorrespondenceDistance': '0.1',
        'Icp/PointToPlane': 'true',
        'Icp/PointToPlaneK': '20',
        'Icp/PointToPlaneRadius': '0',
        'Icp/Iterations': '10',
        'Icp/Epsilon': '0.001',
        'Grid/Sensor': '0',               # 2D grid from the laser
        'Grid/RangeMax': '12',
    }
    rtabmap = Node(
        package='rtabmap_slam', executable='rtabmap', name='rtabmap', output='screen',
        parameters=[params],
        remappings=[('scan', scan)],
        arguments=['-d'],                 # start a fresh database
    )
    cloud = ExecuteProcess(
        cmd=['python3', os.path.join(HERE, 'survey_cloud.py'),
             '--out', os.path.join(out, 'cloud.ply'),
             '--max-range', LaunchConfiguration('max_range').perform(context),
             '--depth-topic', LaunchConfiguration('depth_topic').perform(context),
             '--info-topic', LaunchConfiguration('info_topic').perform(context),
             '--cam-pitch-deg', LaunchConfiguration('cam_pitch').perform(context),
             '--cam-yaw-deg', LaunchConfiguration('cam_yaw').perform(context),
             '--cam-fwd', LaunchConfiguration('cam_fwd').perform(context),
             '--depth-scale', LaunchConfiguration('depth_scale').perform(context),
             '--depth-offset', LaunchConfiguration('depth_offset').perform(context),
             *(['--live-topic', '/survey_cloud_live'] if show else []),
             '--ros-args', '-p', 'use_sim_time:=true'],
        output='screen',
    )
    # --clock: /clock from the bag's receive times (= header stamps on the
    # robot). clock:=false for sim bags, which carry their own /clock.
    clock = ['--clock'] if LaunchConfiguration('clock').perform(context).lower() == 'true' else []
    # until:=<s from bag start>: e.g. stop where the map was saved, before a bad stretch
    until = float(LaunchConfiguration('until').perform(context) or -1)
    stop = []
    if until > 0:
        meta = open(os.path.join(bag, 'metadata.yaml')).read()
        t0 = int(re.search(r'starting_time:\s*\n\s*nanoseconds_since_epoch:\s*(\d+)', meta).group(1))
        stop = ['--playback-until-sec', '%.3f' % (t0 * 1e-9 + until)]
    play = ExecuteProcess(
        cmd=['ros2', 'bag', 'play', bag, *clock, '--rate', rate, *stop,
             *([] if use_bag_slam else ['--remap', '/tf:=/tf_bag'])],
        output='screen',
    )
    # slam:=bag uses the map->odom that SLAM Toolbox published live on the robot
    # (recorded in /tf) instead of re-estimating poses with RTAB-Map. Better when
    # the live map was good: RTAB-Map lidar-only can drift in long corridors.
    actions = ([] if use_bag_slam else [rtabmap]) + [cloud, TimerAction(period=4.0, actions=[play])]
    if show:  # watch the cloud being built (needs nevo_bot sourced for the robot model)
        actions.append(Node(package='rviz2', executable='rviz2', output='log',
                            arguments=['-d', os.path.join(HERE, 'build_survey.rviz')],
                            parameters=[{'use_sim_time': True}]))
        try:
            from ament_index_python.packages import get_package_share_directory
            xacro_file = os.path.join(get_package_share_directory('nevo_bot'),
                                      'description', 'robot.urdf.xacro')
            actions.append(Node(
                package='robot_state_publisher', executable='robot_state_publisher',
                output='log', parameters=[{'use_sim_time': True, 'robot_description':
                                           ParameterValue(Command(['xacro ', xacro_file]),
                                                          value_type=str)}]))
        except Exception:
            pass  # no nevo_bot sourced: no robot model, everything else works
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('bag', description='survey bag directory'),
        DeclareLaunchArgument('out', default_value='~/cafe_survey'),
        DeclareLaunchArgument('rate', default_value='1.0'),
        DeclareLaunchArgument('slam', default_value='rtabmap',
                              description="rtabmap (re-estimate poses) or bag (robot's own "
                                          'SLAM Toolbox map->odom from the recording)'),
        DeclareLaunchArgument('until', default_value='-1',
                              description='stop playback this many s after the bag start'),
        DeclareLaunchArgument('clock', default_value='true',
                              description='false for sim bags that recorded /clock'),
        DeclareLaunchArgument('scan', default_value='/scan_filtered'),
        DeclareLaunchArgument('max_range', default_value='3.0',
                              description='Kinect depth cut-off in m'),
        DeclareLaunchArgument('depth_topic', default_value='/depth/image_raw'),
        DeclareLaunchArgument('info_topic', default_value='/depth/camera_info'),
        # Mount corrections for bags recorded before the URDF was calibrated
        # (2026-10-25): cam_pitch:=2.45 cam_yaw:=2.0 cam_fwd:=0.035. Newer bags: 0.
        DeclareLaunchArgument('cam_pitch', default_value='0.0',
                              description='Kinect tilt missing from the URDF, deg, + = down'),
        DeclareLaunchArgument('cam_yaw', default_value='0.0',
                              description='Kinect yaw missing from the URDF, deg, + = left'),
        DeclareLaunchArgument('cam_fwd', default_value='0.0',
                              description='m the Kinect really sits ahead of the URDF pose'),
        # nevo's Kinect, wall calibration 2026-10-25. Sim bags: depth_scale:=1 depth_offset:=0
        DeclareLaunchArgument('depth_scale', default_value='1.0436',
                              description='Kinect depth calibration: z = scale*z + offset'),
        DeclareLaunchArgument('depth_offset', default_value='-0.026', description='m'),
        DeclareLaunchArgument('rviz', default_value='false'),
        OpaqueFunction(function=setup),
    ])
