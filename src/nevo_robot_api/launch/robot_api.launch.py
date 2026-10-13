from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration , EnvironmentVariable
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    robot_id_arg = DeclareLaunchArgument(
        'robot_id',
        default_value='TFRB2',
        description='Robot ID handled by this API node'
    )

    ui_id_arg = DeclareLaunchArgument(
        'ui_id',
        default_value='TFRB2',
        description='UI ID handled by this API node'
    )
    
    mode_arg = DeclareLaunchArgument(
        'mode',
        default_value='nav',
        description='Initial robot mode: nav (IDLE) or mapping (MAPPING)'
    )

    map_path = DeclareLaunchArgument(
        'map_base_path',
        default_value=[EnvironmentVariable('HOME'), '/nevo_maps']
    )

    # Reject OP goals whose map_id differs from the loaded map. Default false
    # (warn only) because the dashboard currently sends map_id "100" on every OP
    # regardless of the selected map; set true once it sends the real map_id.
    enforce_map_id_arg = DeclareLaunchArgument(
        'enforce_map_id',
        default_value='false',
        description='true = reject OP on map_id mismatch, false = log a warning only'
    )

    node = Node(
        package='nevo_robot_api',
        executable='robot_api_node',
        name='nevo_robot_api',
        output='screen',
        remappings=[('/diff_cont', '/cmd_vel_remote')],
        parameters=[{
            'robot_id': LaunchConfiguration('robot_id'),
            'ui_id': LaunchConfiguration('ui_id'),
            'mode': LaunchConfiguration('mode'),
            'map_base_path':  LaunchConfiguration('map_base_path'),
            'enforce_map_id': ParameterValue(LaunchConfiguration('enforce_map_id'), value_type=bool),
        }]
    )

    return LaunchDescription([
        robot_id_arg,
        ui_id_arg,
        mode_arg,
        map_path,
        enforce_map_id_arg,
        node
    ])
