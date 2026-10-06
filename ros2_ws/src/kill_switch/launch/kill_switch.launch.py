from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    start_hardware = LaunchConfiguration('start_hardware')
    debug = LaunchConfiguration('debug')

    params_file = PathJoinSubstitution([
        FindPackageShare('vdt_bringup'),
        'System_Params.yaml'
    ])

    kill_switch = Node(
        package='kill_switch',
        executable='kill_switch_node',
        name='kill_switch_node',
        condition=IfCondition(start_hardware),
        parameters=[params_file, {'debug_enabled': debug}],
        output='screen'
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'start_hardware',
            default_value='true'
        ),
        DeclareLaunchArgument(
            'debug',
            default_value='false'
        ),
        kill_switch
    ])