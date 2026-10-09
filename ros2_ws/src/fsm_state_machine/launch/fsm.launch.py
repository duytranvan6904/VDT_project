from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    debug = LaunchConfiguration('debug')

    params_file = PathJoinSubstitution([
        FindPackageShare('bringup'),
        'System_Params.yaml'
    ])

    fsm = Node(
        package='fsm_state_machine',
        executable='fsm_node',
        name='fsm_node',
        parameters=[params_file, {'debug_enabled': debug}],
        output='screen'
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'debug',
            default_value='false'
        ),
        fsm
    ])