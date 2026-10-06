from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    start_servo = LaunchConfiguration('start_servo')
    debug = LaunchConfiguration('debug')

    params_file = PathJoinSubstitution([
        FindPackageShare('vdt_bringup'),
        'System_Params.yaml'
    ])

    servo = Node(
        package='servo_control',
        executable='servo_node',
        name='servo_node',
        condition=IfCondition(start_servo),
        parameters=[params_file, {'debug_enabled': debug}],
        output='screen'
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'start_servo',
            default_value='false'
        ),
        DeclareLaunchArgument(
            'debug',
            default_value='false'
        ),
        servo
    ])