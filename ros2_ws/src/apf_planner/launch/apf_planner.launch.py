import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    default_params = os.path.join(
        get_package_share_directory('apf_planner'), 'config', 'apf_params.yaml')
    params = LaunchConfiguration('params_file')

    return LaunchDescription([
        DeclareLaunchArgument('params_file', default_value=default_params),
        DeclareLaunchArgument('generator', default_value='false'),
        DeclareLaunchArgument('planner_type', default_value='iapf'),
        Node(
            package='apf_planner', executable='planner_node', name='apf_planner',
            output='screen',
            parameters=[params, {'planner_type': LaunchConfiguration('planner_type')}]),
        Node(
            package='apf_planner', executable='pointcloud_generator',
            name='apf_pointcloud_generator', output='screen',
            parameters=[params], condition=IfCondition(LaunchConfiguration('generator'))),
        Node(package='apf_planner', executable='planner_merge_node',
            name='planner_merge', output='screen', parameters=[params]),
    ])
