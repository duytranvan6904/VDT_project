import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Load System_Params.yaml
    config_file = os.path.join(
        os.path.dirname(os.path.dirname(os.path.dirname(__file__))),
        '..', '..', '..',
        'System_Params.yaml'
    )
    
    return LaunchDescription([
        DeclareLaunchArgument('generator', default_value='false'),
        DeclareLaunchArgument('planner_type', default_value='iapf'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        
        Node(
            package='apf_planner', 
            executable='planner_node', 
            name='apf_planner',
            output='screen',
            parameters=[
                {"use_sim_time": LaunchConfiguration("use_sim_time")},
                config_file,
                {'planner_type': LaunchConfiguration('planner_type')}
            ]),
        
        Node(
            package='apf_planner', 
            executable='pointcloud_generator',
            name='apf_pointcloud_generator', 
            output='screen',
            parameters=[
                {"use_sim_time": LaunchConfiguration("use_sim_time")},
                config_file
            ], 
            condition=IfCondition(LaunchConfiguration('generator'))),
        
        Node(
            package='apf_planner', 
            executable='planner_merge_node',
            name='planner_merge', 
            output='screen', 
            parameters=[
                {"use_sim_time": LaunchConfiguration("use_sim_time")},
                config_file
            ]),
    ])