import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    default_params = os.path.join(
        get_package_share_directory('bringup'), 'System_Params.yaml'
    )
    params_file = LaunchConfiguration('params_file')

    def node(executable, name):
        return Node(
            package='landing_guidance',
            executable=executable,
            name=name,
            output='screen',
            parameters=[params_file],
        )

    return LaunchDescription([
        DeclareLaunchArgument('params_file', default_value=default_params),
        node('covariance_gate_node', 'covariance_gate_node'),
        node('smc_guidance_node', 'smc_guidance_node'),
        node('touchdown_detector_node', 'touchdown_detector_node'),
    ])