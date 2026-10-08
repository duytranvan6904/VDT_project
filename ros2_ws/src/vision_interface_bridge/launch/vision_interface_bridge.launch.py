import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    params = os.path.join(
        get_package_share_directory('vision_interface_bridge'), 'config',
        'vision_interface_bridge.yaml')
    return LaunchDescription([
        Node(package='vision_interface_bridge', executable='vision_interface_bridge_node',
             name='vision_interface_bridge', parameters=[params], output='screen'),
    ])