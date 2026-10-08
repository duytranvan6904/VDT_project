import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    params = os.path.join(
        get_package_share_directory('px4_state_bridge'), 'config', 'px4_state_bridge.yaml')
    return LaunchDescription([
        Node(package='px4_state_bridge', executable='px4_state_bridge_node',
             name='px4_state_bridge', parameters=[params], output='screen'),
    ])