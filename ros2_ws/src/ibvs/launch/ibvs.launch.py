from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os

def generate_launch_description() -> LaunchDescription:
    # Load System_Params.yaml
    config_file = os.path.join(get_package_share_directory('vdt_bringup'), 'System_Params.yaml')
    
    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        Node(
            package="ibvs",
            executable="ibvs_controller",
            name="ibvs_controller",
            output="screen",
            parameters=[
                {"use_sim_time": LaunchConfiguration("use_sim_time")},
                config_file,
            ],
        ),
    ])