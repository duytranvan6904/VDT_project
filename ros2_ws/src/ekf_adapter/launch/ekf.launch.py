from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

ARGUMENT_DEFAULTS = {
    "use_sim_time": "false",
    "publish_odom_tf": "false",
    "publish_camera_tf": "false",
    "odom_topic": "/odom",
    "state_topic": "/hpad/state_filtered",
    "world_frame": "world",
    "base_frame": "base_link",
    "camera_frame": "camera_optical_frame",
    "cam_x": "0.0",
    "cam_y": "0.0",
    "cam_z": "0.0",
    "cam_roll": "-1.5708",
    "cam_pitch": "0.0",
    "cam_yaw": "-1.5708",
}


def generate_launch_description() -> LaunchDescription:
    declarations = [
        DeclareLaunchArgument(name, default_value=value) for name, value in ARGUMENT_DEFAULTS.items()
    ]
    
    # Load System_Params.yaml
    config_file = os.path.join(get_package_share_directory('vdt_bringup'), 'System_Params.yaml')
    
    use_sim_time = {"use_sim_time": LaunchConfiguration("use_sim_time")}
    
    odom_tf = Node(
        package="ekf_adapter",
        executable="odom_tf_node",
        parameters=[
            use_sim_time,
            config_file,
            {
                "odom_topic": LaunchConfiguration("odom_topic"),
                "world_frame": LaunchConfiguration("world_frame"),
                "base_frame": LaunchConfiguration("base_frame"),
            },
        ],
        condition=IfCondition(LaunchConfiguration("publish_odom_tf")),
    )
    
    camera_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=[
            "--x", LaunchConfiguration("cam_x"),
            "--y", LaunchConfiguration("cam_y"),
            "--z", LaunchConfiguration("cam_z"),
            "--roll", LaunchConfiguration("cam_roll"),
            "--pitch", LaunchConfiguration("cam_pitch"),
            "--yaw", LaunchConfiguration("cam_yaw"),
            "--frame-id", LaunchConfiguration("base_frame"),
            "--child-frame-id", LaunchConfiguration("camera_frame"),
        ],
        condition=IfCondition(LaunchConfiguration("publish_camera_tf")),
    )
    
    ekf = Node(
        package="ekf_adapter",
        executable="ekf_node",
        parameters=[
            use_sim_time,
            config_file,
            {
                "target_frame": LaunchConfiguration("world_frame"),
                "state_topic": LaunchConfiguration("state_topic"),
            },
        ],
        output="screen",
    )
    
    return LaunchDescription(declarations + [odom_tf, camera_tf, ekf])