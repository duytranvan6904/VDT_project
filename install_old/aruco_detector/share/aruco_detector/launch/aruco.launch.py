from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

MODE_PARAMETERS = {
    "sim": {
        "marker_size_m": 0.895,
        "min_detection_distance_m": 2.2,
        "min_z_m": 1.8,
        "image_topic": "/camera",
        "camera_info_topic": "/camera_info",
    },
    "hw": {
        "marker_size_m": 0.15,
        "min_detection_distance_m": 0.0,
        "min_z_m": 0.0,
        "image_topic": "/camera/infra1/image_rect_raw",
        "camera_info_topic": "/camera/infra1/camera_info",
    },
}


def read_launch_argument(context, name: str) -> str:
    return LaunchConfiguration(name).perform(context)


def build_parameters(context) -> dict:
    mode = read_launch_argument(context, "mode")
    if mode not in MODE_PARAMETERS:
        raise RuntimeError(f"mode must be sim or hw, got: {mode}")
    
    # Load System_Params.yaml
    config_file = os.path.join(
        os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(__file__)))),
        'System_Params.yaml'
    )
    
    parameters = dict(MODE_PARAMETERS[mode])
    parameters["marker_id"] = int(read_launch_argument(context, "marker_id"))
    
    # Return both config file and overrides
    return [config_file, parameters]


def build_aruco_node(context) -> Node:
    return Node(
        package="aruco_detector",
        executable="aruco_node",
        name="aruco_node",
        output="screen",
        parameters=build_parameters(context),
    )


def build_depth_preview_node() -> Node:
    return Node(
        package="aruco_detector",
        executable="depth_to_image_node",
        name="depth_to_image_node",
        output="screen",
    )


def build_nodes(context) -> list:
    nodes = [build_aruco_node(context)]
    if read_launch_argument(context, "mode") == "sim":
        nodes.append(build_depth_preview_node())
    return nodes


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription([
        DeclareLaunchArgument("mode", default_value="sim"),
        DeclareLaunchArgument("marker_id", default_value="42"),
        OpaqueFunction(function=build_nodes),
    ])