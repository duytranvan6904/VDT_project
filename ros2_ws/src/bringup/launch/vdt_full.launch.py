from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def include(pkg, name, args=None):
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution(
            [FindPackageShare(pkg), 'launch', name])),
        launch_arguments=(args or {}).items())


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('start_servo', default_value='true'),
        include('bringup', 'vdt_system.launch.py',
                {'start_servo': LaunchConfiguration('start_servo')}),
        include('aruco_detector', 'aruco.launch.py', {'mode': 'hw'}),
        include('ekf_adapter', 'ekf.launch.py',
                {'publish_odom_tf': 'false', 'publish_camera_tf': 'false'}),
        include('ibvs', 'ibvs.launch.py'),
        include('apf_planner', 'apf_planner.launch.py',
                {'planner_type': 'iapf'}),
        include('landing_guidance', 'landing.launch.py'),
    ])
