from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def include(pkg, name, args=None):
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution(
            [FindPackageShare(pkg), 'launch', name])),
        launch_arguments=(args or {}).items())


def log_runtime_config(context, *args, **kwargs):
    servo = context.launch_configurations.get('start_servo', 'true')
    print("=" * 65, flush=True)
    print("[VDT_FULL] >>> FULL SYSTEM LAUNCH CONFIGURATION EVALUATED:", flush=True)
    print(f"[VDT_FULL]   - start_servo : {servo}", flush=True)
    print("[VDT_FULL]   - Packages: bringup (vdt_system), aruco_detector, ekf_adapter, ibvs, apf_planner, landing_guidance", flush=True)
    print("=" * 65, flush=True)
    return []


def generate_launch_description():
    print("\n[VDT_FULL] [INIT] Generating LaunchDescription for FULL system (embedded + vision + planner)...", flush=True)
    return LaunchDescription([
        DeclareLaunchArgument('start_servo', default_value='true'),
        OpaqueFunction(function=log_runtime_config),
        include('bringup', 'vdt_system.launch.py',
                {'start_servo': LaunchConfiguration('start_servo')}),
        include('aruco_detector', 'aruco.launch.py', {'mode': 'hw'}),
        include('ekf_adapter', 'ekf.launch.py',
                {'publish_odom_tf': 'false', 'publish_camera_tf': 'false'}),
        include('ibvs', 'ibvs.launch.py'),
        include('apf_planner', 'apf_planner.launch.py',
                {'planner_type': 'iapf', 'depth_map': 'true'}),
        include('landing_guidance', 'landing.launch.py'),
    ])
