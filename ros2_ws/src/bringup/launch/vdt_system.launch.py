from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    RegisterEventHandler,
    TimerAction,
)
from launch.event_handlers import OnProcessExit, OnProcessStart
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def log_runtime_config(context, *args, **kwargs):
    hw = context.launch_configurations.get('start_hardware', 'true')
    servo = context.launch_configurations.get('start_servo', 'false')
    dbg = context.launch_configurations.get('debug', 'false')
    print("=" * 65, flush=True)
    print("[VDT_SYSTEM] >>> SYSTEM NODES LAUNCH CONFIGURATION EVALUATED:", flush=True)
    print(f"[VDT_SYSTEM]   - start_hardware : {hw}", flush=True)
    print(f"[VDT_SYSTEM]   - start_servo    : {servo}", flush=True)
    print(f"[VDT_SYSTEM]   - debug (verbose): {dbg}", flush=True)
    print("[VDT_SYSTEM] =====================================================", flush=True)
    return []


def log_step(message):
    def _action(context, *args, **kwargs):
        print(f"[VDT_SYSTEM] {message}", flush=True)
        return []
    return OpaqueFunction(function=_action)


def on_node_start(event, context):
    pname = getattr(event, 'process_name', None) or getattr(getattr(event, 'action', None), 'name', 'unknown_node')
    pid = getattr(event, 'pid', 'N/A')
    print(f"[VDT_SYSTEM VERBOSE] [NODE STARTED] Process: {pname} | PID: {pid}", flush=True)


def on_node_exit(event, context):
    pname = getattr(event, 'process_name', None) or getattr(getattr(event, 'action', None), 'name', 'unknown_node')
    code = getattr(event, 'returncode', 'N/A')
    print(f"[VDT_SYSTEM VERBOSE] [NODE EXITED]  Process: {pname} | Return Code: {code}", flush=True)


def generate_launch_description():
    print("\n[VDT_SYSTEM] [INIT] Generating LaunchDescription for VDT system packages...", flush=True)

    start_hardware = LaunchConfiguration('start_hardware')
    start_servo = LaunchConfiguration('start_servo')
    debug = LaunchConfiguration('debug')

    xrce = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('xrce_bridge_manager'),
                'launch',
                'xrce_bringe_manager.launch.py'
            ])
        ),
        launch_arguments={
            'debug': debug
        }.items()
    )

    px4_state_bridge = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('px4_state_bridge'),
                'launch',
                'px4_state_bridge.launch.py'
            ])
        )
    )

    vision_interface_bridge = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('vision_interface_bridge'),
                'launch',
                'vision_interface_bridge.launch.py'
            ])
        )
    )

    input_cache = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('input_state_cache'),
                'launch',
                'input_state_cache.launch.py'
            ])
        ),
        launch_arguments={
            'debug': debug
        }.items()
    )

    rc_parser = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('rc_parser'),
                'launch',
                'rc_parser.launch.py'
            ])
        ),
        launch_arguments={
            'start_hardware': start_hardware,
            'debug': debug
        }.items()
    )

    kill_switch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('kill_switch'),
                'launch',
                'kill_switch.launch.py'
            ])
        ),
        launch_arguments={
            'start_hardware': start_hardware,
            'debug': debug
        }.items()
    )

    safety = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('offboard_safety_monitor'),
                'launch',
                'offboard_safety_monitor.launch.py'
            ])
        ),
        launch_arguments={
            'debug': debug
        }.items()
    )

    fsm = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('fsm_state_machine'),
                'launch',
                'fsm.launch.py'
            ])
        ),
        launch_arguments={
            'debug': debug
        }.items()
    )

    gimbal = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('gimbal_control'),
                'launch',
                'gimbal.launch.py'
            ])
        ),
        launch_arguments={
            'debug': debug
        }.items()
    )

    offboard = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('offboard_manager'),
                'launch',
                'offboard_manager.launch.py'
            ])
        ),
        launch_arguments={
            'debug': debug
        }.items()
    )

    servo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('servo_control'),
                'launch',
                'servo_control.launch.py'
            ])
        ),
        launch_arguments={
            'start_servo': start_servo,
            'debug': debug
        }.items()
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'start_hardware',
            default_value='true'
        ),
        DeclareLaunchArgument(
            'start_servo',
            default_value='false'
        ),
        DeclareLaunchArgument(
            'debug',
            default_value='false'
        ),
        OpaqueFunction(function=log_runtime_config),
        RegisterEventHandler(
            OnProcessStart(on_start=on_node_start)
        ),
        RegisterEventHandler(
            OnProcessExit(on_exit=on_node_exit)
        ),
        log_step("[T+0.0s] Triggering base bridges: xrce_bridge_manager, px4_state_bridge, vision_interface_bridge"),
        xrce,
        px4_state_bridge,
        vision_interface_bridge,
        TimerAction(
            period=1.0,
            actions=[
                log_step("[T+1.0s] Triggering state cache: input_state_cache"),
                input_cache
            ]
        ),
        TimerAction(
            period=2.0,
            actions=[
                log_step("[T+2.0s] Triggering hardware inputs: rc_parser, kill_switch"),
                rc_parser,
                kill_switch
            ]
        ),
        TimerAction(
            period=3.0,
            actions=[
                log_step("[T+3.0s] Triggering safety monitor: offboard_safety_monitor"),
                safety
            ]
        ),
        TimerAction(
            period=4.0,
            actions=[
                log_step("[T+4.0s] Triggering high-level decision nodes: fsm_state_machine, gimbal_control"),
                fsm,
                gimbal
            ]
        ),
        TimerAction(
            period=5.0,
            actions=[
                log_step("[T+5.0s] Triggering offboard actuator manager: offboard_manager"),
                offboard
            ]
        ),
        TimerAction(
            period=6.0,
            actions=[
                log_step("[T+6.0s] Triggering optional servo control: servo_control"),
                servo
            ]
        )
    ])