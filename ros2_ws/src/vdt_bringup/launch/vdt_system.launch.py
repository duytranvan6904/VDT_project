from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    start_hardware = LaunchConfiguration('start_hardware')
    start_servo = LaunchConfiguration('start_servo')
    debug = LaunchConfiguration('debug')

    xrce = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('xrce_bridge_manager'),
                'launch',
                'xrce_bridge.launch.py'
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
                'input_cache.launch.py'
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
        xrce,
        px4_state_bridge,
        vision_interface_bridge,
        TimerAction(
            period=1.0,
            actions=[input_cache]
        ),
        TimerAction(
            period=2.0,
            actions=[rc_parser, kill_switch]
        ),
        TimerAction(
            period=3.0,
            actions=[safety]
        ),
        TimerAction(
            period=4.0,
            actions=[fsm, gimbal]
        ),
        TimerAction(
            period=5.0,
            actions=[offboard]
        ),
        TimerAction(
            period=6.0,
            actions=[servo]
        )
    ])