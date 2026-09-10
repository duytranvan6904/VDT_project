from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    start_hardware = LaunchConfiguration('start_hardware')
    start_servo = LaunchConfiguration('start_servo')
    debug = LaunchConfiguration('debug')

    xrce = Node(
        package='xrce_bridge_manager',
        executable='xrce_bridge_node',
        name='xrce_bridge_node',
        parameters=[{'debug_enabled': debug}],
        output='screen',
    )

    input_cache = Node(
        package='input_state_cache',
        executable='input_cache_node',
        name='input_cache_node',
        parameters=[{'debug_enabled': debug}],
        output='screen',
    )

    rc_parser = Node(
        package='rc_parser',
        executable='rc_node',
        name='rc_node',
        condition=IfCondition(start_hardware),
        parameters=[{'debug_enabled': debug}],
        output='screen',
    )

    kill_switch = Node(
        package='kill_switch',
        executable='kill_switch_node',
        name='kill_switch_node',
        condition=IfCondition(start_hardware),
        parameters=[{'debug_enabled': debug}],
        output='screen',
    )

    safety = Node(
        package='offboard_safety_monitor',
        executable='safety_monitor_node',
        name='safety_monitor_node',
        parameters=[{'debug_enabled': debug}],
        output='screen',
    )

    fsm = Node(
        package='fsm_state_machine',
        executable='fsm_node',
        name='fsm_node',
        parameters=[{'debug_enabled': debug}],
        output='screen',
    )

    gimbal = Node(
        package='gimbal_control',
        executable='gimbal_node',
        name='gimbal_node',
        parameters=[{'debug_enabled': debug}],
        output='screen',
    )

    offboard = Node(
        package='offboard_manager',
        executable='offboard_node',
        name='offboard_node',
        parameters=[{'debug_enabled': debug}],
        output='screen',
    )

    servo = Node(
        package='servo_control',
        executable='servo_node',
        name='servo_node',
        condition=IfCondition(start_servo),
        parameters=[{'debug_enabled': debug}],
        output='screen',
    )

    return LaunchDescription([
        DeclareLaunchArgument('start_hardware', default_value='true'),
        DeclareLaunchArgument('start_servo', default_value='false'),
        DeclareLaunchArgument('debug', default_value='false'),
        xrce,
        TimerAction(period=1.0, actions=[input_cache]),
        TimerAction(period=2.0, actions=[rc_parser, kill_switch]),
        TimerAction(period=3.0, actions=[safety]),
        TimerAction(period=4.0, actions=[fsm, gimbal]),
        TimerAction(period=5.0, actions=[offboard]),
        TimerAction(period=6.0, actions=[servo]),
    ])
