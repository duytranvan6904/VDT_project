import math
import os
import tempfile
import time

import launch
import launch.actions
import launch_pytest
import numpy as np
import pytest
import rclpy
import rosbag2_py
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource
from nav_msgs.msg import Odometry
from rclpy.serialization import serialize_message
from std_msgs.msg import String

try:
    from vdt_msgs.msg import PlannerOutput
except ImportError:
    from offboard_manager.msg import PlannerOutput

BAG_DELAY = 6.0
BASE_NS = 1_000_000_000
ODOM_HZ = 20
DRONE = (-8.0, -8.0, 3.0)
TARGET = (8.0, 8.0, 0.0)
PHASES = [
    (2.0, 'IDLE', 'LOST'),
    (3.0, 'FOLLOW', 'TRACKING'),
    (1.5, 'FOLLOW', 'EXPIRED'),
    (2.0, 'APPROACH', 'TRACKING'),
    (1.5, 'IDLE', 'LOST'),
]
BAG_DURATION = sum(p[0] for p in PHASES)

TOPICS = [
    (0, '/odom', 'nav_msgs/msg/Odometry'),
    (1, '/hpad/state_filtered', 'nav_msgs/msg/Odometry'),
    (2, '/ekf/tracking_mode', 'std_msgs/msg/String'),
    (3, '/mission/phase', 'std_msgs/msg/String'),
]


def odom_msg(pos):
    m = Odometry()
    m.header.frame_id = 'world'
    m.pose.pose.position.x, m.pose.pose.position.y, m.pose.pose.position.z = pos
    m.pose.pose.orientation.w = 1.0
    return m


def string_msg(text):
    m = String()
    m.data = text
    return m


def create_topic(writer, index, name, type_name):
    try:
        meta = rosbag2_py.TopicMetadata(name=name, type=type_name, serialization_format='cdr')
    except TypeError:
        meta = rosbag2_py.TopicMetadata(
            id=index, name=name, type=type_name, serialization_format='cdr')
    writer.create_topic(meta)


def build_bag(path):
    writer = rosbag2_py.SequentialWriter()
    writer.open(
        rosbag2_py.StorageOptions(uri=path, storage_id='sqlite3'),
        rosbag2_py.ConverterOptions('', ''))
    for index, name, type_name in TOPICS:
        create_topic(writer, index, name, type_name)
    period = int(1e9 / ODOM_HZ)
    t_ns = BASE_NS
    for duration, phase, mode in PHASES:
        for step in range(int(duration * ODOM_HZ)):
            writer.write('/odom', serialize_message(odom_msg(DRONE)), t_ns)
            writer.write('/hpad/state_filtered', serialize_message(odom_msg(TARGET)), t_ns)
            if step % 2 == 0:
                writer.write('/mission/phase', serialize_message(string_msg(phase)), t_ns)
                writer.write('/ekf/tracking_mode', serialize_message(string_msg(mode)), t_ns)
            t_ns += period
    del writer


@launch_pytest.fixture
def generate_test_description():
    bag_dir = os.path.join(tempfile.mkdtemp(), 'apf_bag')
    build_bag(bag_dir)
    share = get_package_share_directory('apf_planner')
    include = launch.actions.IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(share, 'launch', 'apf_planner.launch.py')),
        launch_arguments={'planner_type': 'apf', 'generator': 'false'}.items())
    play = launch.actions.ExecuteProcess(cmd=['ros2', 'bag', 'play', bag_dir], output='screen')
    return launch.LaunchDescription([
        include,
        launch.actions.TimerAction(period=BAG_DELAY, actions=[play]),
        launch_pytest.actions.ReadyToTest(),
    ])


def fraction(items, predicate):
    return sum(1 for i in items if predicate(i)) / len(items) if items else 0.0


@pytest.mark.launch(fixture=generate_test_description)
def test_planner_pipeline_from_bag():
    rclpy.init()
    node = rclpy.create_node('apf_rosbag_probe')
    state = {'phase': 'NONE', 'mode': 'NONE'}
    records = []

    def on_phase(m):
        state['phase'] = m.data

    def on_mode(m):
        state['mode'] = m.data

    def on_out(m):
        records.append((state['phase'], state['mode'], m.vx, m.vy, m.vz, m.yaw))

    node.create_subscription(String, '/mission/phase', on_phase, 20)
    node.create_subscription(String, '/ekf/tracking_mode', on_mode, 20)
    node.create_subscription(PlannerOutput, '/planner/velocity_setpoint', on_out, 100)
    end = time.time() + BAG_DELAY + BAG_DURATION + 4.0
    while time.time() < end:
        rclpy.spin_once(node, timeout_sec=0.05)
    node.destroy_node()
    rclpy.shutdown()

    assert len(records) >= 100

    idle = [r for r in records if r[0] == 'IDLE']
    assert idle
    assert fraction(idle, lambda r: abs(r[2]) + abs(r[3]) + abs(r[4]) < 1e-9) >= 0.9

    follow = [r for r in records if r[0] == 'FOLLOW' and r[1] == 'TRACKING']
    assert follow
    assert fraction(follow, lambda r: math.hypot(r[2], r[3]) > 0.1) >= 0.7
    assert np.median([r[2] for r in follow]) > 0.2
    assert np.median([r[3] for r in follow]) > 0.2
    assert fraction(follow, lambda r: math.isfinite(r[5]) and abs(r[5] - math.pi / 4) < 0.05) >= 0.7

    expired = [r for r in records if r[0] == 'FOLLOW' and r[1] == 'EXPIRED']
    assert expired
    assert fraction(expired, lambda r: abs(r[2]) + abs(r[3]) + abs(r[4]) < 1e-9) >= 0.8

    approach = [r for r in records if r[0] == 'APPROACH' and r[1] == 'TRACKING']
    assert approach
    assert fraction(approach, lambda r: r[2] > 0 and r[3] > 0 and r[4] < -0.05) >= 0.7