import os
import tempfile
import time

import launch
import launch.actions
import launch_pytest
import launch_ros.actions
import numpy as np
import pytest
import rclpy
import rosbag2_py
from geometry_msgs.msg import PointStamped
from rclpy.serialization import serialize_message
from sensor_msgs.msg import Image
from std_msgs.msg import Bool

from marker_utils import make_camera_info, make_frame, make_image_msg

BAG_DELAY = 5.0
BASE_NS = 1_000_000_000
FPS = 10
PHASES = [
    (1.5, None),
    (2.0, 1.0),
    (2.0, 1.5),
    (1.0, None),
]
BAG_DURATION = sum(p[0] for p in PHASES)
TOPICS = [
    (0, '/camera', 'sensor_msgs/msg/Image'),
    (1, '/camera_info', 'sensor_msgs/msg/CameraInfo'),
]


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
    info = make_camera_info()
    period = int(1e9 / FPS)
    t_ns = BASE_NS
    for duration, z in PHASES:
        frame = make_frame(with_marker=False) if z is None else make_frame(z=z)
        for _ in range(int(duration * FPS)):
            sec, nsec = divmod(t_ns, 1_000_000_000)
            image = make_image_msg(frame, 'mono8', (sec, nsec))
            info.header.stamp.sec, info.header.stamp.nanosec = sec, nsec
            writer.write('/camera', serialize_message(image), t_ns)
            writer.write('/camera_info', serialize_message(info), t_ns)
            t_ns += period
    del writer


@launch_pytest.fixture
def generate_test_description():
    bag_dir = os.path.join(tempfile.mkdtemp(), 'aruco_bag')
    build_bag(bag_dir)
    aruco = launch_ros.actions.Node(
        package='aruco_detector', executable='aruco_node', name='aruco_node',
        parameters=[{'marker_size_m': 0.3, 'marker_id': 42}], output='screen')
    play = launch.actions.ExecuteProcess(cmd=['ros2', 'bag', 'play', bag_dir], output='screen')
    return launch.LaunchDescription([
        aruco,
        launch.actions.TimerAction(period=BAG_DELAY, actions=[play]),
        launch_pytest.actions.ReadyToTest(),
    ])


@pytest.mark.launch(fixture=generate_test_description)
def test_aruco_pipeline_from_bag():
    rclpy.init()
    node = rclpy.create_node('aruco_rosbag_probe')
    detected, depths, frames = [], [], []
    node.create_subscription(Bool, '/hpad/detected', lambda m: detected.append(m.data), 100)
    node.create_subscription(
        PointStamped, '/hpad/position_camera', lambda m: depths.append(m.point.z), 100)
    node.create_subscription(Image, '/hpad/annotated', lambda m: frames.append(m), 100)
    end = time.time() + BAG_DELAY + BAG_DURATION + 4.0
    while time.time() < end:
        rclpy.spin_once(node, timeout_sec=0.05)
    node.destroy_node()
    rclpy.shutdown()

    assert len(detected) >= 40
    assert detected[0] is False and detected[-1] is False
    assert sum(detected) >= 32
    assert sum(detected) <= 42
    assert len(frames) >= 40

    near = [z for z in depths if z < 1.25]
    far = [z for z in depths if z >= 1.25]
    assert len(near) >= 12 and len(far) >= 12
    assert np.median(near) == pytest.approx(1.0, abs=0.1)
    assert np.median(far) == pytest.approx(1.5, abs=0.15)
    assert depths.index(near[0]) < depths.index(far[0])