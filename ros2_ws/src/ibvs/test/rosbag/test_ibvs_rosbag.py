import math
import os
import shutil
import signal
import subprocess
import time

import numpy as np
import pytest

rosbag2_py = pytest.importorskip("rosbag2_py")
import rclpy
from builtin_interfaces.msg import Time as TimeMsg
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.serialization import serialize_message
from sensor_msgs.msg import CameraInfo
from std_msgs.msg import Bool, Float32, Float64, String
from vision_msgs.msg import BoundingBox2D

pytestmark = pytest.mark.skipif(shutil.which("ros2") is None, reason="ros2 CLI unavailable")

T0 = 100.0
DURATION_S = 6.0
MARKER_VISIBLE_UNTIL_S = 3.0
BBOX_HZ = 15.0
ODOM_HZ = 50.0
SLOW_HZ = 10.0
FX = 500.0
FY = 500.0
CX = 320.0
CY = 240.0
U_OFFSET_PX = 150.0
V_OFFSET_PX = 80.0
GIMBAL_DEG = -30.0
YAW_DEADBAND_PX = 12.0
K_YAW = 0.5
PITCH_LIMIT_DEG = 20.0
PLAY_TIMEOUT_S = 40.0

TOPICS = (
    ("/odom", "nav_msgs/msg/Odometry"),
    ("/camera_info", "sensor_msgs/msg/CameraInfo"),
    ("/mission/phase", "std_msgs/msg/String"),
    ("/gimbal/target_angle_deg", "std_msgs/msg/Float32"),
    ("/hpad/detected", "std_msgs/msg/Bool"),
    ("/hpad/bbox", "vision_msgs/msg/BoundingBox2D"),
)


def to_stamp(seconds):
    msg = TimeMsg()
    msg.sec = int(seconds)
    msg.nanosec = int(round((seconds - int(seconds)) * 1e9))
    return msg


def expected_yaw_cmd():
    effective = U_OFFSET_PX - YAW_DEADBAND_PX
    return -K_YAW * (effective / FX) * math.cos(math.radians(GIMBAL_DEG))


def build_events():
    events = []
    for t in np.arange(0.0, DURATION_S, 1.0 / ODOM_HZ):
        odom = Odometry()
        odom.header.stamp = to_stamp(T0 + t)
        odom.header.frame_id = "world"
        odom.pose.pose.orientation.w = 1.0
        events.append((t, "/odom", odom))
    for t in np.arange(0.0, DURATION_S, 1.0 / SLOW_HZ):
        info = CameraInfo()
        info.header.stamp = to_stamp(T0 + t)
        info.k = [FX, 0.0, CX, 0.0, FY, CY, 0.0, 0.0, 1.0]
        events.append((t, "/camera_info", info))
        events.append((t, "/mission/phase", String(data="FOLLOW")))
        events.append((t, "/gimbal/target_angle_deg", Float32(data=GIMBAL_DEG)))
    for t in np.arange(0.0, DURATION_S, 1.0 / BBOX_HZ):
        visible = t < MARKER_VISIBLE_UNTIL_S
        events.append((t, "/hpad/detected", Bool(data=visible)))
        if visible:
            bbox = BoundingBox2D()
            bbox.center.position.x = CX + U_OFFSET_PX
            bbox.center.position.y = CY + V_OFFSET_PX
            events.append((t, "/hpad/bbox", bbox))
    return sorted(events, key=lambda event: event[0])


def write_bag(path):
    writer = rosbag2_py.SequentialWriter()
    writer.open(
        rosbag2_py.StorageOptions(uri=str(path), storage_id="sqlite3"),
        rosbag2_py.ConverterOptions("", ""),
    )
    for name, type_name in TOPICS:
        writer.create_topic(
            rosbag2_py.TopicMetadata(
                name=name, type=type_name, serialization_format="cdr", offered_qos_profiles=""
            )
        )
    for t, topic, msg in build_events():
        writer.write(topic, serialize_message(msg), int((T0 + t) * 1e9))
    del writer


class Collector(Node):
    def __init__(self):
        super().__init__("ibvs_rosbag_collector")
        self.yaws = []
        self.trims = []
        self.create_subscription(Float64, "/ibvs/yaw_cmd", self.on_yaw, 100)
        self.create_subscription(Float32, "/ibvs/pitch_trim_deg", self.on_trim, 100)

    def on_yaw(self, msg):
        self.yaws.append((time.monotonic(), msg.data))

    def on_trim(self, msg):
        self.trims.append((time.monotonic(), msg.data))

    def window(self, series, start, end):
        return [value for t, value in series if start <= t <= end]


def spawn(command, processes):
    process = subprocess.Popen(
        command, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, start_new_session=True
    )
    processes.append(process)
    return process


def stop(processes):
    for process in processes:
        if process.poll() is None:
            os.killpg(process.pid, signal.SIGTERM)
    for process in processes:
        try:
            process.wait(timeout=5)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGKILL)


def spin_for(node, seconds):
    end = time.monotonic() + seconds
    while time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=0.02)


@pytest.fixture(scope="module")
def replay(tmp_path_factory):
    bag = tmp_path_factory.mktemp("bag") / "ibvs_flight"
    write_bag(bag)
    processes = []
    rclpy.init()
    collector = Collector()
    try:
        spawn(["ros2", "run", "ibvs", "ibvs_controller"], processes)
        spin_for(collector, 3.0)
        player = spawn(["ros2", "bag", "play", str(bag)], processes)
        deadline = time.monotonic() + PLAY_TIMEOUT_S
        while player.poll() is None and time.monotonic() < deadline:
            rclpy.spin_once(collector, timeout_sec=0.02)
        collector.play_end = time.monotonic()
        spin_for(collector, 0.5)
        yield collector
    finally:
        stop(processes)
        collector.destroy_node()
        rclpy.shutdown()


def test_commands_are_published_and_finite(replay):
    assert len(replay.yaws) > 50
    assert len(replay.trims) > 50
    values = [v for _, v in replay.yaws] + [v for _, v in replay.trims]
    assert all(math.isfinite(value) for value in values)
    assert all(-math.pi <= v <= math.pi for _, v in replay.yaws)


def test_yaw_converges_to_pixel_target(replay):
    final = replay.window(replay.yaws, replay.play_end - 1.0, replay.play_end + 0.5)
    assert final
    assert final[-1] == pytest.approx(expected_yaw_cmd(), abs=0.01)


def test_yaw_turns_toward_marker_side(replay):
    assert min(v for _, v in replay.yaws) < -0.05


def test_pitch_trim_saturates_without_exceeding_limit(replay):
    values = [v for _, v in replay.trims]
    assert min(values) >= -PITCH_LIMIT_DEG - 0.01
    assert values[-1] <= -15.0


def test_commands_hold_when_marker_is_lost_without_ekf(replay):
    window_start = replay.play_end - 2.0
    yaws = replay.window(replay.yaws, window_start, replay.play_end)
    trims = replay.window(replay.trims, window_start, replay.play_end)
    assert yaws and trims
    assert max(yaws) - min(yaws) < 1e-3
    assert max(trims) - min(trims) < 1e-3