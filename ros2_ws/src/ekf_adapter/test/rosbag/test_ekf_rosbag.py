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
from geometry_msgs.msg import PointStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.serialization import serialize_message
from std_msgs.msg import String

pytestmark = pytest.mark.skipif(shutil.which("ros2") is None, reason="ros2 CLI unavailable")

T0 = 100.0
DURATION_S = 8.0
MEASUREMENT_HZ = 15.0
ODOM_HZ = 50.0
DROPOUT_S = (3.0, 6.0)
SPEED_MPS = 0.5
START_XYZ = (1.0, 0.5, 3.0)
MEASUREMENT_NOISE_M = 0.02
PLAY_TIMEOUT_S = 40.0


def to_stamp(seconds):
    msg = TimeMsg()
    msg.sec = int(seconds)
    msg.nanosec = int(round((seconds - int(seconds)) * 1e9))
    return msg


def truth(t):
    return np.array([START_XYZ[0] + SPEED_MPS * t, START_XYZ[1], START_XYZ[2]])


def build_events():
    rng = np.random.default_rng(3)
    events = []
    for t in np.arange(0.0, DURATION_S, 1.0 / ODOM_HZ):
        odom = Odometry()
        odom.header.stamp = to_stamp(T0 + t)
        odom.header.frame_id = "world"
        odom.child_frame_id = "base_link"
        odom.pose.pose.orientation.w = 1.0
        events.append((t, "/odom", odom))
    for t in np.arange(0.0, DURATION_S, 1.0 / MEASUREMENT_HZ):
        if DROPOUT_S[0] <= t < DROPOUT_S[1]:
            continue
        point = PointStamped()
        point.header.stamp = to_stamp(T0 + t)
        point.header.frame_id = "camera_optical_frame"
        x, y, z = truth(t) + rng.normal(0.0, MEASUREMENT_NOISE_M, 3)
        point.point.x, point.point.y, point.point.z = float(x), float(y), float(z)
        events.append((t, "/hpad/position_camera", point))
    return sorted(events, key=lambda event: event[0])


def write_bag(path):
    writer = rosbag2_py.SequentialWriter()
    writer.open(
        rosbag2_py.StorageOptions(uri=str(path), storage_id="sqlite3"),
        rosbag2_py.ConverterOptions("", ""),
    )
    for name, type_name in (
        ("/odom", "nav_msgs/msg/Odometry"),
        ("/hpad/position_camera", "geometry_msgs/msg/PointStamped"),
    ):
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
        super().__init__("ekf_rosbag_collector", parameter_overrides=[Parameter("use_sim_time", Parameter.Type.BOOL, True)])
        self.states = []
        self.modes = []
        self.create_subscription(Odometry, "/hpad/state_filtered", self.on_state, 50)
        self.create_subscription(String, "/ekf/tracking_mode", self.on_mode, 50)

    def on_state(self, msg):
        self.states.append((msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9 - T0, msg))

    def on_mode(self, msg):
        self.modes.append((self.get_clock().now().nanoseconds * 1e-9 - T0, msg.data))

    def modes_between(self, start, end):
        return [mode for t, mode in self.modes if start <= t < end]


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


@pytest.fixture(scope="module")
def replay(tmp_path_factory):
    bag = tmp_path_factory.mktemp("bag") / "flight"
    write_bag(bag)
    processes = []
    rclpy.init()
    collector = Collector()
    try:
        spawn(
            ["ros2", "run", "tf2_ros", "static_transform_publisher",
             "--frame-id", "base_link", "--child-frame-id", "camera_optical_frame"],
            processes,
        )
        sim_time = ["--ros-args", "-p", "use_sim_time:=true"]
        spawn(["ros2", "run", "ekf_adapter", "odom_tf_node", *sim_time], processes)
        spawn(["ros2", "run", "ekf_adapter", "ekf_node", *sim_time], processes)
        warmup = time.monotonic() + 3.0
        while time.monotonic() < warmup:
            rclpy.spin_once(collector, timeout_sec=0.05)
        player = spawn(["ros2", "bag", "play", str(bag), "--clock"], processes)
        deadline = time.monotonic() + PLAY_TIMEOUT_S
        while player.poll() is None and time.monotonic() < deadline:
            rclpy.spin_once(collector, timeout_sec=0.02)
        settle = time.monotonic() + 1.0
        while time.monotonic() < settle:
            rclpy.spin_once(collector, timeout_sec=0.02)
        yield collector
    finally:
        stop(processes)
        collector.destroy_node()
        rclpy.shutdown()


def test_state_stream_is_published(replay):
    assert len(replay.states) > 100
    assert len(replay.modes) > 100


def test_state_frames(replay):
    _, message = replay.states[-1]
    assert message.header.frame_id == "world"
    assert message.child_frame_id == "hpad"


def test_position_error_before_dropout(replay):
    errors = [
        np.linalg.norm(np.array([m.pose.pose.position.x, m.pose.pose.position.y, m.pose.pose.position.z]) - truth(t))
        for t, m in replay.states
        if 2.0 <= t < DROPOUT_S[0]
    ]
    assert errors
    assert max(errors) < 0.15


def test_velocity_estimate_before_dropout(replay):
    velocities = [m.twist.twist.linear.x for t, m in replay.states if 2.0 <= t < DROPOUT_S[0]]
    assert velocities
    assert np.mean(velocities) == pytest.approx(SPEED_MPS, abs=0.2)


def test_position_error_after_dropout(replay):
    errors = [
        np.linalg.norm(np.array([m.pose.pose.position.x, m.pose.pose.position.y, m.pose.pose.position.z]) - truth(t))
        for t, m in replay.states
        if t >= DROPOUT_S[1] + 1.0
    ]
    assert errors
    assert max(errors) < 0.15


def test_mode_sequence_through_dropout(replay):
    assert "TRACKING" in replay.modes_between(1.0, DROPOUT_S[0])
    assert "PREDICTING_DEGRADED" in replay.modes_between(DROPOUT_S[0] + 1.1, DROPOUT_S[0] + 1.8)
    assert "EXPIRED" in replay.modes_between(DROPOUT_S[0] + 2.2, DROPOUT_S[1])
    after = replay.modes_between(DROPOUT_S[1] + 0.5, DURATION_S)
    assert after
    assert after[-1] == "TRACKING"


def test_mode_values_are_valid(replay):
    valid = {"TRACKING", "PREDICTING", "PREDICTING_DEGRADED", "EXPIRED"}
    assert {mode for _, mode in replay.modes} <= valid