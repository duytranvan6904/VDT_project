import math
import time

import numpy as np
import pytest
import rclpy
from geometry_msgs.msg import PointStamped, TransformStamped
from nav_msgs.msg import Odometry
from rclpy.duration import Duration
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from std_msgs.msg import String
from tf2_ros import Buffer, StaticTransformBroadcaster, TransformBroadcaster, TransformListener

from ekf_adapter.ekf_logic import NoiseConfig, TrackerConfig
from ekf_adapter.ekf_node import EkfNode, build_odometry_message, stamp_to_seconds
from ekf_adapter.odom_tf_node import OdomTfNode
from ekf_adapter.target_state_ekf import EstimatorSnapshot

CAMERA_FRAME = "camera_optical_frame"


@pytest.fixture(scope="module", autouse=True)
def ros_context():
    rclpy.init()
    yield
    rclpy.shutdown()


def make_transform(parent, child, stamp, translation=(0.0, 0.0, 0.0), rotation=(0.0, 0.0, 0.0, 1.0)):
    transform = TransformStamped()
    transform.header.stamp = stamp
    transform.header.frame_id = parent
    transform.child_frame_id = child
    transform.transform.translation.x = float(translation[0])
    transform.transform.translation.y = float(translation[1])
    transform.transform.translation.z = float(translation[2])
    transform.transform.rotation.x = float(rotation[0])
    transform.transform.rotation.y = float(rotation[1])
    transform.transform.rotation.z = float(rotation[2])
    transform.transform.rotation.w = float(rotation[3])
    return transform


class Harness:
    def __init__(self, with_ekf=True, with_odom_tf=False):
        self.executor = SingleThreadedExecutor()
        self.helper = Node("ekf_test_helper")
        self.executor.add_node(self.helper)
        self.ekf = EkfNode() if with_ekf else None
        self.odom_tf = OdomTfNode() if with_odom_tf else None
        for node in (self.ekf, self.odom_tf):
            if node is not None:
                self.executor.add_node(node)
        self.states = []
        self.modes = []
        self.helper.create_subscription(Odometry, "/hpad/state_filtered", self.states.append, 10)
        self.helper.create_subscription(String, "/ekf/tracking_mode", lambda msg: self.modes.append(msg.data), 10)
        self.point_pub = self.helper.create_publisher(PointStamped, "/hpad/position_camera", 10)
        self.phase_pub = self.helper.create_publisher(String, "/mission/phase", 10)
        best_effort = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=10)
        self.odom_pub = self.helper.create_publisher(Odometry, "/odom", best_effort)
        self.tf_pub = TransformBroadcaster(self.helper)
        self.static_pub = StaticTransformBroadcaster(self.helper)
        self.static_pub.sendTransform(make_transform("base_link", CAMERA_FRAME, Time().to_msg()))
        self.spin_for(0.5)

    def now_msg(self):
        return self.helper.get_clock().now().to_msg()

    def spin_until(self, predicate, timeout=3.0):
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            self.executor.spin_once(timeout_sec=0.05)
            if predicate():
                return True
        return False

    def spin_for(self, seconds):
        self.spin_until(lambda: False, seconds)

    def publish_point(self, xyz, frame_id=CAMERA_FRAME, stamp=None):
        msg = PointStamped()
        msg.header.frame_id = frame_id
        msg.header.stamp = stamp if stamp is not None else self.now_msg()
        msg.point.x, msg.point.y, msg.point.z = (float(value) for value in xyz)
        self.point_pub.publish(msg)

    def send_measurement(self, xyz, base_xyz=(0.0, 0.0, 0.0)):
        stamp = self.now_msg()
        self.tf_pub.sendTransform(make_transform("world", "base_link", stamp, base_xyz))
        self.spin_for(0.15)
        self.publish_point(xyz, stamp=stamp)

    def close(self):
        self.executor.shutdown()
        for node in (self.ekf, self.odom_tf, self.helper):
            if node is not None:
                node.destroy_node()


@pytest.fixture
def harness():
    instance = Harness()
    yield instance
    instance.close()


@pytest.fixture
def odom_harness():
    instance = Harness(with_ekf=False, with_odom_tf=True)
    yield instance
    instance.close()


def test_default_parameters_match_dataclass_defaults(harness):
    assert harness.ekf.tracker.config == TrackerConfig()
    assert harness.ekf.noise_config == NoiseConfig()
    assert harness.ekf.target_frame == "world"
    assert harness.ekf.child_frame == "hpad"
    assert harness.ekf.phase == "FOLLOW"


def test_no_state_and_expired_mode_before_first_measurement(harness):
    harness.spin_for(0.4)
    assert harness.states == []
    assert harness.modes
    assert set(harness.modes) == {"EXPIRED"}


def test_valid_measurement_publishes_state_in_world_frame(harness):
    harness.send_measurement((1.0, 2.0, 3.0))
    assert harness.spin_until(lambda: len(harness.states) > 0)
    message = harness.states[-1]
    assert message.header.frame_id == "world"
    assert message.child_frame_id == "hpad"
    assert stamp_to_seconds(message.header.stamp) > 0.0
    position = message.pose.pose.position
    np.testing.assert_allclose([position.x, position.y, position.z], [1.0, 2.0, 3.0], atol=0.05)
    assert harness.spin_until(lambda: harness.modes and harness.modes[-1] == "TRACKING")
    assert harness.ekf.outcome_counts["initialized"] == 1


def test_measurement_uses_tf_at_image_stamp(harness):
    harness.send_measurement((1.0, 2.0, 3.0), base_xyz=(10.0, 0.0, 0.0))
    assert harness.spin_until(lambda: len(harness.states) > 0)
    position = harness.states[-1].pose.pose.position
    np.testing.assert_allclose([position.x, position.y, position.z], [11.0, 2.0, 3.0], atol=0.05)


def test_output_message_covariance_and_orientation(harness):
    harness.send_measurement((1.0, 2.0, 3.0))
    assert harness.spin_until(lambda: len(harness.states) > 0)
    message = harness.states[-1]
    assert message.pose.pose.orientation.w == 1.0
    pose_cov = np.array(message.pose.covariance).reshape(6, 6)
    twist_cov = np.array(message.twist.covariance).reshape(6, 6)
    assert np.all(np.diag(pose_cov)[:3] > 0.0)
    assert np.all(np.diag(twist_cov)[:3] > 0.0)
    np.testing.assert_allclose(np.diag(pose_cov)[3:], 1e6)
    np.testing.assert_allclose(np.diag(twist_cov)[3:], 1e6)


def test_build_odometry_message_maps_state_and_covariance():
    covariance = np.diag([1, 2, 3, 4, 5, 6]).astype(float)
    snapshot = EstimatorSnapshot(1.0, np.array([1, 2, 3, 4, 5, 6], dtype=float), covariance, True)
    message = build_odometry_message(snapshot, Time().to_msg(), "world", "hpad")
    assert (message.pose.pose.position.x, message.pose.pose.position.y, message.pose.pose.position.z) == (1, 2, 3)
    assert (message.twist.twist.linear.x, message.twist.twist.linear.y, message.twist.twist.linear.z) == (4, 5, 6)
    pose_cov = np.array(message.pose.covariance).reshape(6, 6)
    twist_cov = np.array(message.twist.covariance).reshape(6, 6)
    np.testing.assert_allclose(np.diag(pose_cov)[:3], [1, 2, 3])
    np.testing.assert_allclose(np.diag(twist_cov)[:3], [4, 5, 6])


@pytest.mark.parametrize(
    "xyz",
    [(math.nan, 0.0, 1.0), (math.inf, 0.0, 1.0), (0.0, 0.0, 0.0), (0.0, 0.0, -1.0)],
)
def test_invalid_position_is_rejected(harness, xyz):
    harness.send_measurement(xyz)
    harness.spin_for(0.4)
    assert not harness.ekf.tracker.initialized
    assert harness.states == []


def test_measurement_without_stamp_is_rejected(harness):
    harness.publish_point((1.0, 0.0, 3.0), stamp=Time().to_msg())
    harness.spin_for(0.4)
    assert not harness.ekf.tracker.initialized


def test_measurement_without_frame_id_is_rejected(harness):
    harness.publish_point((1.0, 0.0, 3.0), frame_id="")
    harness.spin_for(0.4)
    assert not harness.ekf.tracker.initialized


def test_unknown_frame_increments_tf_reject_counter(harness):
    harness.tf_pub.sendTransform(make_transform("world", "base_link", harness.now_msg()))
    harness.spin_for(0.15)
    harness.publish_point((1.0, 0.0, 3.0), frame_id="missing_frame")
    assert harness.spin_until(lambda: harness.ekf.tf_reject_count >= 1)
    assert not harness.ekf.tracker.initialized
    assert harness.states == []


def test_phase_topic_updates_phase(harness):
    harness.phase_pub.publish(String(data="APPROACH"))
    assert harness.spin_until(lambda: harness.ekf.phase == "APPROACH")


def test_mode_expires_after_approach_limit(harness):
    harness.phase_pub.publish(String(data="APPROACH"))
    assert harness.spin_until(lambda: harness.ekf.phase == "APPROACH")
    harness.send_measurement((1.0, 2.0, 3.0))
    assert harness.spin_until(lambda: harness.modes and harness.modes[-1] == "TRACKING")
    harness.spin_for(1.4)
    assert harness.modes[-1] == "EXPIRED"
    assert len(harness.states) > 0


def test_state_keeps_publishing_while_track_is_coasting(harness):
    harness.send_measurement((1.0, 2.0, 3.0))
    assert harness.spin_until(lambda: len(harness.states) > 0)
    count = len(harness.states)
    harness.spin_for(0.5)
    assert len(harness.states) > count


def test_odom_tf_node_broadcasts_normalized_transform(odom_harness):
    buffer = Buffer()
    listener = TransformListener(buffer, odom_harness.helper)
    msg = Odometry()
    msg.header.stamp = odom_harness.now_msg()
    msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z = 1.0, 2.0, 3.0
    msg.pose.pose.orientation.w = 2.0

    def publish_and_check():
        odom_harness.odom_pub.publish(msg)
        return buffer.can_transform("world", "base_link", Time(), Duration(seconds=0.0))

    assert odom_harness.spin_until(publish_and_check, 3.0)
    transform = buffer.lookup_transform("world", "base_link", Time()).transform
    np.testing.assert_allclose(
        [transform.translation.x, transform.translation.y, transform.translation.z], [1.0, 2.0, 3.0]
    )
    assert transform.rotation.w == pytest.approx(1.0)
    del listener


def test_odom_tf_node_uses_clock_when_stamp_is_zero(odom_harness):
    buffer = Buffer()
    listener = TransformListener(buffer, odom_harness.helper)
    msg = Odometry()
    msg.pose.pose.orientation.w = 1.0

    def publish_and_check():
        odom_harness.odom_pub.publish(msg)
        return buffer.can_transform("world", "base_link", Time(), Duration(seconds=0.0))

    assert odom_harness.spin_until(publish_and_check, 3.0)
    stamp = buffer.lookup_transform("world", "base_link", Time()).header.stamp
    assert stamp_to_seconds(stamp) > 0.0
    del listener


def test_odom_tf_node_ignores_zero_norm_quaternion(odom_harness):
    buffer = Buffer()
    listener = TransformListener(buffer, odom_harness.helper)
    msg = Odometry()
    msg.header.stamp = odom_harness.now_msg()
    for _ in range(10):
        odom_harness.odom_pub.publish(msg)
        odom_harness.spin_for(0.05)
    assert not buffer.can_transform("world", "base_link", Time(), Duration(seconds=0.0))
    del listener