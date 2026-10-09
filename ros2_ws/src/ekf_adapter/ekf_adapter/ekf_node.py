from __future__ import annotations

from collections import Counter
from dataclasses import fields

import numpy as np
import rclpy
import tf2_ros
from geometry_msgs.msg import PointStamped
from nav_msgs.msg import Odometry
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from std_msgs.msg import String
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

from .ekf_logic import (
    MeasurementOutcome,
    NoiseConfig,
    TargetTracker,
    TrackerConfig,
    build_world_covariance,
    quaternion_to_matrix,
    transform_point,
)
from .target_state_ekf import EstimatorSnapshot, TargetStateEKF
from .tracking_policy import classify_tracking_mode

STATISTICS_PERIOD_S = 2.0
ORIENTATION_UNKNOWN_VARIANCE = 1e6


def declare_dataclass_parameters(node: Node, config_class) -> None:
    for item in fields(config_class):
        node.declare_parameter(item.name, item.default)


def read_dataclass_parameters(node: Node, config_class):
    values = {
        item.name: type(item.default)(node.get_parameter(item.name).value)
        for item in fields(config_class)
    }
    return config_class(**values)


def stamp_to_seconds(stamp) -> float:
    return stamp.sec + stamp.nanosec * 1e-9


def build_odometry_message(snapshot: EstimatorSnapshot, stamp, frame_id: str, child_frame_id: str) -> Odometry:
    state = snapshot.state
    covariance = snapshot.covariance
    message = Odometry()
    message.header.stamp = stamp
    message.header.frame_id = frame_id
    message.child_frame_id = child_frame_id
    message.pose.pose.position.x = float(state[0])
    message.pose.pose.position.y = float(state[1])
    message.pose.pose.position.z = float(state[2])
    message.pose.pose.orientation.w = 1.0
    message.twist.twist.linear.x = float(state[3])
    message.twist.twist.linear.y = float(state[4])
    message.twist.twist.linear.z = float(state[5])
    pose_covariance = np.eye(6) * ORIENTATION_UNKNOWN_VARIANCE
    pose_covariance[:3, :3] = covariance[:3, :3]
    twist_covariance = np.eye(6) * ORIENTATION_UNKNOWN_VARIANCE
    twist_covariance[:3, :3] = covariance[3:, 3:]
    message.pose.covariance = pose_covariance.flatten().tolist()
    message.twist.covariance = twist_covariance.flatten().tolist()
    return message


class EkfNode(Node):
    def __init__(self) -> None:
        super().__init__("ekf_node")
        self.declare_node_parameters()
        self.target_frame = str(self.get_parameter("target_frame").value)
        self.child_frame = str(self.get_parameter("child_frame").value)
        self.tf_timeout_s = float(self.get_parameter("tf_timeout_s").value)
        self.noise_config = read_dataclass_parameters(self, NoiseConfig)
        self.tracker = TargetTracker(self.create_estimator(), read_dataclass_parameters(self, TrackerConfig))
        self.phase = "FOLLOW"
        self.outcome_counts = Counter()
        self.tf_reject_count = 0
        self.last_statistics_time = 0.0
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self, spin_thread=True)
        self.create_interfaces()

    def declare_node_parameters(self) -> None:
        self.declare_parameter("process_accel_variance", [1.0, 1.0, 0.5])
        self.declare_parameter("gate_threshold", 16.27)
        self.declare_parameter("max_target_speed", 2.5)
        self.declare_parameter("max_target_vz", 1.5)
        self.declare_parameter("target_frame", "world")
        self.declare_parameter("child_frame", "hpad")
        self.declare_parameter("tf_timeout_s", 0.03)
        self.declare_parameter("output_rate_hz", 50.0)
        self.declare_parameter("position_topic", "/hpad/position_camera")
        self.declare_parameter("phase_topic", "/mission/phase")
        self.declare_parameter("state_topic", "/hpad/state_filtered")
        self.declare_parameter("mode_topic", "/ekf/tracking_mode")
        declare_dataclass_parameters(self, TrackerConfig)
        declare_dataclass_parameters(self, NoiseConfig)

    def create_estimator(self) -> TargetStateEKF:
        return TargetStateEKF(
            process_accel_variance=tuple(self.get_parameter("process_accel_variance").value),
            gate_threshold=float(self.get_parameter("gate_threshold").value),
            v_max=float(self.get_parameter("max_target_speed").value),
            vz_max=float(self.get_parameter("max_target_vz").value),
        )

    def create_interfaces(self) -> None:
        self.create_subscription(
            PointStamped, str(self.get_parameter("position_topic").value), self.position_callback, 10
        )
        self.create_subscription(String, str(self.get_parameter("phase_topic").value), self.phase_callback, 10)
        self.state_publisher = self.create_publisher(Odometry, str(self.get_parameter("state_topic").value), 10)
        self.mode_publisher = self.create_publisher(String, str(self.get_parameter("mode_topic").value), 10)
        period = 1.0 / float(self.get_parameter("output_rate_hz").value)
        self.create_timer(period, self.timer_callback)

    def phase_callback(self, msg: String) -> None:
        self.phase = msg.data

    def lookup_transform(self, header):
        try:
            return self.tf_buffer.lookup_transform(
                self.target_frame,
                header.frame_id,
                Time.from_msg(header.stamp),
                timeout=Duration(seconds=self.tf_timeout_s),
            ).transform
        except (
            tf2_ros.LookupException,
            tf2_ros.ConnectivityException,
            tf2_ros.ExtrapolationException,
        ) as error:
            self.tf_reject_count += 1
            self.get_logger().warning(
                f"tf rejected #{self.tf_reject_count} {header.frame_id} -> {self.target_frame}: {error}",
                throttle_duration_sec=2.0,
            )
            return None

    def position_callback(self, msg: PointStamped) -> None:
        stamp = stamp_to_seconds(msg.header.stamp)
        if stamp <= 0.0 or not msg.header.frame_id:
            self.get_logger().warning("measurement without stamp or frame_id rejected", throttle_duration_sec=2.0)
            return
        camera_position = np.array([msg.point.x, msg.point.y, msg.point.z], dtype=float)
        if not np.all(np.isfinite(camera_position)) or camera_position[2] <= 0.0:
            return
        transform = self.lookup_transform(msg.header)
        if transform is None:
            return
        rotation = quaternion_to_matrix(
            transform.rotation.x, transform.rotation.y, transform.rotation.z, transform.rotation.w
        )
        translation = (transform.translation.x, transform.translation.y, transform.translation.z)
        world_position = transform_point(camera_position, rotation, translation)
        covariance = build_world_covariance(camera_position, rotation, self.noise_config)
        outcome = self.tracker.process_measurement(world_position, covariance, stamp)
        self.outcome_counts[outcome.value] += 1
        self.log_outcome(outcome, world_position)

    def log_outcome(self, outcome: MeasurementOutcome, world_position: np.ndarray) -> None:
        text = f"({world_position[0]:.2f}, {world_position[1]:.2f}, {world_position[2]:.2f})"
        if outcome == MeasurementOutcome.INITIALIZED:
            self.get_logger().info(f"track initialized at {text}")
        elif outcome == MeasurementOutcome.REACQUIRED:
            self.get_logger().info(f"track reacquired at {text}")
        elif outcome == MeasurementOutcome.REJECTED_GATE:
            self.get_logger().warning(f"measurement outside gate at {text}", throttle_duration_sec=0.5)
        elif outcome == MeasurementOutcome.REJECTED_ORDER:
            self.get_logger().warning("out-of-order measurement rejected", throttle_duration_sec=1.0)

    def timer_callback(self) -> None:
        now = self.get_clock().now()
        now_s = now.nanoseconds * 1e-9
        age = self.tracker.age(now_s)
        mode = classify_tracking_mode(self.tracker.detected(now_s), age, self.phase)
        self.mode_publisher.publish(String(data=mode.value))
        snapshot = self.tracker.snapshot_at(now_s)
        if snapshot is None:
            return
        self.state_publisher.publish(
            build_odometry_message(snapshot, now.to_msg(), self.target_frame, self.child_frame)
        )
        self.log_statistics(now_s, snapshot, mode.value, age)

    def log_statistics(self, now_s: float, snapshot: EstimatorSnapshot, mode: str, age: float) -> None:
        if now_s - self.last_statistics_time < STATISTICS_PERIOD_S:
            return
        self.last_statistics_time = now_s
        state = snapshot.state
        counts = dict(self.outcome_counts)
        self.get_logger().info(
            f"mode={mode} age={age:.2f}s pos=({state[0]:.2f}, {state[1]:.2f}, {state[2]:.2f}) "
            f"vel=({state[3]:.2f}, {state[4]:.2f}, {state[5]:.2f}) tf_rejects={self.tf_reject_count} {counts}"
        )


def main() -> None:
    rclpy.init()
    node = EkfNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
