#!/usr/bin/env python3
"""EKF ROS Adapter — bridge ArUco camera-frame detections to world-frame state.

This node is the glue between the ArUco detection node and the core EKF
estimator.  It performs coordinate transforms via TF2 and publishes the
smoothed target state in the world frame.

Pipeline:
  /hpad/position_camera (PointStamped, camera_optical_frame)
  + /hpad/detected (Bool)
  + /odom (Odometry)
  → TF2 transform camera_optical_frame → world
  → TargetStateEKF.predict() / .update()
  → /ekf/target_state (Odometry, world frame)
  → /ekf/tracking_mode (String)
"""

from __future__ import annotations

import math
import os
import sys
import time

import numpy as np

# Ensure project root is on sys.path for vision package imports.
REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if REPO_ROOT not in sys.path:
    sys.path.insert(0, REPO_ROOT)

from vision.target_state_ekf import TargetStateEKF
from vision.target_tracking_policy import classify_tracking_mode, TrackingMode

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from rclpy.time import Time

import tf2_ros
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

from geometry_msgs.msg import PointStamped, TransformStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, String


def _transform_point_manual(
    point_camera: np.ndarray,
    tf_msg: TransformStamped,
) -> np.ndarray:
    """Transform a 3D point using a TransformStamped message.

    Avoids dependency on tf2_geometry_msgs (which requires C++ bindings
    that may not be importable in all environments).  Uses the rotation
    quaternion (x, y, z, w) and translation from the transform.
    """
    t = tf_msg.transform.translation
    q = tf_msg.transform.rotation
    # Quaternion rotation: v' = q * v * q_conj
    # Expand for efficiency:
    qx, qy, qz, qw = q.x, q.y, q.z, q.w
    px, py, pz = point_camera

    # Rotation matrix from quaternion
    r00 = 1.0 - 2.0 * (qy * qy + qz * qz)
    r01 = 2.0 * (qx * qy - qz * qw)
    r02 = 2.0 * (qx * qz + qy * qw)
    r10 = 2.0 * (qx * qy + qz * qw)
    r11 = 1.0 - 2.0 * (qx * qx + qz * qz)
    r12 = 2.0 * (qy * qz - qx * qw)
    r20 = 2.0 * (qx * qz - qy * qw)
    r21 = 2.0 * (qy * qz + qx * qw)
    r22 = 1.0 - 2.0 * (qx * qx + qy * qy)

    rx = r00 * px + r01 * py + r02 * pz + t.x
    ry = r10 * px + r11 * py + r12 * pz + t.y
    rz = r20 * px + r21 * py + r22 * pz + t.z

    return np.array([rx, ry, rz])


class EKFRosAdapter(Node):
    """Bridge ArUco → EKF → world-frame target state estimation."""

    def __init__(self):
        super().__init__('ekf_ros_adapter')

        # ── Parameters ───────────────────────────────────────────────────
        self.declare_parameter('process_accel_variance', [1.0, 1.0, 0.5])
        self.declare_parameter('gate_threshold', 16.27)
        self.declare_parameter('predict_rate_hz', 50.0)
        self.declare_parameter('source_frame', 'camera_optical_frame')
        self.declare_parameter('target_frame', 'world')

        accel_var = self.get_parameter('process_accel_variance').value
        gate = self.get_parameter('gate_threshold').value
        predict_hz = self.get_parameter('predict_rate_hz').value
        self.source_frame = self.get_parameter('source_frame').value
        self.target_frame = self.get_parameter('target_frame').value

        # ── Core EKF ─────────────────────────────────────────────────────
        self.ekf = TargetStateEKF(
            process_accel_variance=tuple(accel_var),
            gate_threshold=gate,
        )

        # ── TF2 ──────────────────────────────────────────────────────────
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # ── State ────────────────────────────────────────────────────────
        self.last_detection_time = 0.0       # monotonic timestamp of last True detection
        self.last_ekf_time = None            # last timestamp fed to EKF (for dt calc)
        self.detected = False
        self.detection_count = 0             # consecutive detection counter
        self.phase = 'FOLLOW'                # current mission phase (for tracking policy)

        # ── Measurement covariance ───────────────────────────────────────
        # Position measurement noise (PnP + TF uncertainty)
        self.R = np.diag([0.10, 0.10, 0.08]) ** 2

        # ── Subscribers ──────────────────────────────────────────────────
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST, depth=5,
        )
        self.create_subscription(
            PointStamped, '/hpad/position_camera',
            self.position_cb, 10,
        )
        self.create_subscription(
            Bool, '/hpad/detected',
            self.detected_cb, 10,
        )
        self.create_subscription(
            String, '/mission/phase',
            self.phase_cb, 10,
        )

        # ── Publishers ───────────────────────────────────────────────────
        self.state_pub = self.create_publisher(
            Odometry, '/ekf/target_state', 10,
        )
        self.mode_pub = self.create_publisher(
            String, '/ekf/tracking_mode', 10,
        )

        # ── Predict timer (event-driven predict at fixed rate) ───────────
        predict_dt = 1.0 / predict_hz
        self.create_timer(predict_dt, self.predict_timer_cb)

        self.get_logger().info(
            f'EKF ROS Adapter ready: {self.source_frame} → {self.target_frame}, '
            f'predict at {predict_hz:.0f} Hz'
        )

    # ── Callbacks ────────────────────────────────────────────────────────

    def detected_cb(self, msg: Bool):
        self.detected = msg.data
        if msg.data:
            self.last_detection_time = time.monotonic()
            self.detection_count += 1
        else:
            self.detection_count = 0

    def phase_cb(self, msg: String):
        self.phase = msg.data

    def position_cb(self, msg: PointStamped):
        """Called when ArUco detection publishes a 3D position in camera frame.

        This is the EVENT-DRIVEN update path: transform to world, feed EKF.
        """
        if not self.detected:
            return

        # Transform camera → world via TF2
        camera_pos = np.array([msg.point.x, msg.point.y, msg.point.z])
        world_pos = self._transform_to_world(camera_pos, msg.header)
        if world_pos is None:
            return

        # Compute dt
        now = time.monotonic()
        if self.last_ekf_time is not None:
            dt = now - self.last_ekf_time
            dt = max(0.001, min(dt, 1.0))  # clamp to [1ms, 1s]
        else:
            dt = 1.0 / 30.0  # initial guess
        self.last_ekf_time = now

        # EKF predict + update
        self.ekf.predict(dt)
        self.ekf.update(world_pos, self.R)

        # Publish updated state
        self._publish_state()

    def predict_timer_cb(self):
        """Fixed-rate EKF predict for smooth state estimation when no measurements."""
        if self.last_ekf_time is None:
            return  # not initialized yet

        snap = self.ekf.snapshot(time.monotonic())
        if not snap.initialized:
            return

        now = time.monotonic()
        dt = now - self.last_ekf_time
        dt = max(0.001, min(dt, 1.0))

        # Only predict (no update) — the measurement path handles updates
        if not self.detected:
            self.ekf.predict(dt)
            self.last_ekf_time = now
            self._publish_state()

    # ── Helpers ──────────────────────────────────────────────────────────

    def _transform_to_world(
        self, camera_pos: np.ndarray, header,
    ) -> np.ndarray | None:
        """Transform a point from camera_optical_frame to world using TF2."""
        try:
            tf_msg = self.tf_buffer.lookup_transform(
                self.target_frame,
                self.source_frame,
                rclpy.time.Time(),  # latest available
                timeout=rclpy.duration.Duration(seconds=0.1),
            )
            return _transform_point_manual(camera_pos, tf_msg)
        except (
            tf2_ros.LookupException,
            tf2_ros.ConnectivityException,
            tf2_ros.ExtrapolationException,
        ) as exc:
            self.get_logger().warning(
                f'TF2 lookup failed ({self.source_frame} → {self.target_frame}): {exc}',
                throttle_duration_sec=2.0,
            )
            return None

    def _publish_state(self):
        """Publish current EKF state as Odometry + tracking mode as String."""
        now_mono = time.monotonic()
        snap = self.ekf.snapshot(now_mono)

        if not snap.initialized:
            return

        # --- Tracking mode ---
        age = now_mono - self.last_detection_time if self.last_detection_time > 0 else 999.0
        mode = classify_tracking_mode(
            detected=self.detected,
            age_since_measurement_s=age,
            phase=self.phase if self.phase in ('APPROACH', 'FOLLOW') else 'FOLLOW',
        )
        self.mode_pub.publish(String(data=mode.value))

        # --- Target state as Odometry ---
        state = snap.state  # [x, y, z, vx, vy, vz]
        cov = snap.covariance  # 6x6

        msg = Odometry()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.target_frame
        msg.child_frame_id = 'target'

        # Position
        msg.pose.pose.position.x = float(state[0])
        msg.pose.pose.position.y = float(state[1])
        msg.pose.pose.position.z = float(state[2])
        msg.pose.pose.orientation.w = 1.0  # no orientation estimated

        # Pack position covariance (6x6 → first 3 rows/cols into pose.covariance)
        pose_cov = np.zeros(36)
        for i in range(3):
            for j in range(3):
                pose_cov[i * 6 + j] = cov[i, j]
        msg.pose.covariance = pose_cov.tolist()

        # Velocity
        msg.twist.twist.linear.x = float(state[3])
        msg.twist.twist.linear.y = float(state[4])
        msg.twist.twist.linear.z = float(state[5])

        # Pack velocity covariance
        twist_cov = np.zeros(36)
        for i in range(3):
            for j in range(3):
                twist_cov[i * 6 + j] = cov[i + 3, j + 3]
        msg.twist.covariance = twist_cov.tolist()

        self.state_pub.publish(msg)


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main():
    rclpy.init()
    node = EKFRosAdapter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print('\n[EKF Adapter] Shutting down...')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
