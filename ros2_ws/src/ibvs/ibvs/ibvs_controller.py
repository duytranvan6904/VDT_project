#!/usr/bin/env python3
from __future__ import annotations

import math
import time

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo
from std_msgs.msg import Bool, Float32, Float64, String
from vision_msgs.msg import BoundingBox2D

from ibvs.ibvs_logic import (
    YAW_DEADBAND_PX,
    clamp,
    deadband,
    pixel_pitch_correction,
    quaternion_to_yaw,
    slew_angle,
    tangential_yaw_feedforward,
    wrap_angle,
)


class IBVSController(Node):
    def __init__(self):
        super().__init__('ibvs_controller')

        self.declare_parameter('K_pitch', 0.8)
        self.declare_parameter('K_yaw', 0.5)
        self.declare_parameter('focal_x', 466.0)
        self.declare_parameter('focal_y', 466.0)
        self.declare_parameter('u0', 320.0)
        self.declare_parameter('v0', 240.0)
        self.declare_parameter('pitch_rate_limit', 1.5)
        self.declare_parameter('pitch_ema_alpha', 0.25)
        self.declare_parameter('pitch_pixel_trim_gain', 0.5)
        self.declare_parameter('pitch_trim_limit_deg', 20.0)
        self.declare_parameter('yaw_rate_limit', 0.50)
        self.declare_parameter('nadir_yaw_cutoff_deg', -60.0)
        self.declare_parameter('nadir_distance_cutoff_m', 1.0)
        self.declare_parameter('target_state_timeout', 0.50)

        self.K_pitch = float(self.get_parameter('K_pitch').value)
        self.K_yaw = float(self.get_parameter('K_yaw').value)
        self.fx = float(self.get_parameter('focal_x').value)
        self.fy = float(self.get_parameter('focal_y').value)
        self.u0 = float(self.get_parameter('u0').value)
        self.v0 = float(self.get_parameter('v0').value)
        self.pitch_rate_limit = float(self.get_parameter('pitch_rate_limit').value)
        self.ema_alpha = float(self.get_parameter('pitch_ema_alpha').value)
        self.pitch_pixel_trim_gain = float(self.get_parameter('pitch_pixel_trim_gain').value)
        self.pitch_trim_limit = math.radians(float(self.get_parameter('pitch_trim_limit_deg').value))
        self.yaw_rate_limit = float(self.get_parameter('yaw_rate_limit').value)
        self.nadir_yaw_cutoff_rad = math.radians(float(self.get_parameter('nadir_yaw_cutoff_deg').value))
        self.nadir_distance_cutoff_m = float(self.get_parameter('nadir_distance_cutoff_m').value)
        self.target_state_timeout = float(self.get_parameter('target_state_timeout').value)

        self.phase = 'IDLE'
        self.detected = False
        self.tracking_mode = 'EXPIRED'
        self.drone_yaw = 0.0
        self.drone_pitch = 0.0
        self.gimbal_angle_rad = math.radians(-30.0)
        self.pitch_trim = 0.0
        self.yaw_cmd = 0.0
        self.have_odom = False
        self.last_update_time = time.monotonic()
        self.last_bbox_time = 0.0
        self.last_pixel_u = self.u0
        self.last_pixel_v = self.v0
        self.target_vel = [0.0, 0.0, 0.0]
        self.last_target_time = 0.0
        self.last_eu_sign = 0.0

        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )
        self.drone_pos = [0.0, 0.0, 0.0]
        self.target_pos = None

        self.create_subscription(CameraInfo, '/camera_info', self.camera_info_cb, sensor_qos)
        self.create_subscription(BoundingBox2D, '/hpad/bbox', self.bbox_cb, 10)
        self.create_subscription(Bool, '/hpad/detected', self.detected_cb, 10)
        self.create_subscription(String, '/ekf/tracking_mode', self.tracking_mode_cb, 10)
        self.create_subscription(String, '/mission/phase', self.phase_cb, sensor_qos)
        self.create_subscription(Odometry, '/odom', self.odom_cb, sensor_qos)
        self.create_subscription(Odometry, '/ekf/target_state', self.target_cb, 10)
        self.create_subscription(
            Float32, '/gimbal/target_angle_deg', self.gimbal_angle_cb, sensor_qos
        )

        self.trim_pub = self.create_publisher(Float32, '/ibvs/pitch_trim_deg', 10)
        self.yaw_pub = self.create_publisher(Float64, '/ibvs/yaw_cmd', 10)

        self.create_timer(1.0 / 30.0, self.fallback_timer_cb)

        self.get_logger().info('IBVS controller ready with Nadir Singularity Avoidance + Pitch Compensation')

    def camera_info_cb(self, msg: CameraInfo):
        if msg.k[0] > 0.0:
            self.fx = float(msg.k[0])
            self.fy = float(msg.k[4])
            self.u0 = float(msg.k[2])
            self.v0 = float(msg.k[5])

    def gimbal_angle_cb(self, msg: Float32):
        if math.isfinite(msg.data):
            self.gimbal_angle_rad = math.radians(msg.data)

    def detected_cb(self, msg: Bool):
        self.detected = msg.data

    def tracking_mode_cb(self, msg: String):
        self.tracking_mode = msg.data

    def phase_cb(self, msg: String):
        previous_phase = self.phase
        self.phase = msg.data
        if self.phase != previous_phase:
            self.pitch_trim = 0.0

        if previous_phase == 'SEARCH' and self.phase == 'FOLLOW' and self.have_odom:
            self.yaw_cmd = self.drone_yaw
            self.last_update_time = time.monotonic()
            self.get_logger().info(
                f'SEARCH->FOLLOW: reset yaw_cmd to {math.degrees(self.drone_yaw):.1f} deg'
            )

    def odom_cb(self, msg: Odometry):
        p = msg.pose.pose.position
        self.drone_pos = [p.x, p.y, p.z]
        q = msg.pose.pose.orientation
        sinp = 2.0 * (q.w * q.y - q.z * q.x)
        self.drone_pitch = math.asin(max(-1.0, min(1.0, sinp)))
        self.drone_yaw = quaternion_to_yaw(q)
        if not self.have_odom:
            self.yaw_cmd = self.drone_yaw
            self.have_odom = True

    def target_cb(self, msg: Odometry):
        p = msg.pose.pose.position
        self.target_pos = [p.x, p.y, p.z]
        v = msg.twist.twist.linear
        self.target_vel = [v.x, v.y, v.z]
        self.last_target_time = time.monotonic()

    def bbox_cb(self, msg: BoundingBox2D):
        self.last_pixel_u = float(msg.center.position.x)
        self.last_pixel_v = float(msg.center.position.y)
        self.last_bbox_time = time.monotonic()

    def _update_pitch_trim(self, correction: float, target_is_usable: bool, has_pixel: bool, dt: float):
        # Compensate for drone body pitch (drone_pitch tilts camera up/down when accelerating/braking)
        correction += self.drone_pitch
        if target_is_usable:
            trim_target = correction
        elif has_pixel:
            trim_target = self.pitch_trim + correction
        else:
            trim_target = self.pitch_trim
        trim_target = clamp(trim_target, -self.pitch_trim_limit, self.pitch_trim_limit)
        max_step = self.pitch_rate_limit * dt
        limited = clamp(trim_target, self.pitch_trim - max_step, self.pitch_trim + max_step)
        self.pitch_trim = (
            self.ema_alpha * limited + (1.0 - self.ema_alpha) * self.pitch_trim
        )

    def _compute_ibvs(self, u: float | None, v: float | None, dt: float):
        if self.phase == 'LAND':
            self.pitch_trim = 0.0
            # In LAND phase or near touchdown, lock yaw steady (no spinning under nadir)
            self.yaw_cmd = self.drone_yaw
            self._publish_commands()
            return

        if self.phase in ('FOLLOW', 'APPROACH'):
            now = time.monotonic()
            target_state_age = (now - self.last_target_time) if self.last_target_time > 0.0 else float('inf')
            target_is_usable = (
                self.target_pos is not None
                and self.tracking_mode in ('TRACKING', 'PREDICTING', 'PREDICTING_DEGRADED')
                and target_state_age <= self.target_state_timeout
            )
            has_pixel = u is not None and v is not None and self.detected

            dx = dy = dist_h = 0.0
            if target_is_usable:
                dx = self.target_pos[0] - self.drone_pos[0]
                dy = self.target_pos[1] - self.drone_pos[1]
                dist_h = math.hypot(dx, dy)

            correction = 0.0
            if has_pixel:
                correction = pixel_pitch_correction(
                    v - self.v0, self.K_pitch, self.fy, self.pitch_pixel_trim_gain
                )
            self._update_pitch_trim(correction, target_is_usable, has_pixel, dt)

            # Nadir Singularity Avoidance:
            # When camera pitch < -60 deg or horizontal distance < 1.0m,
            # yawing based on pixel error or atan2(dy, dx) causes rapid spinning!
            # Therefore disable optical yaw rotation at nadir.
            is_nadir_zone = (
                self.gimbal_angle_rad < self.nadir_yaw_cutoff_rad
                or (target_is_usable and dist_h < self.nadir_distance_cutoff_m)
            )

            yaw_ff = 0.0
            if target_is_usable and dist_h > 1.0 and not is_nadir_zone:
                yaw_ff = tangential_yaw_feedforward(
                    dx, dy, self.target_vel[0], self.target_vel[1]
                )

            if has_pixel and not is_nadir_zone:
                eu = u - self.u0
                self.last_eu_sign = 1.0 if eu >= 0 else -1.0
                eu_eff = deadband(eu, YAW_DEADBAND_PX)
                pitch_cos = max(0.4, math.cos(self.gimbal_angle_rad))
                desired_yaw = self.drone_yaw - self.K_yaw * (eu_eff / self.fx) * pitch_cos
                self.yaw_cmd = slew_angle(
                    self.yaw_cmd, desired_yaw, self.yaw_rate_limit * dt
                )
                if abs(yaw_ff) > 0.01:
                    self.yaw_cmd = wrap_angle(self.yaw_cmd + yaw_ff * dt)
            elif target_is_usable and dist_h > 0.3 and not is_nadir_zone:
                pred_dx = dx + self.target_vel[0] * 0.3
                pred_dy = dy + self.target_vel[1] * 0.3
                target_yaw = math.atan2(pred_dy, pred_dx)
                self.yaw_cmd = slew_angle(
                    self.yaw_cmd, target_yaw, self.yaw_rate_limit * dt
                )
            else:
                # Inside nadir zone or no valid target: hold current heading smoothly
                self.yaw_cmd = slew_angle(
                    self.yaw_cmd, self.drone_yaw, self.yaw_rate_limit * dt
                )

            self._publish_commands()

    def _publish_commands(self):
        trim_msg = Float32()
        trim_msg.data = float(math.degrees(self.pitch_trim))
        self.trim_pub.publish(trim_msg)

        yaw_msg = Float64()
        yaw_msg.data = float(self.yaw_cmd)
        self.yaw_pub.publish(yaw_msg)

    def _search_yaw(self, dt: float) -> float:
        if self.target_pos is None or not self.have_odom or self.drone_pos[2] < 0.3:
            return self.drone_yaw
        dx = self.target_pos[0] - self.drone_pos[0]
        dy = self.target_pos[1] - self.drone_pos[1]
        if math.hypot(dx, dy) < 0.3:
            return self.drone_yaw
        return slew_angle(self.yaw_cmd, math.atan2(dy, dx), self.yaw_rate_limit * dt)

    def fallback_timer_cb(self):
        now = time.monotonic()
        dt = now - self.last_update_time
        dt = max(0.001, min(dt, 0.5))

        if self.phase == 'SEARCH':
            self.pitch_trim = 0.0
            self.last_update_time = now
            self.yaw_cmd = self._search_yaw(dt)
            self._publish_commands()

        elif self.phase == 'IDLE':
            self.pitch_trim = 0.0
            self.yaw_cmd = self.drone_yaw
            self._publish_commands()

        elif self.phase in ('FOLLOW', 'APPROACH'):
            self.last_update_time = now
            self._compute_ibvs(
                self.last_pixel_u if self.detected else None,
                self.last_pixel_v if self.detected else None,
                dt,
            )


def main():
    rclpy.init()
    node = IBVSController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
