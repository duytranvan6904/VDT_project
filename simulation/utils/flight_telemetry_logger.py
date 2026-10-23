#!/usr/bin/env python3
"""Flight Telemetry Logger Node.

Thu thập đồng bộ dữ liệu toàn bộ hệ thống Quadrotor trong quá trình bay mô phỏng:
- Mốc thời gian mô phỏng (Gazebo Sim Time qua /clock)
- Trạng thái FSM (/mission/phase)
- Vị trí, vận tốc, góc hướng drone (/odom)
- Vị trí bãi đáp thực tế (/hpad/ground_truth hoặc từ tham số cố định)
- Ước lượng vị trí bãi đáp từ EKF (/ekf/target_state)
- Hiệp phương sai an toàn (/landing/uncertainty_radius, /landing/safe_to_land)
- Góc ngẩng gimbal (/ibvs/gimbal_pitch) và lệnh quay yaw (/ibvs/yaw_cmd)
- Cự ly an toàn tới chướng ngại vật gần nhất (tính từ các cột trụ trong SDF)
- Sự kiện tiếp đất (/landing/touchdown)

Xuất dữ liệu:
- File CSV chuỗi thời gian chi tiết @ 20Hz
- File JSON tổng kết chỉ số KPI cho lần bay
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import os
import sys
import time
import xml.etree.ElementTree as ET
from typing import List, Dict, Any, Optional

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
if REPO_ROOT not in sys.path:
    sys.path.insert(0, REPO_ROOT)
from simulation.utils.world_geometry import load_hpad_pose_from_sdf
from simulation.utils.coordinate_frames import rotate_body_vector_to_world

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from rosgraph_msgs.msg import Clock
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, Twist
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Float64, String
from diagnostic_msgs.msg import DiagnosticStatus


def _quaternion_to_euler(q) -> tuple[float, float, float]:
    sinr_cosp = 2.0 * (q.w * q.x + q.y * q.z)
    cosr_cosp = 1.0 - 2.0 * (q.x * q.x + q.y * q.y)
    roll = math.atan2(sinr_cosp, cosr_cosp)
    sinp = 2.0 * (q.w * q.y - q.z * q.x)
    pitch = math.asin(max(-1.0, min(1.0, sinp)))
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return roll, pitch, yaw


def _wrap_angle(angle: float) -> float:
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def load_cylinders_from_sdf(sdf_path: str) -> List[Dict[str, float]]:
    cylinders = []
    if not os.path.isfile(sdf_path):
        return cylinders
    try:
        tree = ET.parse(sdf_path)
        root = tree.getroot()
        world = root.find('world') or root
        for model in world.findall('model'):
            m_name = model.get('name', '')
            if not m_name.startswith(('cyl_', 'cylinder_obs_')):
                continue
            pose_elem = model.find('pose')
            if pose_elem is None or not pose_elem.text:
                continue
            parts = [float(p) for p in pose_elem.text.split()]
            if len(parts) < 3:
                continue
            cx, cy = parts[0], parts[1]
            r = 0.4
            h = 3.0
            cyl_elem = model.find('.//cylinder')
            if cyl_elem is not None:
                r_elem = cyl_elem.find('radius')
                h_elem = cyl_elem.find('length')
                if r_elem is not None and r_elem.text:
                    r = float(r_elem.text)
                if h_elem is not None and h_elem.text:
                    h = float(h_elem.text)
            cylinders.append({'cx': cx, 'cy': cy, 'cz': parts[2], 'r': r, 'h': h})
    except Exception as e:
        print(f"[TelemetryLogger] Warning loading cylinders from SDF: {e}")
    return cylinders


class FlightTelemetryLogger(Node):
    def __init__(self, output_csv: str, summary_json: str, world_sdf: str = "", default_hpad_pos: tuple = (3.0, 1.0, 0.02)):
        super().__init__('flight_telemetry_logger')

        if not self.has_parameter('use_sim_time'):
            self.declare_parameter('use_sim_time', True)
        self.output_csv = output_csv
        self.summary_json = summary_json
        self.default_hpad_pos = default_hpad_pos
        self.cylinders = load_cylinders_from_sdf(world_sdf) if world_sdf else []

        # State storage
        self.sim_time = 0.0
        self.start_sim_time = None
        self.drone_pos = [0.0, 0.0, 0.0]
        self.drone_vel = [0.0, 0.0, 0.0]
        self.drone_roll = 0.0
        self.drone_pitch = 0.0
        self.drone_yaw = 0.0
        self.body_angular_velocity = [0.0, 0.0, 0.0]
        self.has_drone_odom = False

        self.target_pos = list(default_hpad_pos)
        self.target_vel = [0.0, 0.0, 0.0]
        self.has_target_gt = False

        self.ekf_pos = [0.0, 0.0, 0.0]
        self.ekf_vel = [0.0, 0.0, 0.0]
        self.has_ekf = False

        self.fsm_phase = "IDLE"
        self.tracking_mode = "UNKNOWN"
        self.follow_yaw_source = "UNKNOWN"
        self.apf_target_velocity_ff = [0.0, 0.0]
        self.gimbal_pitch = 0.0
        self.gimbal_pitch_actual: float | None = None
        self.ibvs_yaw_cmd = 0.0
        self.mission_yaw_cmd: float | None = None
        self.safe_to_land = False
        self.uncertainty_2sigma = 0.0
        self.touchdown = False
        self.touchdown_seen = False
        self.first_touchdown_time = None
        self.touchdown_snapshot = None

        # Statistics trackers
        self.takeoff_time = None
        self.first_follow_time = None
        self.min_obstacle_dist = 999.0
        self.yaw_errors_in_follow = []
        self.attitude_samples: list[dict[str, float | str]] = []
        self.records_count = 0
        self.wall_start_time = time.monotonic()
        self.last_log_attitude_wall_time: float | None = None
        self.last_log_yaw: float | None = None
        self.body_yaw_rate_wall = 0.0

        # ROS 2 Subscribers with Best Effort QoS where needed
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        self.create_subscription(Clock, '/clock', self._clock_cb, sensor_qos)
        self.create_subscription(Odometry, '/odom', self._odom_cb, sensor_qos)
        self.create_subscription(PoseStamped, '/hpad/ground_truth', self._target_gt_cb, 10)
        self.create_subscription(Odometry, '/ekf/target_state', self._ekf_cb, 10)
        self.create_subscription(String, '/mission/phase', self._phase_cb, 10)
        self.create_subscription(String, '/ekf/tracking_mode', self._tracking_mode_cb, 10)
        self.create_subscription(
            String, '/mission/follow_yaw_source', self._follow_yaw_source_cb, 10,
        )
        self.create_subscription(
            Twist, '/apf/target_velocity_ff', self._apf_target_ff_cb, 10,
        )
        self.create_subscription(Float64, '/ibvs/gimbal_pitch', self._gimbal_cb, 10)
        self.create_subscription(JointState, '/joint_states', self._joint_state_cb, sensor_qos)
        self.create_subscription(Float64, '/ibvs/yaw_cmd', self._yaw_cmd_cb, 10)
        self.create_subscription(
            Float64, '/mission/yaw_setpoint', self._mission_yaw_cb, 10,
        )
        self.create_subscription(Bool, '/landing/safe_to_land', self._safe_cb, 10)
        self.create_subscription(Float64, '/landing/uncertainty_radius', self._unc_cb, 10)
        self.create_subscription(Bool, '/landing/touchdown', self._touchdown_cb, 10)

        # Prepare CSV Writer
        os.makedirs(os.path.dirname(os.path.abspath(self.output_csv)), exist_ok=True)
        self.csv_file = open(self.output_csv, mode='w', newline='')
        self.fieldnames = [
            'sim_time', 'wall_time_s', 'phase',
            'drone_x', 'drone_y', 'drone_z',
            'drone_vx', 'drone_vy', 'drone_vz',
            'drone_roll_deg', 'drone_pitch_deg', 'drone_yaw',
            'body_roll_rate_deg_s', 'body_pitch_rate_deg_s',
            'body_yaw_rate_deg_s', 'body_yaw_rate_wall_deg_s',
            'mission_yaw_cmd_deg', 'ibvs_yaw_cmd_deg',
            'tracking_mode', 'follow_yaw_source',
            'apf_target_ff_vx', 'apf_target_ff_vy',
            'target_x', 'target_y', 'target_z',
            'ekf_x', 'ekf_y', 'ekf_z',
            'ekf_vx', 'ekf_vy', 'ekf_vz',
            'rel_dist_xy', 'rel_dist_3d', 'ekf_err_3d',
            'yaw_los_err_deg', 'gimbal_pitch_deg', 'gimbal_pitch_actual_deg',
            'uncertainty_2sigma', 'safe_to_land',
            'min_obstacle_dist', 'touchdown'
        ]
        self.csv_writer = csv.DictWriter(self.csv_file, fieldnames=self.fieldnames)
        self.csv_writer.writeheader()

        # Logging timer @ 20 Hz
        self.log_timer = self.create_timer(0.05, self._log_sample)
        self.get_logger().info(f"Telemetry Logger started. Writing to: {self.output_csv}")

    def _clock_cb(self, msg: Clock):
        t = msg.clock.sec + msg.clock.nanosec * 1e-9
        self.sim_time = t
        if self.start_sim_time is None and t > 0:
            self.start_sim_time = t

    def _odom_cb(self, msg: Odometry):
        self.drone_pos = [
            float(msg.pose.pose.position.x),
            float(msg.pose.pose.position.y),
            float(msg.pose.pose.position.z)
        ]
        p = msg.twist.twist.linear
        q = msg.pose.pose.orientation
        self.drone_vel = rotate_body_vector_to_world(
            (q.x, q.y, q.z, q.w), (p.x, p.y, p.z),
        ).tolist()
        self.drone_roll, self.drone_pitch, self.drone_yaw = _quaternion_to_euler(
            msg.pose.pose.orientation
        )
        angular = msg.twist.twist.angular
        self.body_angular_velocity = [
            float(angular.x), float(angular.y), float(angular.z),
        ]
        self.has_drone_odom = True

    def _target_gt_cb(self, msg: PoseStamped):
        self.target_pos = [
            float(msg.pose.position.x),
            float(msg.pose.position.y),
            float(msg.pose.position.z)
        ]
        self.has_target_gt = True

    def _ekf_cb(self, msg: Odometry):
        self.ekf_pos = [
            float(msg.pose.pose.position.x),
            float(msg.pose.pose.position.y),
            float(msg.pose.pose.position.z)
        ]
        self.ekf_vel = [
            float(msg.twist.twist.linear.x),
            float(msg.twist.twist.linear.y),
            float(msg.twist.twist.linear.z)
        ]
        self.has_ekf = True

    def _phase_cb(self, msg: String):
        prev = self.fsm_phase
        self.fsm_phase = msg.data

        if prev == "IDLE" and self.fsm_phase in ("SEARCH", "FOLLOW"):
            if self.takeoff_time is None:
                self.takeoff_time = self.sim_time

        if self.fsm_phase == "FOLLOW" and self.first_follow_time is None:
            self.first_follow_time = self.sim_time

    def _tracking_mode_cb(self, msg: String):
        self.tracking_mode = str(msg.data)

    def _follow_yaw_source_cb(self, msg: String):
        self.follow_yaw_source = str(msg.data)

    def _apf_target_ff_cb(self, msg: Twist):
        self.apf_target_velocity_ff = [
            float(msg.linear.x), float(msg.linear.y),
        ]

    def _gimbal_cb(self, msg: Float64):
        self.gimbal_pitch = float(msg.data)

    def _joint_state_cb(self, msg: JointState):
        if 'CameraJoint' in msg.name:
            index = msg.name.index('CameraJoint')
            if index < len(msg.position):
                self.gimbal_pitch_actual = float(msg.position[index])

    def _yaw_cmd_cb(self, msg: Float64):
        self.ibvs_yaw_cmd = float(msg.data)

    def _mission_yaw_cb(self, msg: Float64):
        self.mission_yaw_cmd = float(msg.data)

    def _safe_cb(self, msg: Bool):
        self.safe_to_land = bool(msg.data)

    def _unc_cb(self, msg: Float64):
        self.uncertainty_2sigma = float(msg.data)

    def _touchdown_cb(self, msg: Bool):
        self.touchdown = bool(msg.data)
        if self.touchdown and self.first_touchdown_time is None:
            self.first_touchdown_time = self.sim_time
            self.touchdown_seen = True
            self.touchdown_snapshot = {
                'drone_pos': list(self.drone_pos),
                'drone_vel': list(self.drone_vel),
                'drone_yaw': self.drone_yaw,
                'target_pos': list(self.target_pos),
                'phase': self.fsm_phase,
                'sim_time': self.sim_time,
            }

    def _compute_min_obstacle_distance(self) -> float:
        if not self.cylinders:
            return 999.0
        dx, dy, dz = self.drone_pos
        min_d = 999.0
        for cyl in self.cylinders:
            cx, cy = cyl['cx'], cyl['cy']
            r, h = cyl['r'], cyl['h']
            cz = cyl.get('cz', 0.5 * h)
            radial_gap = max(0.0, math.hypot(dx - cx, dy - cy) - r)
            vertical_gap = max(0.0, abs(dz - cz) - 0.5 * h)
            clearance = math.hypot(radial_gap, vertical_gap) - 0.30
            if clearance < min_d:
                min_d = clearance
        return min_d

    def _log_sample(self):
        if not self.has_drone_odom:
            return

        wall_now = time.monotonic()
        if self.last_log_attitude_wall_time is not None and self.last_log_yaw is not None:
            dt_wall = wall_now - self.last_log_attitude_wall_time
            if 1e-3 < dt_wall <= 1.0:
                measured_wall_yaw_rate = (
                    _wrap_angle(self.drone_yaw - self.last_log_yaw) / dt_wall
                )
                self.body_yaw_rate_wall = (
                    0.35 * measured_wall_yaw_rate
                    + 0.65 * self.body_yaw_rate_wall
                )
        self.last_log_attitude_wall_time = wall_now
        self.last_log_yaw = self.drone_yaw

        # Relative distances
        dx = self.drone_pos[0] - self.target_pos[0]
        dy = self.drone_pos[1] - self.target_pos[1]
        dz = self.drone_pos[2] - self.target_pos[2]
        rel_xy = math.hypot(dx, dy)
        rel_3d = math.sqrt(dx*dx + dy*dy + dz*dz)

        # EKF error vs Ground Truth
        ekf_err = math.sqrt(
            (self.ekf_pos[0] - self.target_pos[0])**2 +
            (self.ekf_pos[1] - self.target_pos[1])**2 +
            (self.ekf_pos[2] - self.target_pos[2])**2
        ) if self.has_ekf else 0.0

        # Yaw error to LOS
        los_angle = math.atan2(self.target_pos[1] - self.drone_pos[1], self.target_pos[0] - self.drone_pos[0])
        yaw_err = math.degrees(abs(_wrap_angle(self.drone_yaw - los_angle)))

        if self.fsm_phase == "FOLLOW":
            self.yaw_errors_in_follow.append(yaw_err)

        # Obstacle clearance
        obs_d = self._compute_min_obstacle_distance()
        if obs_d < self.min_obstacle_dist and self.drone_pos[2] > 0.5:
            self.min_obstacle_dist = obs_d

        row = {
            'sim_time': f"{self.sim_time:.3f}",
            'wall_time_s': f"{wall_now - self.wall_start_time:.3f}",
            'phase': self.fsm_phase,
            'drone_x': f"{self.drone_pos[0]:.4f}",
            'drone_y': f"{self.drone_pos[1]:.4f}",
            'drone_z': f"{self.drone_pos[2]:.4f}",
            'drone_vx': f"{self.drone_vel[0]:.4f}",
            'drone_vy': f"{self.drone_vel[1]:.4f}",
            'drone_vz': f"{self.drone_vel[2]:.4f}",
            'drone_roll_deg': f"{math.degrees(self.drone_roll):.2f}",
            'drone_pitch_deg': f"{math.degrees(self.drone_pitch):.2f}",
            'drone_yaw': f"{math.degrees(self.drone_yaw):.2f}",
            'body_roll_rate_deg_s': f"{math.degrees(self.body_angular_velocity[0]):.2f}",
            'body_pitch_rate_deg_s': f"{math.degrees(self.body_angular_velocity[1]):.2f}",
            'body_yaw_rate_deg_s': f"{math.degrees(self.body_angular_velocity[2]):.2f}",
            'body_yaw_rate_wall_deg_s': f"{math.degrees(self.body_yaw_rate_wall):.2f}",
            'mission_yaw_cmd_deg': (
                f"{math.degrees(self.mission_yaw_cmd):.2f}"
                if self.mission_yaw_cmd is not None else ""
            ),
            'ibvs_yaw_cmd_deg': f"{math.degrees(self.ibvs_yaw_cmd):.2f}",
            'tracking_mode': self.tracking_mode,
            'follow_yaw_source': self.follow_yaw_source,
            'apf_target_ff_vx': f"{self.apf_target_velocity_ff[0]:.4f}",
            'apf_target_ff_vy': f"{self.apf_target_velocity_ff[1]:.4f}",
            'target_x': f"{self.target_pos[0]:.4f}",
            'target_y': f"{self.target_pos[1]:.4f}",
            'target_z': f"{self.target_pos[2]:.4f}",
            'ekf_x': f"{self.ekf_pos[0]:.4f}",
            'ekf_y': f"{self.ekf_pos[1]:.4f}",
            'ekf_z': f"{self.ekf_pos[2]:.4f}",
            'ekf_vx': f"{self.ekf_vel[0]:.4f}",
            'ekf_vy': f"{self.ekf_vel[1]:.4f}",
            'ekf_vz': f"{self.ekf_vel[2]:.4f}",
            'rel_dist_xy': f"{rel_xy:.4f}",
            'rel_dist_3d': f"{rel_3d:.4f}",
            'ekf_err_3d': f"{ekf_err:.4f}",
            'yaw_los_err_deg': f"{yaw_err:.2f}",
            'gimbal_pitch_deg': f"{math.degrees(self.gimbal_pitch):.2f}",
            'gimbal_pitch_actual_deg': (
                f"{math.degrees(self.gimbal_pitch_actual):.2f}"
                if self.gimbal_pitch_actual is not None else ""
            ),
            'uncertainty_2sigma': f"{self.uncertainty_2sigma:.4f}",
            'safe_to_land': int(self.safe_to_land),
            'min_obstacle_dist': f"{obs_d:.3f}",
            'touchdown': int(self.touchdown)
        }
        self.csv_writer.writerow(row)
        self.records_count += 1
        self.attitude_samples.append({
            'phase': self.fsm_phase,
            'roll': math.degrees(self.drone_roll),
            'pitch': math.degrees(self.drone_pitch),
            'roll_rate': math.degrees(self.body_angular_velocity[0]),
            'pitch_rate': math.degrees(self.body_angular_velocity[1]),
            'yaw_rate': math.degrees(self.body_angular_velocity[2]),
            'yaw_rate_wall': math.degrees(self.body_yaw_rate_wall),
        })

    def close(self):
        try:
            self.csv_file.flush()
            self.csv_file.close()
        except Exception:
            pass

        # Calculate final KPIs
        t_lock = None
        if self.first_follow_time is not None and self.takeoff_time is not None:
            t_lock = max(0.0, self.first_follow_time - self.takeoff_time)

        # Camera mount offset from base_link (Oak-D forward 0.20m, left 0.03m)
        x_off, y_off = 0.20, 0.03
        touchdown = self.touchdown_snapshot or {
            'drone_pos': self.drone_pos,
            'drone_vel': self.drone_vel,
            'drone_yaw': self.drone_yaw,
            'target_pos': self.target_pos,
            'phase': self.fsm_phase,
            'sim_time': self.sim_time,
        }
        land_pos = touchdown['drone_pos']
        land_target = touchdown['target_pos']
        land_yaw = touchdown['drone_yaw']
        cam_x = land_pos[0] + x_off * math.cos(land_yaw) - y_off * math.sin(land_yaw)
        cam_y = land_pos[1] + x_off * math.sin(land_yaw) + y_off * math.cos(land_yaw)
        cam_rxy = math.hypot(cam_x - land_target[0], cam_y - land_target[1])
        base_rxy = math.hypot(land_pos[0] - land_target[0], land_pos[1] - land_target[1])

        touchdown_vz = abs(touchdown['drone_vel'][2])
        mean_yaw_err = float(np.mean(self.yaw_errors_in_follow)) if self.yaw_errors_in_follow else 0.0
        attitude_stability = {}
        for phase in ('FOLLOW', 'APPROACH', 'LAND'):
            samples = [
                sample for sample in self.attitude_samples
                if sample['phase'] == phase
            ]
            if not samples:
                continue
            attitude_stability[phase] = {
                'samples': len(samples),
                'roll_abs_p95_deg': round(float(np.percentile(
                    [abs(float(s['roll'])) for s in samples], 95,
                )), 2),
                'roll_abs_peak_deg': round(max(
                    abs(float(s['roll'])) for s in samples
                ), 2),
                'pitch_abs_p95_deg': round(float(np.percentile(
                    [abs(float(s['pitch'])) for s in samples], 95,
                )), 2),
                'pitch_abs_peak_deg': round(max(
                    abs(float(s['pitch'])) for s in samples
                ), 2),
                'roll_rate_abs_p95_deg_s': round(float(np.percentile(
                    [abs(float(s['roll_rate'])) for s in samples], 95,
                )), 2),
                'pitch_rate_abs_p95_deg_s': round(float(np.percentile(
                    [abs(float(s['pitch_rate'])) for s in samples], 95,
                )), 2),
                'yaw_rate_abs_p95_deg_s': round(float(np.percentile(
                    [abs(float(s['yaw_rate'])) for s in samples], 95,
                )), 2),
                'yaw_rate_abs_peak_deg_s': round(max(
                    abs(float(s['yaw_rate'])) for s in samples
                ), 2),
                'yaw_rate_wall_abs_p95_deg_s': round(float(np.percentile(
                    [abs(float(s['yaw_rate_wall'])) for s in samples], 95,
                )), 2),
                'yaw_rate_wall_abs_peak_deg_s': round(max(
                    abs(float(s['yaw_rate_wall'])) for s in samples
                ), 2),
            }
        # Success criteria relaxed to Rxy <= 20cm (half marker size) and safe descent
        success = bool(
            (base_rxy <= 0.20 or cam_rxy <= 0.20)
            and (self.touchdown_seen or land_pos[2] <= 0.15)
            and self.fsm_phase == 'IDLE'
        )

        summary = {
            'records_logged': self.records_count,
            'flight_duration_sim_s': round(self.sim_time - (self.start_sim_time or 0.0), 2),
            'target_lock_time_s': round(t_lock, 3) if t_lock is not None else None,
            'final_landing_error_rxy_cm': round(base_rxy * 100.0, 2),
            'base_link_landing_error_rxy_cm': round(base_rxy * 100.0, 2),
            'cam_landing_error_rxy_cm': round(cam_rxy * 100.0, 2),
            'touchdown_vz_m_s': round(touchdown_vz, 3),
            'touchdown_altitude_m': round(float(land_pos[2]), 3),
            'touchdown_sim_time_s': round(float(touchdown['sim_time']), 3),
            'touchdown_detected': self.touchdown_seen,
            'mean_follow_yaw_error_deg': round(mean_yaw_err, 2),
            'attitude_stability_by_phase': attitude_stability,
            'min_obstacle_clearance_m': round(self.min_obstacle_dist, 3) if self.min_obstacle_dist < 900.0 else None,
            'mission_success': success,
            'strict_kpi_pass': bool(base_rxy <= 0.10),
            'final_phase': self.fsm_phase
        }

        with open(self.summary_json, 'w') as jf:
            json.dump(summary, jf, indent=2)

        self.get_logger().info(f"Summary saved to: {self.summary_json}")


def main():
    parser = argparse.ArgumentParser(description="Flight Telemetry Logger")
    parser.add_argument('--csv', required=True, help="Output CSV path")
    parser.add_argument('--json', required=True, help="Output summary JSON path")
    parser.add_argument('--world-sdf', default="", help="Path to world SDF")
    parser.add_argument('--hpad-x', type=float, default=None)
    parser.add_argument('--hpad-y', type=float, default=None)
    parser.add_argument('--hpad-z', type=float, default=None)
    args, unknown = parser.parse_known_args()

    world_hpad = load_hpad_pose_from_sdf(args.world_sdf)
    hpad_pos = (
        world_hpad[0] if args.hpad_x is None else args.hpad_x,
        world_hpad[1] if args.hpad_y is None else args.hpad_y,
        world_hpad[2] if args.hpad_z is None else args.hpad_z,
    )

    rclpy.init()
    node = FlightTelemetryLogger(
        output_csv=args.csv,
        summary_json=args.json,
        world_sdf=args.world_sdf,
        default_hpad_pos=hpad_pos
    )
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.close()
        try:
            node.destroy_node()
        except Exception:
            pass
        if rclpy.ok():
            try:
                rclpy.shutdown()
            except Exception:
                pass


if __name__ == '__main__':
    main()
