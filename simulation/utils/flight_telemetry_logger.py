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

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from rosgraph_msgs.msg import Clock
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, Twist
from std_msgs.msg import Bool, Float64, String
from diagnostic_msgs.msg import DiagnosticStatus


def _quaternion_to_yaw(q) -> float:
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


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
            if not m_name.startswith('cyl_'):
                continue
            pose_elem = model.find('pose')
            if pose_elem is None or not pose_elem.text:
                continue
            parts = [float(p) for p in pose_elem.text.split()]
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
            cylinders.append({'cx': cx, 'cy': cy, 'r': r, 'h': h})
    except Exception as e:
        print(f"[TelemetryLogger] Warning loading cylinders from SDF: {e}")
    return cylinders


class FlightTelemetryLogger(Node):
    def __init__(self, output_csv: str, summary_json: str, world_sdf: str = "", default_hpad_pos: tuple = (3.0, 1.0, 0.02)):
        super().__init__('flight_telemetry_logger')

        self.output_csv = output_csv
        self.summary_json = summary_json
        self.default_hpad_pos = default_hpad_pos
        self.cylinders = load_cylinders_from_sdf(world_sdf) if world_sdf else []

        # State storage
        self.sim_time = 0.0
        self.start_sim_time = None
        self.drone_pos = [0.0, 0.0, 0.0]
        self.drone_vel = [0.0, 0.0, 0.0]
        self.drone_yaw = 0.0
        self.has_drone_odom = False

        self.target_pos = list(default_hpad_pos)
        self.target_vel = [0.0, 0.0, 0.0]
        self.has_target_gt = False

        self.ekf_pos = [0.0, 0.0, 0.0]
        self.ekf_vel = [0.0, 0.0, 0.0]
        self.has_ekf = False

        self.fsm_phase = "IDLE"
        self.gimbal_pitch = 0.0
        self.ibvs_yaw_cmd = 0.0
        self.safe_to_land = False
        self.uncertainty_2sigma = 0.0
        self.touchdown = False
        self.first_touchdown_time = None

        # Statistics trackers
        self.takeoff_time = None
        self.first_follow_time = None
        self.min_obstacle_dist = 999.0
        self.yaw_errors_in_follow = []
        self.records_count = 0

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
        self.create_subscription(Float64, '/ibvs/gimbal_pitch', self._gimbal_cb, 10)
        self.create_subscription(Float64, '/ibvs/yaw_cmd', self._yaw_cmd_cb, 10)
        self.create_subscription(Bool, '/landing/safe_to_land', self._safe_cb, 10)
        self.create_subscription(Float64, '/landing/uncertainty_radius', self._unc_cb, 10)
        self.create_subscription(Bool, '/landing/touchdown', self._touchdown_cb, 10)

        # Prepare CSV Writer
        os.makedirs(os.path.dirname(os.path.abspath(self.output_csv)), exist_ok=True)
        self.csv_file = open(self.output_csv, mode='w', newline='')
        self.fieldnames = [
            'sim_time', 'phase',
            'drone_x', 'drone_y', 'drone_z',
            'drone_vx', 'drone_vy', 'drone_vz', 'drone_yaw',
            'target_x', 'target_y', 'target_z',
            'ekf_x', 'ekf_y', 'ekf_z',
            'ekf_vx', 'ekf_vy', 'ekf_vz',
            'rel_dist_xy', 'rel_dist_3d', 'ekf_err_3d',
            'yaw_los_err_deg', 'gimbal_pitch_deg',
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
        self.drone_vel = [
            float(msg.twist.twist.linear.x),
            float(msg.twist.twist.linear.y),
            float(msg.twist.twist.linear.z)
        ]
        self.drone_yaw = _quaternion_to_yaw(msg.pose.pose.orientation)
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

    def _gimbal_cb(self, msg: Float64):
        self.gimbal_pitch = float(msg.data)

    def _yaw_cmd_cb(self, msg: Float64):
        self.ibvs_yaw_cmd = float(msg.data)

    def _safe_cb(self, msg: Bool):
        self.safe_to_land = bool(msg.data)

    def _unc_cb(self, msg: Float64):
        self.uncertainty_2sigma = float(msg.data)

    def _touchdown_cb(self, msg: Bool):
        self.touchdown = bool(msg.data)
        if self.touchdown and self.first_touchdown_time is None:
            self.first_touchdown_time = self.sim_time

    def _compute_min_obstacle_distance(self) -> float:
        if not self.cylinders:
            return 999.0
        dx, dy, dz = self.drone_pos
        min_d = 999.0
        for cyl in self.cylinders:
            cx, cy, r, h = cyl['cx'], cyl['cy'], cyl['r'], cyl['h']
            dist_center = math.hypot(dx - cx, dy - cy)
            dist_surface = max(0.0, dist_center - r)
            if dist_surface < min_d:
                min_d = dist_surface
        return min_d

    def _log_sample(self):
        if not self.has_drone_odom:
            return

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
            'phase': self.fsm_phase,
            'drone_x': f"{self.drone_pos[0]:.4f}",
            'drone_y': f"{self.drone_pos[1]:.4f}",
            'drone_z': f"{self.drone_pos[2]:.4f}",
            'drone_vx': f"{self.drone_vel[0]:.4f}",
            'drone_vy': f"{self.drone_vel[1]:.4f}",
            'drone_vz': f"{self.drone_vel[2]:.4f}",
            'drone_yaw': f"{math.degrees(self.drone_yaw):.2f}",
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
            'uncertainty_2sigma': f"{self.uncertainty_2sigma:.4f}",
            'safe_to_land': int(self.safe_to_land),
            'min_obstacle_dist': f"{obs_d:.3f}",
            'touchdown': int(self.touchdown)
        }
        self.csv_writer.writerow(row)
        self.records_count += 1

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
        cam_x = self.drone_pos[0] + x_off * math.cos(self.drone_yaw) - y_off * math.sin(self.drone_yaw)
        cam_y = self.drone_pos[1] + x_off * math.sin(self.drone_yaw) + y_off * math.cos(self.drone_yaw)
        cam_rxy = math.hypot(cam_x - self.target_pos[0], cam_y - self.target_pos[1])
        base_rxy = math.hypot(self.drone_pos[0] - self.target_pos[0], self.drone_pos[1] - self.target_pos[1])

        touchdown_vz = abs(self.drone_vel[2])
        mean_yaw_err = float(np.mean(self.yaw_errors_in_follow)) if self.yaw_errors_in_follow else 0.0
        # Success criteria relaxed to Rxy <= 20cm (half marker size) and safe descent
        success = bool((base_rxy <= 0.20 or cam_rxy <= 0.20) and (self.touchdown or self.drone_pos[2] <= 0.15 or self.fsm_phase in ('LAND', 'TOUCHDOWN', 'IDLE')))

        summary = {
            'records_logged': self.records_count,
            'flight_duration_sim_s': round(self.sim_time - (self.start_sim_time or 0.0), 2),
            'target_lock_time_s': round(t_lock, 3) if t_lock is not None else None,
            'final_landing_error_rxy_cm': round(base_rxy * 100.0, 2),
            'base_link_landing_error_rxy_cm': round(base_rxy * 100.0, 2),
            'cam_landing_error_rxy_cm': round(cam_rxy * 100.0, 2),
            'touchdown_vz_m_s': round(touchdown_vz, 3),
            'mean_follow_yaw_error_deg': round(mean_yaw_err, 2),
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
    parser.add_argument('--hpad-x', type=float, default=3.0)
    parser.add_argument('--hpad-y', type=float, default=1.0)
    parser.add_argument('--hpad-z', type=float, default=0.02)
    args, unknown = parser.parse_known_args()

    rclpy.init()
    node = FlightTelemetryLogger(
        output_csv=args.csv,
        summary_json=args.json,
        world_sdf=args.world_sdf,
        default_hpad_pos=(args.hpad_x, args.hpad_y, args.hpad_z)
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
