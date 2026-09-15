#!/usr/bin/env python3
"""APF (Artificial Potential Field) Obstacle Avoidance Planner.

Port from MATLAB ``Avoidance/APFplanner_1_Obstacle.m`` sang Python + ROS 2.

Thuật toán:
  1. Lực hút (Attractive): F_att = 2 * k_att * (goal - pos)
  2. Lực đẩy (Repulsive) + lực tiếp tuyến cho mỗi vật cản trong phạm vi d0
  3. Tổng lực → chuẩn hóa → velocity command ≤ v_max
  4. Yaw command theo hướng velocity ngang

ROS 2 interface:
  Subscribe: /odom, /ekf/target_state, /mission/phase
  Publish:   /apf/velocity_cmd, /apf/force_markers
"""

from __future__ import annotations

import math
import os
import sys
import xml.etree.ElementTree as ET
from dataclasses import dataclass, field
from typing import List, Optional, Tuple

import numpy as np

# ---------------------------------------------------------------------------
# Core APF algorithm (no ROS dependency)
# ---------------------------------------------------------------------------

@dataclass
class APFParams:
    """Tunable APF parameters."""
    d0: float = 5.0          # Obstacle influence distance (m)
    v_max: float = 2.0       # Maximum velocity (m/s)
    k_att: float = 10.0      # Attractive gain
    k_rep: float = 90000.0   # Repulsive gain
    goal_threshold: float = 0.15  # Stop distance near goal (m)


@dataclass
class APFResult:
    """Output of one APF computation cycle."""
    velocity: np.ndarray       # [vx, vy, vz] in world ENU (m/s)
    yaw_cmd: float             # heading angle (rad)
    f_att: np.ndarray          # attractive force (debug)
    f_rep: np.ndarray          # total repulsive force (debug)
    f_total: np.ndarray        # total force (debug)
    at_goal: bool              # True if within goal_threshold


class APFCore:
    """Pure-Python APF planner — no ROS dependency.

    Directly mirrors the MATLAB logic in ``APFplanner_1_Obstacle.m`` with the
    addition of tangential force to avoid local minima, and per-obstacle
    cylinder-surface nearest-point computation.
    """

    def __init__(self, params: Optional[APFParams] = None) -> None:
        self.params = params or APFParams()

    def compute(
        self,
        pos: np.ndarray,
        goal: np.ndarray,
        obstacles: List[Tuple[float, float, float]],
        *,
        k_rep_override: Optional[float] = None,
    ) -> APFResult:
        """Compute velocity command given drone position, goal, and obstacles.

        Parameters
        ----------
        pos : (3,) drone position [x, y, z] ENU
        goal : (3,) target/goal position [x, y, z] ENU
        obstacles : list of (x, y, z) obstacle center positions ENU
        k_rep_override : if set, overrides self.params.k_rep (for APPROACH)

        Returns
        -------
        APFResult with velocity command and debug forces
        """
        p = self.params
        k_rep = k_rep_override if k_rep_override is not None else p.k_rep

        pos = np.asarray(pos, dtype=float)
        goal = np.asarray(goal, dtype=float)

        # --- Distance to goal ---
        d_goal_vec = goal - pos
        d_goal_mag = np.linalg.norm(d_goal_vec)

        # --- Attractive force ---
        f_att = 2.0 * p.k_att * d_goal_vec

        # --- Repulsive force (sum over all obstacles) ---
        f_rep = np.zeros(3)
        for obs in obstacles:
            obs_pos = np.asarray(obs, dtype=float)
            d_vec = pos - obs_pos          # vector from obstacle to drone
            d = np.linalg.norm(d_vec)

            if d < 1e-6:
                d = 1e-6                   # avoid division by zero
            if d > p.d0:
                continue                   # outside influence zone

            # Unit vector away from obstacle
            grad_d = d_vec / d

            # Repulsive magnitude (from MATLAB)
            rep_mag = 2.0 * k_rep * (1.0 / d - 1.0 / p.d0) * (1.0 / (d * d))

            # Tangential component to avoid local minima (cross with z-up)
            tan_dir = np.cross(np.array([0.0, 0.0, 1.0]), grad_d)
            tan_norm = np.linalg.norm(tan_dir)
            if tan_norm > 1e-3:
                tan_dir = tan_dir / tan_norm
            else:
                tan_dir = np.array([0.0, 1.0, 0.0])
            f_tan = rep_mag * tan_dir

            f_rep += rep_mag * grad_d + f_tan

        # --- Total force ---
        f_total = f_att + f_rep
        f_norm = np.linalg.norm(f_total)

        # --- Velocity command ---
        at_goal = d_goal_mag < p.goal_threshold

        if at_goal:
            velocity = np.zeros(3)
        elif f_norm > 1e-4:
            velocity = p.v_max * f_total / f_norm
        else:
            velocity = np.zeros(3)

        # --- Yaw command (heading toward velocity direction) ---
        v_horizontal = math.sqrt(velocity[0] ** 2 + velocity[1] ** 2)
        if v_horizontal > 0.05:
            yaw_cmd = math.atan2(velocity[1], velocity[0])
        else:
            yaw_cmd = 0.0

        return APFResult(
            velocity=velocity,
            yaw_cmd=yaw_cmd,
            f_att=f_att,
            f_rep=f_rep,
            f_total=f_total,
            at_goal=at_goal,
        )


# ---------------------------------------------------------------------------
# Utility: load obstacle positions from Gazebo SDF world
# ---------------------------------------------------------------------------

def load_cylinder_obstacles_from_sdf(
    sdf_path: str,
) -> List[Tuple[float, float, float, float, float]]:
    """Parse Gazebo SDF and return cylinder obstacles as (cx, cy, r, h, cz).

    cz is the center altitude of the cylinder (= h/2 if sitting on ground).
    """
    if not os.path.isfile(sdf_path):
        return []
    root = ET.parse(sdf_path).getroot()
    result = []
    for model in root.findall('./world/model'):
        name = model.attrib.get('name', '')
        if not (name.startswith('cyl_') or name.startswith('cylinder_obs_')):
            continue
        pose = model.findtext('pose', '').split()
        cyl = model.find('.//cylinder')
        if len(pose) < 3 or cyl is None:
            continue
        cx, cy = float(pose[0]), float(pose[1])
        r = float(cyl.findtext('radius', '0.5'))
        h = float(cyl.findtext('length', '1.0'))
        cz = float(pose[2])  # SDF pose z is the center of the cylinder
        result.append((cx, cy, r, h, cz))
    return result


def cylinder_nearest_point(
    drone_pos: np.ndarray,
    cx: float, cy: float, r: float, h: float, cz: float,
) -> np.ndarray:
    """Return the nearest surface point on a vertical cylinder to the drone.

    The cylinder axis is along Z from (cz - h/2) to (cz + h/2).
    """
    # Horizontal distance from cylinder axis
    dx = drone_pos[0] - cx
    dy = drone_pos[1] - cy
    dist_h = math.sqrt(dx * dx + dy * dy)

    # Nearest point on cylinder surface (horizontal)
    if dist_h > 1e-6:
        nx = cx + r * dx / dist_h
        ny = cy + r * dy / dist_h
    else:
        nx = cx + r  # arbitrary direction when on axis
        ny = cy

    # Nearest z on cylinder
    z_low = cz - h / 2.0
    z_high = cz + h / 2.0
    nz = float(np.clip(drone_pos[2], z_low, z_high))

    return np.array([nx, ny, nz])


# ---------------------------------------------------------------------------
# ROS 2 Node wrapper
# ---------------------------------------------------------------------------

def _create_ros_node():
    """Lazy import rclpy and create the APF planner ROS 2 node."""
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
    from geometry_msgs.msg import Twist, Vector3
    from nav_msgs.msg import Odometry
    from std_msgs.msg import String
    from visualization_msgs.msg import Marker, MarkerArray

    class APFPlannerNode(Node):
        """ROS 2 node wrapping the APF planner core.

        Subscribes to drone odometry and EKF target state, publishes velocity
        commands and debug force markers.  Obstacle positions are loaded from
        the Gazebo SDF world file at startup.
        """

        def __init__(self):
            super().__init__('apf_planner')

            # ── Parameters ───────────────────────────────────────────────
            self.declare_parameter('world_sdf', '')
            self.declare_parameter('d0', 5.0)
            self.declare_parameter('v_max', 2.0)
            self.declare_parameter('k_att', 10.0)
            self.declare_parameter('k_rep', 90000.0)
            self.declare_parameter('k_rep_approach', 45000.0)
            self.declare_parameter('goal_threshold', 0.15)

            params = APFParams(
                d0=self.get_parameter('d0').value,
                v_max=self.get_parameter('v_max').value,
                k_att=self.get_parameter('k_att').value,
                k_rep=self.get_parameter('k_rep').value,
                goal_threshold=self.get_parameter('goal_threshold').value,
            )
            self.apf = APFCore(params)
            self.k_rep_approach = self.get_parameter('k_rep_approach').value

            # ── State ────────────────────────────────────────────────────
            self.drone_pos = np.zeros(3)
            self.goal_pos = None               # from EKF target state
            self.has_odom = False
            self.phase = 'IDLE'

            # ── Load obstacles from SDF ──────────────────────────────────
            sdf_path = self.get_parameter('world_sdf').value
            raw_obs = load_cylinder_obstacles_from_sdf(sdf_path)
            self.cylinders = raw_obs  # (cx, cy, r, h, cz)
            self.get_logger().info(
                f'Loaded {len(self.cylinders)} cylinder obstacles from '
                f'{sdf_path or "(none)"}'
            )

            # ── Subscribers ──────────────────────────────────────────────
            sensor_qos = QoSProfile(
                reliability=ReliabilityPolicy.BEST_EFFORT,
                history=HistoryPolicy.KEEP_LAST, depth=5,
            )
            self.create_subscription(
                Odometry, '/odom', self.odom_cb, sensor_qos,
            )
            self.create_subscription(
                Odometry, '/ekf/target_state', self.target_cb, 10,
            )
            self.create_subscription(
                String, '/mission/phase', self.phase_cb, 10,
            )

            # ── Publishers ───────────────────────────────────────────────
            self.vel_pub = self.create_publisher(Twist, '/apf/velocity_cmd', 10)
            self.marker_pub = self.create_publisher(
                MarkerArray, '/apf/force_markers', 10,
            )

            # ── Timer (compute APF at 30 Hz fallback) ────────────────────
            self.create_timer(1.0 / 30.0, self.compute_and_publish)

            self.get_logger().info('APF Planner node ready.')

        # ── Callbacks ────────────────────────────────────────────────────

        def odom_cb(self, msg: Odometry):
            p = msg.pose.pose.position
            self.drone_pos = np.array([p.x, p.y, p.z])
            self.has_odom = True

        def target_cb(self, msg: Odometry):
            p = msg.pose.pose.position
            self.goal_pos = np.array([p.x, p.y, p.z])

        def phase_cb(self, msg: String):
            self.phase = msg.data

        # ── Main loop ────────────────────────────────────────────────────

        def compute_and_publish(self):
            # Don't compute if no odometry or in IDLE/LAND
            if not self.has_odom:
                return
            if self.phase in ('IDLE', 'LAND'):
                self._publish_zero_velocity()
                return

            # If no goal (EKF not publishing), hover
            if self.goal_pos is None:
                self._publish_zero_velocity()
                return

            # Compute nearest surface points for each cylinder obstacle
            obstacle_points = []
            for cx, cy, r, h, cz in self.cylinders:
                nearest = cylinder_nearest_point(
                    self.drone_pos, cx, cy, r, h, cz,
                )
                obstacle_points.append(tuple(nearest))

            # Use reduced repulsive gain in APPROACH phase
            k_rep_override = None
            if self.phase == 'APPROACH':
                k_rep_override = self.k_rep_approach

            result = self.apf.compute(
                self.drone_pos, self.goal_pos, obstacle_points,
                k_rep_override=k_rep_override,
            )

            # Publish velocity command
            cmd = Twist()
            cmd.linear.x = float(result.velocity[0])
            cmd.linear.y = float(result.velocity[1])
            cmd.linear.z = float(result.velocity[2])
            self.vel_pub.publish(cmd)

            # Publish debug markers
            self._publish_force_markers(result)

        def _publish_zero_velocity(self):
            self.vel_pub.publish(Twist())

        def _publish_force_markers(self, result: APFResult):
            """Visualize attractive + repulsive forces as arrows in RViz2."""
            now = self.get_clock().now().to_msg()
            markers = MarkerArray()

            # Arrow: Attractive force (green)
            att = Marker()
            att.header.stamp = now
            att.header.frame_id = 'world'
            att.ns = 'apf_att'
            att.id = 0
            att.type = Marker.ARROW
            att.action = Marker.ADD
            att.scale.x = 0.08   # shaft diameter
            att.scale.y = 0.15   # head diameter
            att.scale.z = 0.0
            att.color.g = 1.0
            att.color.a = 0.8
            start = Vector3(
                x=self.drone_pos[0], y=self.drone_pos[1], z=self.drone_pos[2],
            )
            f_att_norm = np.linalg.norm(result.f_att)
            scale = min(2.0, f_att_norm / max(self.apf.params.k_att, 1.0))
            if f_att_norm > 1e-4:
                direction = result.f_att / f_att_norm * scale
            else:
                direction = np.zeros(3)
            end = Vector3(
                x=self.drone_pos[0] + direction[0],
                y=self.drone_pos[1] + direction[1],
                z=self.drone_pos[2] + direction[2],
            )
            att.points = [start, end]
            markers.markers.append(att)

            # Arrow: Repulsive force (red)
            rep = Marker()
            rep.header.stamp = now
            rep.header.frame_id = 'world'
            rep.ns = 'apf_rep'
            rep.id = 1
            rep.type = Marker.ARROW
            rep.action = Marker.ADD
            rep.scale.x = 0.08
            rep.scale.y = 0.15
            rep.scale.z = 0.0
            rep.color.r = 1.0
            rep.color.a = 0.8
            f_rep_norm = np.linalg.norm(result.f_rep)
            scale = min(2.0, f_rep_norm / max(self.apf.params.k_rep * 0.0001, 1.0))
            if f_rep_norm > 1e-4:
                direction = result.f_rep / f_rep_norm * scale
            else:
                direction = np.zeros(3)
            rep.points = [
                start,
                Vector3(
                    x=self.drone_pos[0] + direction[0],
                    y=self.drone_pos[1] + direction[1],
                    z=self.drone_pos[2] + direction[2],
                ),
            ]
            markers.markers.append(rep)

            # Arrow: Total / velocity direction (blue)
            total = Marker()
            total.header.stamp = now
            total.header.frame_id = 'world'
            total.ns = 'apf_total'
            total.id = 2
            total.type = Marker.ARROW
            total.action = Marker.ADD
            total.scale.x = 0.10
            total.scale.y = 0.18
            total.scale.z = 0.0
            total.color.b = 1.0
            total.color.a = 0.9
            vel_norm = np.linalg.norm(result.velocity)
            if vel_norm > 0.05:
                direction = result.velocity / vel_norm * min(2.0, vel_norm)
            else:
                direction = np.zeros(3)
            total.points = [
                start,
                Vector3(
                    x=self.drone_pos[0] + direction[0],
                    y=self.drone_pos[1] + direction[1],
                    z=self.drone_pos[2] + direction[2],
                ),
            ]
            markers.markers.append(total)

            self.marker_pub.publish(markers)

    return APFPlannerNode


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main():
    import rclpy

    rclpy.init()
    NodeClass = _create_ros_node()
    node = NodeClass()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print('\n[APF] Shutting down...')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
