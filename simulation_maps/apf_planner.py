#!/usr/bin/env python3
"""APF (Artificial Potential Field) Obstacle Avoidance Planner.

Port from MATLAB ``Avoidance/APFplanner_1_Obstacle.m`` sang Python + ROS 2.

Thuật toán:
  1. Lực hút (Attractive): F_att = 2 * k_att * (goal - pos)
  2. Lực đẩy (Repulsive) + lực tiếp tuyến cho mỗi vật cản trong phạm vi d0
  3. Tổng lực → chuẩn hóa → velocity command ≤ v_max
  4. Yaw command theo hướng velocity ngang

ROS 2 interface:
  Subscribe: /odom, /ekf/target_state, /ekf/tracking_mode, /mission/phase
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
    d0: float = 3.0          # Obstacle influence distance (m)
    v_max: float = 2.0       # Maximum velocity (m/s)
    k_att: float = 10.0      # Attractive gain
    k_rep: float = 2500.0   # Repulsive gain
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


def make_follow_goal(
    drone_pos: np.ndarray,
    target_pos: np.ndarray,
    follow_distance: float,
    *,
    hold_altitude: bool = True,
    is_3d_distance: bool = True,
) -> np.ndarray:
    """Return the horizontal stand-off point used during FOLLOW.

    APF attracts the drone to a stand-off point behind/towards the target,
    not to the target itself. The altitude is projected onto the current
    drone altitude during FOLLOW; the landing phase owns vertical descent.

    When ``is_3d_distance`` is True (default), ``follow_distance`` represents the
    desired 3D Euclidean slant range from the drone to the physical ground marker.
    By the Pythagorean theorem, the horizontal stand-off distance is:
        R_xy = sqrt(max(0, follow_distance^2 - (z_drone - z_target)^2))
    If the vertical difference exceeds follow_distance, R_xy collapses to 0
    (drone positions directly overhead).
    """
    drone = np.asarray(drone_pos, dtype=float)
    target = np.asarray(target_pos, dtype=float)
    if drone.shape != (3,) or target.shape != (3,):
        raise ValueError('drone_pos and target_pos must be 3-element vectors')

    goal = target.copy()
    delta_xy = target[:2] - drone[:2]
    distance_xy = float(np.linalg.norm(delta_xy))

    if is_3d_distance:
        dz = float(abs(drone[2] - target[2]))
        if follow_distance > dz:
            standoff_xy = float(np.sqrt(follow_distance**2 - dz**2))
        else:
            standoff_xy = 0.0
    else:
        standoff_xy = float(max(0.0, follow_distance))

    if standoff_xy > 0.0:
        if distance_xy > 1e-6:
            direction_to_target = delta_xy / distance_xy
        else:
            # Deterministic fallback prevents the stand-off goal collapsing
            # onto the target when both XY positions temporarily coincide.
            direction_to_target = np.array([1.0, 0.0])
        goal[:2] = target[:2] - direction_to_target * standoff_xy
    else:
        goal[:2] = target[:2]

    if hold_altitude:
        goal[2] = drone[2]
    return goal


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
        # GNRON resolution: do not let obstacles farther than goal repel vehicle away from goal
        effective_d0 = min(p.d0, max(0.4, d_goal_mag))

        for obs in obstacles:
            obs_pos = np.asarray(obs, dtype=float)
            d_vec = pos - obs_pos          # vector from obstacle to drone
            d = np.linalg.norm(d_vec)

            if d < 1e-6:
                d = 1e-6                   # avoid division by zero
            if d > effective_d0:
                continue                   # outside influence zone

            # Unit vector away from obstacle
            grad_d = d_vec / d

            # Repulsive magnitude (from MATLAB)
            rep_mag = 2.0 * k_rep * (1.0 / d - 1.0 / effective_d0) * (1.0 / (d * d))

            # Tangential component to avoid local minima (directed towards goal)
            tan_dir = np.cross(np.array([0.0, 0.0, 1.0]), grad_d)
            tan_norm = np.linalg.norm(tan_dir)
            if tan_norm > 1e-3:
                tan_dir = tan_dir / tan_norm
            else:
                tan_dir = np.array([0.0, 1.0, 0.0])
            if np.dot(tan_dir, d_goal_vec) < 0:
                tan_dir = -tan_dir
            f_tan = 0.8 * rep_mag * tan_dir

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
    from geometry_msgs.msg import Point, Twist, Vector3, TransformStamped
    from nav_msgs.msg import Odometry
    from std_msgs.msg import String
    from tf2_ros import StaticTransformBroadcaster
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
            self.declare_parameter('d0', 2.0)
            self.declare_parameter('v_max', 1.5)
            self.declare_parameter('k_att', 10.0)
            self.declare_parameter('k_rep', 250.0)
            self.declare_parameter('k_rep_approach', 125.0)
            self.declare_parameter('goal_threshold', 0.20)
            self.declare_parameter('follow_distance', 3.5)
            self.declare_parameter('hold_follow_altitude', True)

            params = APFParams(
                d0=self.get_parameter('d0').value,
                v_max=self.get_parameter('v_max').value,
                k_att=self.get_parameter('k_att').value,
                k_rep=self.get_parameter('k_rep').value,
                goal_threshold=self.get_parameter('goal_threshold').value,
            )
            self.apf = APFCore(params)
            self.k_rep_approach = self.get_parameter('k_rep_approach').value
            self.follow_distance = float(self.get_parameter('follow_distance').value)
            self.hold_follow_altitude = bool(
                self.get_parameter('hold_follow_altitude').value
            )

            # ── State ────────────────────────────────────────────────────
            self.drone_pos = np.zeros(3)
            self.goal_pos = None               # from EKF target state
            self.has_odom = False
            self.phase = 'IDLE'
            self.tracking_mode = 'EXPIRED'

            # ── Load obstacles from SDF ──────────────────────────────────
            sdf_path = self.get_parameter('world_sdf').value
            raw_obs = load_cylinder_obstacles_from_sdf(sdf_path)
            self.cylinders = raw_obs  # (cx, cy, r, h, cz)
            self.get_logger().info(
                f'Loaded {len(self.cylinders)} cylinder obstacles from '
                f'{sdf_path or "(none)"}'
            )

            # ── Static TF Broadcaster (world -> map, world -> odom) ───────
            self.tf_static_broadcaster = StaticTransformBroadcaster(self)
            self._publish_static_transforms()

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
            self.create_subscription(
                String, '/ekf/tracking_mode', self.tracking_mode_cb, 10,
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

        def _publish_static_transforms(self):
            now = self.get_clock().now().to_msg()
            t_map = TransformStamped()
            t_map.header.stamp = now
            t_map.header.frame_id = 'world'
            t_map.child_frame_id = 'map'
            t_map.transform.rotation.w = 1.0

            t_odom = TransformStamped()
            t_odom.header.stamp = now
            t_odom.header.frame_id = 'world'
            t_odom.child_frame_id = 'odom'
            t_odom.transform.rotation.w = 1.0

            self.tf_static_broadcaster.sendTransform([t_map, t_odom])

        def odom_cb(self, msg: Odometry):
            p = msg.pose.pose.position
            self.drone_pos = np.array([p.x, p.y, p.z])
            self.has_odom = True

        def target_cb(self, msg: Odometry):
            p = msg.pose.pose.position
            self.goal_pos = np.array([p.x, p.y, p.z])

        def phase_cb(self, msg: String):
            self.phase = msg.data

        def tracking_mode_cb(self, msg: String):
            self.tracking_mode = msg.data

        # ── Main loop ────────────────────────────────────────────────────

        def compute_and_publish(self):
            target_goal = None
            if self.goal_pos is not None:
                target_goal = np.copy(self.goal_pos)
                if self.has_odom and self.phase == 'FOLLOW':
                    target_goal = make_follow_goal(
                        self.drone_pos,
                        self.goal_pos,
                        self.follow_distance,
                        hold_altitude=self.hold_follow_altitude,
                    )

            # Check if APF can compute active flight commands
            can_compute_apf = (
                self.has_odom
                and self.goal_pos is not None
                and self.phase in ('FOLLOW', 'APPROACH')
                and self.tracking_mode in ('TRACKING', 'PREDICTING')
            )

            if not can_compute_apf:
                self._publish_zero_velocity()
                # Always publish scene markers (obstacles, drone, goal) even when idle
                self._publish_force_markers(result=None, target_goal=target_goal)
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
                self.drone_pos, target_goal, obstacle_points,
                k_rep_override=k_rep_override,
            )

            # Publish velocity command
            cmd = Twist()
            cmd.linear.x = float(result.velocity[0])
            cmd.linear.y = float(result.velocity[1])
            cmd.linear.z = float(result.velocity[2])
            self.vel_pub.publish(cmd)

            # Publish debug markers with arrows
            self._publish_force_markers(result, target_goal)

        def _publish_zero_velocity(self):
            self.vel_pub.publish(Twist())

        def _publish_force_markers(
            self,
            result: Optional[APFResult],
            target_goal: Optional[np.ndarray] = None,
        ):
            """Visualize attractive + repulsive forces, goal, drone, and obstacles in RViz2."""
            now = self.get_clock().now().to_msg()
            markers = MarkerArray()

            # Arrow forces: only if result is valid and we have odom
            if result is not None and self.has_odom:
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
                att.color.a = 0.85
                start = Point(
                    x=float(self.drone_pos[0]), y=float(self.drone_pos[1]), z=float(self.drone_pos[2]),
                )
                f_att_norm = np.linalg.norm(result.f_att)
                scale = min(2.0, f_att_norm / max(self.apf.params.k_att, 1.0))
                if f_att_norm > 1e-4:
                    direction = result.f_att / f_att_norm * scale
                else:
                    direction = np.zeros(3)
                end = Point(
                    x=float(self.drone_pos[0] + direction[0]),
                    y=float(self.drone_pos[1] + direction[1]),
                    z=float(self.drone_pos[2] + direction[2]),
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
                rep.color.a = 0.85
                f_rep_norm = np.linalg.norm(result.f_rep)
                scale = min(2.0, f_rep_norm / max(self.apf.params.k_rep * 0.0001, 1.0))
                if f_rep_norm > 1e-4:
                    direction = result.f_rep / f_rep_norm * scale
                else:
                    direction = np.zeros(3)
                rep.points = [
                    start,
                    Point(
                        x=float(self.drone_pos[0] + direction[0]),
                        y=float(self.drone_pos[1] + direction[1]),
                        z=float(self.drone_pos[2] + direction[2]),
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
                    Point(
                        x=float(self.drone_pos[0] + direction[0]),
                        y=float(self.drone_pos[1] + direction[1]),
                        z=float(self.drone_pos[2] + direction[2]),
                    ),
                ]
                markers.markers.append(total)

            # Marker: Drone Position (Cyan Sphere)
            if self.has_odom:
                drone_m = Marker()
                drone_m.header.stamp = now
                drone_m.header.frame_id = 'world'
                drone_m.ns = 'apf_drone'
                drone_m.id = 3
                drone_m.type = Marker.SPHERE
                drone_m.action = Marker.ADD
                drone_m.pose.position.x = float(self.drone_pos[0])
                drone_m.pose.position.y = float(self.drone_pos[1])
                drone_m.pose.position.z = float(self.drone_pos[2])
                drone_m.pose.orientation.w = 1.0
                drone_m.scale.x = 0.35
                drone_m.scale.y = 0.35
                drone_m.scale.z = 0.35
                drone_m.color.r = 0.1
                drone_m.color.g = 0.8
                drone_m.color.b = 0.9
                drone_m.color.a = 0.8
                markers.markers.append(drone_m)

            # Marker: Goal Position (Gold Sphere)
            if target_goal is not None:
                goal_m = Marker()
                goal_m.header.stamp = now
                goal_m.header.frame_id = 'world'
                goal_m.ns = 'apf_goal'
                goal_m.id = 4
                goal_m.type = Marker.SPHERE
                goal_m.action = Marker.ADD
                goal_m.pose.position.x = float(target_goal[0])
                goal_m.pose.position.y = float(target_goal[1])
                goal_m.pose.position.z = float(target_goal[2])
                goal_m.pose.orientation.w = 1.0
                goal_m.scale.x = 0.4
                goal_m.scale.y = 0.4
                goal_m.scale.z = 0.4
                goal_m.color.r = 1.0
                goal_m.color.g = 0.8
                goal_m.color.b = 0.0
                goal_m.color.a = 0.9
                markers.markers.append(goal_m)

            # Markers: Obstacle Cylinders (Orange Translucent Cylinders)
            for idx, (cx, cy, r, h, cz) in enumerate(self.cylinders):
                cyl_m = Marker()
                cyl_m.header.stamp = now
                cyl_m.header.frame_id = 'world'
                cyl_m.ns = 'apf_obstacles'
                cyl_m.id = 10 + idx
                cyl_m.type = Marker.CYLINDER
                cyl_m.action = Marker.ADD
                cyl_m.pose.position.x = float(cx)
                cyl_m.pose.position.y = float(cy)
                cyl_m.pose.position.z = float(cz)
                cyl_m.pose.orientation.w = 1.0
                cyl_m.scale.x = float(2 * r)
                cyl_m.scale.y = float(2 * r)
                cyl_m.scale.z = float(h)
                cyl_m.color.r = 0.95
                cyl_m.color.g = 0.45
                cyl_m.color.b = 0.15
                cyl_m.color.a = 0.55
                markers.markers.append(cyl_m)

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
