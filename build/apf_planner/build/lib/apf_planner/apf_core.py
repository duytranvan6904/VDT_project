from __future__ import annotations

import math
from dataclasses import dataclass
from typing import List, Optional, Tuple

import numpy as np


@dataclass
class APFParams:
    """Tunable APF parameters."""
    d0: float = 3.0               # Obstacle influence distance (m)
    v_max: float = 1.2            # Practical follow speed limit (m/s)
    d_slow: float = 1.5           # Deceleration distance to goal (m)
    k_att: float = 10.0           # Attractive gain
    k_rep: float = 2500.0         # Repulsive gain
    goal_threshold: float = 0.20  # Stop distance near goal (m)
    k_z: float = 0.6              # Altitude P gain (1/s)
    vz_max: float = 0.5           # Max vertical velocity (m/s)


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
    cruise_altitude: float = 3.0,
    is_3d_distance: bool = True,
) -> np.ndarray:
    """Return the horizontal stand-off point used during FOLLOW.

    APF attracts the drone to a stand-off point behind/towards the target,
    not to the target itself. The altitude is held at cruise_altitude during FOLLOW.

    When ``is_3d_distance`` is True (default), ``follow_distance`` represents the
    desired 3D Euclidean slant range from the drone to the physical ground marker.
    By the Pythagorean theorem, the horizontal stand-off distance is:
        R_xy = sqrt(max(0, follow_distance^2 - (z_drone - z_target)^2))
    """
    drone = np.asarray(drone_pos, dtype=float)
    target = np.asarray(target_pos, dtype=float)
    if drone.shape != (3,) or target.shape != (3,):
        raise ValueError('drone_pos and target_pos must be 3-element vectors')

    goal = target.copy()
    delta_xy = target[:2] - drone[:2]
    distance_xy = float(np.linalg.norm(delta_xy))

    if is_3d_distance:
        ref_z = cruise_altitude if hold_altitude else drone[2]
        dz = float(abs(ref_z - target[2]))
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
            direction_to_target = np.array([1.0, 0.0])
        goal[:2] = target[:2] - direction_to_target * standoff_xy
    else:
        goal[:2] = target[:2]

    if hold_altitude:
        goal[2] = cruise_altitude
    return goal


class APFCore:
    """Pure-Python APF planner with smooth deceleration, altitude regulation, and target-locked yaw."""

    def __init__(self, params: Optional[APFParams] = None) -> None:
        self.params = params or APFParams()

    def compute(
        self,
        pos: np.ndarray,
        goal: np.ndarray,
        obstacles: List[Tuple[float, float, float]],
        *,
        k_rep_override: Optional[float] = None,
        target_pos: Optional[np.ndarray] = None,
    ) -> APFResult:
        """Compute velocity command given drone position, goal, and obstacles.

        Parameters
        ----------
        pos : (3,) drone position [x, y, z] ENU
        goal : (3,) target/goal position [x, y, z] ENU
        obstacles : list of (x, y, z) obstacle surface positions ENU
        k_rep_override : if set, overrides self.params.k_rep (for APPROACH)
        target_pos : optional physical target position for camera heading lock

        Returns
        -------
        APFResult with velocity command, smooth deceleration, active altitude holding, and target-locked yaw
        """
        p = self.params
        k_rep = k_rep_override if k_rep_override is not None else p.k_rep

        pos = np.asarray(pos, dtype=float)
        goal = np.asarray(goal, dtype=float)

        # --- 2D Horizontal distance to goal ---
        d_goal_xy = goal[:2] - pos[:2]
        d_goal_mag = float(np.linalg.norm(d_goal_xy))

        # --- Horizontal Attractive force ---
        f_att_xy = 2.0 * p.k_att * d_goal_xy
        f_att = np.array([f_att_xy[0], f_att_xy[1], 0.0])

        # --- Horizontal Repulsive force (sum over all obstacles) ---
        f_rep = np.zeros(3)
        effective_d0 = min(p.d0, max(0.4, d_goal_mag))

        for obs in obstacles:
            obs_pos = np.asarray(obs, dtype=float)
            d_vec_xy = pos[:2] - obs_pos[:2]
            d = float(np.linalg.norm(d_vec_xy))

            if d < 1e-6:
                d = 1e-6                   # avoid division by zero
            if d > effective_d0:
                continue                   # outside influence zone

            grad_d = d_vec_xy / d
            rep_mag = 2.0 * k_rep * (1.0 / d - 1.0 / effective_d0) * (1.0 / (d * d))

            # Tangential component (horizontal 2D perpendicular)
            tan_dir = np.array([-grad_d[1], grad_d[0]])
            if np.dot(tan_dir, d_goal_xy) < 0:
                tan_dir = -tan_dir
            f_tan = 0.8 * rep_mag * tan_dir

            f_rep[:2] += rep_mag * grad_d + f_tan

        # --- Total horizontal force ---
        f_total_xy = f_att_xy + f_rep[:2]
        f_norm_xy = float(np.linalg.norm(f_total_xy))
        f_total = np.array([f_total_xy[0], f_total_xy[1], 0.0])

        # --- Horizontal Velocity with smooth deceleration ramp ---
        at_goal = d_goal_mag < p.goal_threshold

        if at_goal or f_norm_xy < 1e-4:
            vx, vy = 0.0, 0.0
        else:
            u_dir = f_total_xy / f_norm_xy
            # Deceleration scaling: ramps down linearly between d_slow and goal_threshold
            d_slow = max(p.d_slow, p.goal_threshold + 0.1)
            speed_scale = min(1.0, max(0.0, (d_goal_mag - p.goal_threshold) / (d_slow - p.goal_threshold)))
            speed = p.v_max * speed_scale
            vx = speed * u_dir[0]
            vy = speed * u_dir[1]

        # --- Vertical velocity (closed-loop altitude regulation with deadband) ---
        # A deadband of 8cm prevents altitude porpoising/hunting during bank turns
        dz = goal[2] - pos[2]
        deadband = 0.08
        if abs(dz) <= deadband or (at_goal and abs(dz) < 0.15):
            vz = 0.0
        else:
            eff_dz = dz - math.copysign(deadband, dz)
            vz = float(np.clip(p.k_z * eff_dz, -p.vz_max, p.vz_max))

        velocity = np.array([vx, vy, vz])

        # --- Yaw command: Lock heading to target if target_pos is given ---
        if target_pos is not None:
            d_target_xy = np.asarray(target_pos[:2], dtype=float) - pos[:2]
            if np.linalg.norm(d_target_xy) > 0.1:
                yaw_cmd = float(math.atan2(d_target_xy[1], d_target_xy[0]))
            else:
                yaw_cmd = 0.0
        else:
            d_goal = goal[:2] - pos[:2]
            if np.linalg.norm(d_goal) > 0.1:
                yaw_cmd = float(math.atan2(d_goal[1], d_goal[0]))
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

