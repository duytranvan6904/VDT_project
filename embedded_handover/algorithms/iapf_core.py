#!/usr/bin/env python3
"""Improved Artificial Potential Field (IAPF) 3D Planner Core.

Direct Python port from MATLAB ``avoidance/IAPF_Planner_3D_MultiObs.m``.

Features:
  1. Attractive force: F_att = 2 * katt * (goal - pos)
  2. Improved repulsive force for GNRON (Goal Non-Reachable with Obstacle Nearby)
     with sigmoid-based goal-distance weighting (F_rep1 + F_rep2)
  3. Local-minimum detection with hysteresis (F_enter, F_exit)
  4. Dynamic 3D tangent escape on tangent plane
  5. Forward-looking tangent candidate scoring (goal direction, obstacle clearance, continuity)
  6. Directional weighting for oscillation suppression (Eq. 10-11)
  7. Speed scheduling: deceleration near goal and obstacles, with guaranteed escape speed
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import List, Optional, Tuple

import numpy as np


@dataclass
class IAPFParams:
    """Tunable parameters for Improved APF (I-APF)."""
    # Potential field gains
    k_att: float = 10.0            # Attractive gain
    k_rep: float = 250.0           # Repulsive gain
    d0: float = 2.5                # Obstacle influence distance (m)
    v_max: float = 1.5             # Maximum velocity (m/s)
    goal_tol: float = 0.25         # Distance threshold to consider goal reached (m)

    # Local minimum detection (hysteresis)
    f_enter: float = 0.10          # Force threshold to enter local-minimum tangent mode (N)
    f_exit: float = 0.30           # Force threshold to exit local-minimum mode (N)
    goal_min_dist: float = 0.50    # Min distance to goal to trigger local minimum detection (m)

    # Tangential escape force
    k_tan: float = 1.0             # Tangent force magnitude gain
    n_tangent: int = 12            # Number of candidate directions on 3D tangent plane
    n_pred: int = 3                # Forward prediction horizon steps
    look_ahead_ratio: float = 0.25 # look_ahead = max(look_ahead_ratio * d0, min_look_ahead)
    min_look_ahead: float = 0.30   # Minimum look-ahead distance (m)

    # Tangent scoring weights
    w_goal: float = 1.0            # Weight for candidate pointing towards goal
    w_clear: float = 1.0           # Weight for candidate clearance from obstacles
    w_prev: float = 0.60           # Weight for candidate continuity with previous tangent

    # Oscillation suppression
    theta_small: float = math.pi / 6.0   # 30 deg threshold
    theta_large: float = math.pi / 3.0   # 60 deg threshold
    m_prev_1: float = 0.50         # Small turn: prev weight
    m_new_1: float = 0.50          # Small turn: new weight
    m_prev_2: float = 0.70         # Medium turn: prev weight
    m_new_2: float = 0.30          # Medium turn: new weight
    m_prev_3: float = 0.90         # Large turn: prev weight
    m_new_3: float = 0.10          # Large turn: new weight

    # Speed scheduling
    min_goal_factor: float = 0.15  # Minimum speed ratio near goal
    min_obs_factor: float = 0.25   # Minimum speed ratio near obstacles
    v_escape_min_ratio: float = 0.35  # Min speed ratio when escaping local minima (0.35 * v_max)

    # Altitude / 3D behavior
    vz_max: float = 0.5            # Max vertical velocity (m/s)
    clamp_vz: bool = True          # Clamp vz to vz_max


@dataclass
class IAPFResult:
    """Output state of one IAPF calculation cycle."""
    velocity: np.ndarray           # [vx, vy, vz] in world coordinates (m/s)
    yaw_cmd: float                 # heading angle command (rad)
    f_att: np.ndarray              # attractive force vector
    f_rep: np.ndarray              # total repulsive force vector
    f_total: np.ndarray            # total force vector before/after tangent
    at_goal: bool                  # True if goal tolerance satisfied
    tangent_active: bool = False   # True if local-minimum escape is currently active
    f_tan: np.ndarray = field(default_factory=lambda: np.zeros(3))


class IAPFCore:
    """Pure-Python implementation of Improved APF with 3D Tangent and Oscillation Suppression.

    Replicates MATLAB ``IAPF_Planner_3D_MultiObs.m`` logic precisely, with internal state
    tracking across consecutive time steps.
    """

    def __init__(self, params: Optional[IAPFParams] = None) -> None:
        self.params = params or IAPFParams()
        self.reset()

    def reset(self) -> None:
        """Reset internal persistent state (e.g. on new mission or goal)."""
        self.tangent_active: bool = False
        self.tangent_prev: np.ndarray = np.zeros(3, dtype=float)
        self.previous_obs_idx: int = -1
        self.prev_move_dir: np.ndarray = np.zeros(3, dtype=float)
        self.prev_dir_valid: bool = False

    def compute(
        self,
        pos: np.ndarray,
        goal: np.ndarray,
        obstacles: List[Tuple[float, float, float]],
        *,
        k_rep_override: Optional[float] = None,
        target_pos: Optional[np.ndarray] = None,
        is_dynamic_follow: bool = False,
    ) -> IAPFResult:
        """Compute 3D velocity command and heading from current pose, goal, and obstacles.

        Parameters
        ----------
        pos : (3,) current drone position [x, y, z]
        goal : (3,) target/standoff goal position [x, y, z]
        obstacles : list of (x, y, z) obstacle surface or center positions
        k_rep_override : optional override for k_rep (e.g. in APPROACH phase)
        target_pos : optional physical target position for camera heading lock.
                     If None, yaw_cmd aligns with the velocity vector.
        is_dynamic_follow : if True, preserves full obstacle repulsion and follow speed
                            rather than suppressing repulsion near the standoff goal.

        Returns
        -------
        IAPFResult
        """
        p_cfg = self.params
        k_rep = k_rep_override if k_rep_override is not None else p_cfg.k_rep
        k_att = p_cfg.k_att
        d0 = p_cfg.d0
        v_max = p_cfg.v_max
        small_num = 1e-6

        p = np.asarray(pos, dtype=float).copy()
        g = np.asarray(goal, dtype=float).copy()

        # -------------------------------------------------------------
        # 1. Goal Vector & Check Goal Reached
        # -------------------------------------------------------------
        goal_vec = g - p
        d_goal = float(np.linalg.norm(goal_vec))

        if d_goal <= p_cfg.goal_tol:
            self.reset()
            return IAPFResult(
                velocity=np.zeros(3),
                yaw_cmd=0.0,
                f_att=np.zeros(3),
                f_rep=np.zeros(3),
                f_total=np.zeros(3),
                at_goal=True,
                tangent_active=False,
                f_tan=np.zeros(3),
            )

        dir_goal = goal_vec / max(d_goal, small_num)

        # -------------------------------------------------------------
        # 2. Attractive Force
        # -------------------------------------------------------------
        f_att = 2.0 * k_att * goal_vec

        # -------------------------------------------------------------
        # 3. Improved Repulsive Force for GNRON (F_rep1 + F_rep2)
        # -------------------------------------------------------------
        f_rep = np.zeros(3, dtype=float)
        min_obs_dist = 1e6
        nearest_idx = -1
        n_obs = len(obstacles)

        # Compute goal-distance sigmoid weighting once
        if is_dynamic_follow:
            # During dynamic target following, the goal is a moving standoff point.
            # Preserve full obstacle repulsion so evasion is never suppressed near standoff.
            weight_rep = 0.50
            sigmoid_deriv = 0.0
        else:
            exp_goal = math.exp(-min(d_goal, 50.0))
            weight_rep = 1.0 / (1.0 + exp_goal) - 0.5
            sigmoid_deriv = exp_goal / ((1.0 + exp_goal) ** 2)

        for i, obs in enumerate(obstacles):
            obs_pos = np.asarray(obs, dtype=float)
            obs_vec = p - obs_pos
            d_obs = float(np.linalg.norm(obs_vec))

            if d_obs < min_obs_dist:
                min_obs_dist = d_obs
                nearest_idx = i

            if d_obs <= d0:
                d_safe = max(d_obs, small_num)
                dir_away = obs_vec / d_safe

                # Frep1: Push away from obstacle, scaled by goal-distance weight
                coeff1 = 2.0 * k_rep * (1.0 / d_safe - 1.0 / d0) * (1.0 / (d_safe ** 2)) * weight_rep
                f_rep1 = coeff1 * dir_away

                # Frep2: Pull towards goal to eliminate GNRON stall
                coeff2 = k_rep * ((1.0 / d_safe - 1.0 / d0) ** 2) * sigmoid_deriv
                f_rep2 = coeff2 * dir_goal

                f_rep += (f_rep1 + f_rep2)

        # -------------------------------------------------------------
        # 4. Resultant Force & Obstacle Nearby Check
        # -------------------------------------------------------------
        f_total = f_att + f_rep
        f_total_norm = float(np.linalg.norm(f_total))
        obstacle_near = (n_obs > 0) and (min_obs_dist < d0)

        # -------------------------------------------------------------
        # 5. Local Minimum Detection with Hysteresis (Schmitt Trigger)
        # -------------------------------------------------------------
        f_att_norm = float(np.linalg.norm(f_att))
        is_collinear_trap = False
        if obstacle_near and nearest_idx >= 0 and f_att_norm > small_num:
            obs_near = np.asarray(obstacles[nearest_idx], dtype=float)
            vec_to_obs = obs_near - p
            dist_to_obs = float(np.linalg.norm(vec_to_obs))
            if dist_to_obs > small_num:
                dir_to_obs = vec_to_obs / dist_to_obs
                cos_obs_goal = float(np.dot(dir_goal, dir_to_obs))
                # Obstacle lies between drone and goal, and repulsion opposes attraction
                if cos_obs_goal > 0.65 and float(np.dot(f_att, f_rep)) < 0.0:
                    forward_progress = float(np.dot(f_total, dir_goal))
                    if forward_progress <= 0.40 * f_att_norm:
                        is_collinear_trap = True

        if not self.tangent_active:
            if (
                ((f_total_norm < p_cfg.f_enter) or is_collinear_trap)
                and (d_goal > p_cfg.goal_min_dist)
                and obstacle_near
            ):
                self.tangent_active = True
                # Reset previous tangent if obstacle switched
                if nearest_idx != self.previous_obs_idx:
                    self.tangent_prev = np.zeros(3)
                self.previous_obs_idx = nearest_idx
        else:
            obs_cleared = False
            if nearest_idx >= 0:
                obs_near = np.asarray(obstacles[nearest_idx], dtype=float)
                vec_to_obs = obs_near - p
                dist_to_obs = float(np.linalg.norm(vec_to_obs))
                if dist_to_obs > small_num:
                    dir_to_obs = vec_to_obs / dist_to_obs
                    if float(np.dot(dir_goal, dir_to_obs)) < 0.20:
                        obs_cleared = True

            if (
                ((f_total_norm > p_cfg.f_exit and not is_collinear_trap) or obs_cleared)
                or (not obstacle_near)
                or (d_goal <= p_cfg.goal_min_dist)
            ):
                self.tangent_active = False

        # -------------------------------------------------------------
        # 6. Dynamic Tangent Escape
        # -------------------------------------------------------------
        f_tan = np.zeros(3)
        if self.tangent_active and nearest_idx >= 0:
            obs_near = np.asarray(obstacles[nearest_idx], dtype=float)

            # Compute horizontal cylinder normal (obstacles are vertical cylinders)
            normal_vec_xy = np.array([p[0] - obs_near[0], p[1] - obs_near[1]])
            normal_norm_xy = float(np.linalg.norm(normal_vec_xy))

            if normal_norm_xy > small_num:
                nx = normal_vec_xy[0] / normal_norm_xy
                ny = normal_vec_xy[1] / normal_norm_xy
            else:
                dir_goal_xy_norm = float(np.linalg.norm(dir_goal[:2]))
                if dir_goal_xy_norm > small_num:
                    nx = -dir_goal[0] / dir_goal_xy_norm
                    ny = -dir_goal[1] / dir_goal_xy_norm
                else:
                    nx, ny = 1.0, 0.0

            # The two continuous orthogonal horizontal tangents (CCW and CW)
            t_ccw = np.array([-ny, nx, 0.0])
            t_cw = np.array([ny, -nx, 0.0])
            candidates = [t_ccw, t_cw]

            # Forward-looking candidate evaluation
            look_ahead = max(p_cfg.look_ahead_ratio * d0, p_cfg.min_look_ahead)
            tangent_prev_norm = float(np.linalg.norm(self.tangent_prev[:2]))

            best_score = -1e9
            best_tangent = t_ccw

            for t_cand in candidates:
                # Score 1: Goal direction alignment
                goal_score = float(np.dot(t_cand, dir_goal))

                # Score 2: Continuity with previous tangent direction (hysteresis)
                prev_score = 0.0
                if tangent_prev_norm > 1e-4:
                    prev_score = float(np.dot(t_cand, self.tangent_prev / tangent_prev_norm))

                # Score 3: Forward clearance along candidate direction
                min_pred_dist = 1e6
                for h in range(1, p_cfg.n_pred + 1):
                    pred_ratio = h / float(p_cfg.n_pred)
                    p_test = p + pred_ratio * look_ahead * t_cand

                    for j in range(n_obs):
                        cand_dist = float(np.linalg.norm(p_test[:2] - np.asarray(obstacles[j], dtype=float)[:2]))
                        if cand_dist < min_pred_dist:
                            min_pred_dist = cand_dist

                clear_score = min(2.0, min_pred_dist / max(d0, small_num))

                score = (
                    p_cfg.w_goal * goal_score
                    + p_cfg.w_clear * clear_score
                    + p_cfg.w_prev * prev_score
                )

                if score > best_score:
                    best_score = score
                    best_tangent = t_cand

            self.tangent_prev = best_tangent.copy()
            f_rep_norm = float(np.linalg.norm(f_rep))
            tan_mag = p_cfg.k_tan * max(0.5, min(f_att_norm, max(1.0, f_rep_norm)))
            f_tan = tan_mag * best_tangent
            f_cmd_raw = f_total + f_tan
        else:
            f_cmd_raw = f_total.copy()

        # -------------------------------------------------------------
        # 7. Raw Command Direction
        # -------------------------------------------------------------
        f_raw_norm = float(np.linalg.norm(f_cmd_raw))
        if f_raw_norm > small_num:
            w_new = f_cmd_raw / f_raw_norm
        else:
            w_new = dir_goal.copy()

        # -------------------------------------------------------------
        # 8. Directional Weighting for Oscillation Suppression
        # -------------------------------------------------------------
        move_dir = w_new.copy()
        if obstacle_near and self.prev_dir_valid:
            cos_alpha = float(np.clip(np.dot(self.prev_move_dir, w_new), -1.0, 1.0))
            delta_alpha = math.acos(cos_alpha)

            if delta_alpha <= p_cfg.theta_small:
                m_prev = p_cfg.m_prev_1
                m_new = p_cfg.m_new_1
            elif delta_alpha <= p_cfg.theta_large:
                m_prev = p_cfg.m_prev_2
                m_new = p_cfg.m_new_2
            else:
                m_prev = p_cfg.m_prev_3
                m_new = p_cfg.m_new_3

            w_weighted = m_prev * self.prev_move_dir + m_new * w_new
            w_weighted_norm = float(np.linalg.norm(w_weighted))
            if w_weighted_norm > small_num:
                move_dir = w_weighted / w_weighted_norm
            else:
                move_dir = w_new

        self.prev_move_dir = move_dir.copy()
        self.prev_dir_valid = True

        # -------------------------------------------------------------
        # 9. Speed Scheduling
        # -------------------------------------------------------------
        if is_dynamic_follow:
            # During dynamic target following, the goal is a moving standoff point
            # and target velocity feedforward provides forward travel speed.
            # Smoothly ramp guidance velocity to 0 to prevent bang-bang relay limit cycles.
            d_slow = max(1.5, p_cfg.goal_tol + 0.2)
            if d_goal <= p_cfg.goal_tol:
                goal_speed_factor = 0.0
            elif d_goal < d_slow:
                goal_speed_factor = float(
                    (d_goal - p_cfg.goal_tol) / (d_slow - p_cfg.goal_tol)
                )
            else:
                goal_speed_factor = 1.0
        else:
            # Static waypoint navigation: standard MATLAB IAPF formulation with min_goal_factor floor
            # ensuring vehicle decisively crosses goal_tol without stalling outside goal threshold.
            goal_speed_factor = float(np.clip(
                d_goal / max(d0, small_num),
                p_cfg.min_goal_factor,
                1.0,
            ))

        obs_speed_factor = 1.0
        if obstacle_near:
            obs_speed_factor = float(np.clip(
                min_obs_dist / max(d0, small_num),
                p_cfg.min_obs_factor,
                1.0,
            ))

        # Obstacle avoidance activity: if repulsive or tangent forces are active,
        # preserve necessary speed so obstacle evasion is never starved near the goal
        f_rep_tan_norm = float(np.linalg.norm(f_rep + f_tan))
        evasion_factor = (
            min(1.0, f_rep_tan_norm / max(f_att_norm, small_num))
            if obstacle_near else 0.0
        )
        effective_drive = max(goal_speed_factor, evasion_factor)
        v_cmd_mag = v_max * effective_drive * obs_speed_factor

        # Ensure minimum escape velocity when in local minimum
        if self.tangent_active:
            v_escape_min = p_cfg.v_escape_min_ratio * v_max
            if v_cmd_mag < v_escape_min:
                v_cmd_mag = v_escape_min

        velocity = v_cmd_mag * move_dir

        if p_cfg.clamp_vz:
            velocity[2] = float(np.clip(velocity[2], -p_cfg.vz_max, p_cfg.vz_max))

        # -------------------------------------------------------------
        # 10. Yaw Command
        # -------------------------------------------------------------
        # If target_pos is provided (e.g. for camera tracking), lock heading to target
        if target_pos is not None:
            d_target_xy = np.asarray(target_pos[:2], dtype=float) - p[:2]
            if float(np.linalg.norm(d_target_xy)) > 0.1:
                yaw_cmd = float(math.atan2(d_target_xy[1], d_target_xy[0]))
            else:
                yaw_cmd = 0.0
        else:
            horizontal_speed = math.hypot(velocity[0], velocity[1])
            if horizontal_speed > small_num:
                yaw_cmd = float(math.atan2(velocity[1], velocity[0]))
            else:
                yaw_cmd = 0.0

        return IAPFResult(
            velocity=velocity,
            yaw_cmd=yaw_cmd,
            f_att=f_att,
            f_rep=f_rep,
            f_total=f_cmd_raw,
            at_goal=False,
            tangent_active=self.tangent_active,
            f_tan=f_tan,
        )
