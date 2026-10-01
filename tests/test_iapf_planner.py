#!/usr/bin/env python3
"""Comprehensive test suite for Improved APF (I-APF) Planner & A/B Comparison."""

import math
import unittest
import numpy as np

from simulation.control.apf_planner import APFCore, APFParams
from simulation.control.iapf_core import IAPFCore, IAPFParams, IAPFResult


class TestIAPFPlanner(unittest.TestCase):
    """Unit tests for IAPFCore algorithms and verification against standard APF."""

    def test_free_flight_towards_goal(self):
        """Without obstacles, drone should fly directly towards the goal."""
        planner = IAPFCore(IAPFParams(v_max=1.5, k_att=10.0))
        pos = np.array([0.0, 0.0, 3.0])
        goal = np.array([10.0, 0.0, 3.0])

        res = planner.compute(pos, goal, [])
        self.assertFalse(res.at_goal)
        self.assertFalse(res.tangent_active)
        # Velocity should be predominantly along +X
        self.assertGreater(res.velocity[0], 1.0)
        self.assertAlmostEqual(res.velocity[1], 0.0, places=5)
        self.assertAlmostEqual(res.velocity[2], 0.0, places=5)
        self.assertAlmostEqual(res.yaw_cmd, 0.0, places=4)

    def test_goal_reached_stopping(self):
        """When drone is within goal_tol, velocity and force must be zero."""
        planner = IAPFCore(IAPFParams(goal_tol=0.25))
        pos = np.array([5.0, 5.0, 3.0])
        goal = np.array([5.1, 5.0, 3.0])  # dist = 0.1m <= 0.25m

        res = planner.compute(pos, goal, [])
        self.assertTrue(res.at_goal)
        self.assertTrue(np.allclose(res.velocity, np.zeros(3)))
        self.assertFalse(res.tangent_active)

    def test_gnron_resolution(self):
        """Verify that GNRON formulation eliminates repulsive resistance towards goal.

        In standard APF, obstacles near the goal produce a strong repulsive force
        opposing the goal direction: dot(F_rep, dir_goal) < 0.
        In I-APF:
          - Frep1 (push away) is dampened by sigmoid weight_rep -> 0 as d_goal -> 0.
          - Frep2 (push towards goal) increases, ensuring net repulsive force supports
            or does not hinder reaching the goal: dot(F_rep, dir_goal) > 0.
        """
        planner = IAPFCore(IAPFParams(d0=3.0, k_rep=100.0, k_att=0.0, goal_tol=0.01))
        pos = np.array([0.0, 0.0, 3.0])
        obs = [(0.5, 0.0, 3.0)]  # obstacle in front at +0.5m
        dir_goal = np.array([1.0, 0.0, 0.0])

        # Far from goal: Frep1 dominates, net repulsive force pushes back (-X)
        res_far = planner.compute(pos, np.array([2.0, 0.0, 3.0]), obs)
        proj_far = float(np.dot(res_far.f_rep, dir_goal))
        self.assertLess(proj_far, 0.0)  # Pushes drone away from obstacle / opposite to goal

        # Close to goal: Frep2 dominates, net repulsive force actually pushes TOWARDS goal (+X)
        res_close = planner.compute(pos, np.array([0.05, 0.0, 3.0]), obs)
        proj_close = float(np.dot(res_close.f_rep, dir_goal))
        self.assertGreater(proj_close, 0.0)  # GNRON resolved: pushes drone towards goal!

    def test_local_minima_detection_and_tangent_escape(self):
        """Test symmetric head-on collision trap: Drone (1.5, 0), Obs (2.0, 0), Goal (4.0, 0).

        In this symmetric collinear configuration, attractive force [5, 0, 0] is cancelled
        by repulsive force [-5, 0, 0]. Standard APF gets locked or oscillates.
        I-APF detects |F_total| < f_enter, triggers tangent_active = True, and escapes laterally.
        """
        params = IAPFParams(
            k_att=1.0,
            k_rep=1.0138,  # Calibrated to balance attractive force at pos=[1.5, 0, 3]
            d0=2.0,
            f_enter=0.10,
            f_exit=0.30,
            goal_min_dist=0.50,
            k_tan=1.0,
        )
        planner = IAPFCore(params)

        pos = np.array([1.5, 0.0, 3.0])
        goal = np.array([4.0, 0.0, 3.0])
        obs = [(2.0, 0.0, 3.0)]

        res = planner.compute(pos, goal, obs)

        # Resultant force before tangent was almost 0, so tangent must activate!
        self.assertTrue(res.tangent_active)
        self.assertGreater(np.linalg.norm(res.f_tan), 0.1)
        # Velocity must have non-zero lateral component (orthogonal escape on tangent plane)
        lateral_speed = math.hypot(res.velocity[1], res.velocity[2])
        self.assertGreater(lateral_speed, 0.1)

    def test_hysteresis_behavior(self):
        """Check that tangent_active remains True when force norm is between f_enter and f_exit."""
        params = IAPFParams(
            k_att=1.0,
            k_rep=1.0138,
            d0=2.0,
            f_enter=0.10,
            f_exit=0.50,
            goal_min_dist=0.50,
        )
        planner = IAPFCore(params)

        # 1. Trigger local minimum: |F_att + F_rep| < 0.10
        pos1 = np.array([1.5, 0.0, 3.0])
        goal = np.array([4.0, 0.0, 3.0])
        obs = [(2.0, 0.0, 3.0)]
        res1 = planner.compute(pos1, goal, obs)
        self.assertTrue(res1.tangent_active)
        self.assertTrue(planner.tangent_active)

        # 2. Small step: force increases to ~0.068 N (< f_exit 0.50)
        pos2 = np.array([1.498, 0.0, 3.0])
        res2 = planner.compute(pos2, goal, obs)
        # Tangent must STAY active because force hasn't crossed f_exit (0.50)
        self.assertTrue(res2.tangent_active)
        self.assertTrue(planner.tangent_active)

        # 3. Move far away so obstacle is outside d0 -> Tangent must deactivate
        pos3 = np.array([1.3, 0.0, 3.0])
        res3 = planner.compute(pos3, goal, obs)
        # At pos 1.3, resultant force norm is ~3.5 N > f_exit (0.50) -> exits tangent mode
        self.assertFalse(res3.tangent_active)
        self.assertFalse(planner.tangent_active)

    def test_oscillation_suppression(self):
        """Verify that a sudden direction change is smoothed when obstacles are nearby."""
        params = IAPFParams(d0=3.0)
        planner = IAPFCore(params)

        pos = np.array([0.0, 0.0, 3.0])
        goal = np.array([5.0, 0.0, 3.0])
        obs = [(1.0, 0.0, 3.0)]

        # Step 1: Initial move forward along +X
        planner.prev_move_dir = np.array([1.0, 0.0, 0.0])
        planner.prev_dir_valid = True

        # Candidate new direction suddenly points at 90 degrees (+Y)
        w_new = np.array([0.0, 1.0, 0.0])
        cos_alpha = np.clip(np.dot(planner.prev_move_dir, w_new), -1.0, 1.0)
        delta_alpha = math.acos(cos_alpha)
        self.assertGreater(delta_alpha, params.theta_large)

        # Run computation with goal shifted to force a turn
        res = planner.compute(pos, np.array([0.0, 5.0, 3.0]), obs)
        # Direction should retain substantial forward component (+X) rather than snap purely to +Y
        self.assertGreater(planner.prev_move_dir[0], 0.1)

    def test_speed_scheduling_escape_minimum(self):
        """When tangent_active is True, speed should not drop below v_escape_min_ratio * v_max."""
        v_max = 2.0
        params = IAPFParams(
            k_att=1.0,
            k_rep=1.0138,
            d0=2.0,
            f_enter=0.10,
            f_exit=0.30,
            goal_min_dist=0.50,
            v_max=v_max,
            v_escape_min_ratio=0.35,
            min_obs_factor=0.01,
            clamp_vz=False,
        )
        planner = IAPFCore(params)

        pos = np.array([1.5, 0.0, 3.0])
        goal = np.array([4.0, 0.0, 3.0])
        obs = [(2.0, 0.0, 3.0)]

        res = planner.compute(pos, goal, obs)
        self.assertTrue(res.tangent_active)
        speed = float(np.linalg.norm(res.velocity))
        # Escaping minimum speed floor (0.35 * 2.0 = 0.70 m/s) must be maintained!
        self.assertAlmostEqual(speed, 0.35 * v_max, places=5)

    def test_ab_symmetric_trap_benchmark(self):
        """A/B Benchmark: Symmetric twin obstacle trap where standard APF stalls and I-APF succeeds.

        Obstacles at [3.0, 0.5, 3.0] and [3.0, -0.5, 3.0] create a symmetric equilibrium trap.
        Standard APF forces balance along X with zero lateral command -> stalls permanently.
        I-APF detects the local minimum, activates 3D tangent search, breaks symmetry, and reaches goal.
        """
        def run_trajectory(planner, max_steps=400, dt=0.05):
            pos = np.array([0.0, 0.0, 3.0])
            goal = np.array([6.0, 0.0, 3.0])
            obs = [(3.0, 0.5, 3.0), (3.0, -0.5, 3.0)]
            traj = [pos.copy()]
            for step in range(max_steps):
                res = planner.compute(pos, goal, obs)
                if res.at_goal or np.linalg.norm(goal - pos) < 0.25:
                    return True, pos
                pos = pos + res.velocity * dt
                traj.append(pos.copy())
                # If stalled (hardly moving over 30 cycles)
                if step > 30 and np.linalg.norm(traj[-1] - traj[-30]) < 0.02:
                    return False, pos
            return False, pos

        # Standard APF stalls at x ~= 1.8m
        std_planner = APFCore(APFParams(k_att=10.0, k_rep=250.0, d0=2.0, v_max=1.5))
        std_success, std_final = run_trajectory(std_planner)
        self.assertFalse(std_success, "Standard APF should be trapped in symmetric obstacles")
        self.assertLess(std_final[0], 2.5, "Standard APF is stuck before the obstacle at x=3.0")

        # Improved APF successfully escapes and reaches goal (x >= 5.75m)
        iapf_planner = IAPFCore(IAPFParams(
            k_att=10.0, k_rep=250.0, d0=2.0, v_max=1.5,
            f_enter=0.15, f_exit=0.35, k_tan=2.0,
        ))
        iapf_success, iapf_final = run_trajectory(iapf_planner)
        self.assertTrue(iapf_success, "I-APF must break symmetry and reach the goal")
        self.assertGreater(iapf_final[0], 5.5, "I-APF reached the goal region near x=6.0")


class TestAPFPlannerIntegration(unittest.TestCase):
    """Test ROS 2 node parameters and switching mechanism."""

    def test_apf_params_dataclass(self):
        apf_p = APFParams(d0=2.5, v_max=1.5)
        self.assertEqual(apf_p.d0, 2.5)
        self.assertEqual(apf_p.v_max, 1.5)

    def test_iapf_params_dataclass(self):
        iapf_p = IAPFParams(f_enter=0.15, n_tangent=16)
        self.assertEqual(iapf_p.f_enter, 0.15)
        self.assertEqual(iapf_p.n_tangent, 16)


if __name__ == '__main__':
    unittest.main()
