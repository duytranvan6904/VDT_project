import sys
from pathlib import Path
import unittest
import numpy as np

# Ensure landing_guidance package is on sys.path
pkg_path = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(pkg_path))

from landing_guidance.covariance_logic import CovarianceGate, CovarianceStatusResult


class TestCovarianceLogic(unittest.TestCase):
    def setUp(self):
        self.gate = CovarianceGate(
            pad_radius_m=0.75,
            confidence_sigma=2.0,
            max_measurement_age_s=0.5,
            max_horizontal_distance_m=5.0,
            rswitch_base_m=1.5,
            rswitch_min_m=1.5,
            rswitch_max_m=4.0,
            sigma_ideal_m=0.05,
            sigma_bad_m=0.30,
            max_uncertainty_2sigma_m=0.90,
            funnel_slope=0.15,
            use_marker_relative=True,
        )

    def test_nominal_tracking_at_standoff_altitude(self):
        """At z = 2.2m standoff tracking, 2-sigma = 0.45m should pass cleanly with adaptive funnel."""
        drone_pos = np.array([1.0, 0.5, 2.2], dtype=np.float64)
        target_pos = np.array([0.0, 0.0, 0.0], dtype=np.float64)

        # Target cov with 2-sigma around 0.45m (sigma ~ 0.225 -> var ~ 0.05)
        P_t = np.zeros(36)
        P_t[0] = 0.04   # xx
        P_t[7] = 0.04   # yy
        P_t[14] = 0.02  # zz

        P_d = np.zeros(36)
        P_d[0] = 0.01
        P_d[7] = 0.01
        P_d[14] = 0.02

        res = self.gate.evaluate(
            drone_pos=drone_pos,
            target_pos=target_pos,
            drone_cov_36=P_d,
            target_cov_36=P_t,
            is_detected=True,
            measurement_age_s=0.05,
        )

        self.assertTrue(res.safe_to_land, f"Should be safe to land, got reason: {res.reject_reason}")
        self.assertLessEqual(res.r_uncertainty_2sigma, 0.90)
        self.assertEqual(res.allowed_uncertainty, 0.90)
        self.assertGreater(res.rswitch_adaptive, 0.0)
        self.assertGreater(res.sliding_weight, 0.0)

    def test_touchdown_ground_narrow_funnel(self):
        """At z = 0.05m ground level, funnel tightens to pad_radius (0.75m)."""
        drone_pos = np.array([0.05, 0.02, 0.05], dtype=np.float64)
        target_pos = np.array([0.0, 0.0, 0.0], dtype=np.float64)

        # Good fine marker detection: 2-sigma ~ 0.08m
        P_t = np.zeros(36)
        P_t[0] = 0.0016  # sigma=0.04 -> 2-sigma=0.08
        P_t[7] = 0.0016
        P_t[14] = 0.001

        P_d = np.zeros(36)
        P_d[0] = 0.01
        P_d[7] = 0.01
        P_d[14] = 0.01

        res = self.gate.evaluate(
            drone_pos=drone_pos,
            target_pos=target_pos,
            drone_cov_36=P_d,
            target_cov_36=P_t,
            is_detected=True,
            measurement_age_s=0.02,
        )

        self.assertTrue(res.safe_to_land)
        self.assertAlmostEqual(res.allowed_uncertainty, 0.75, delta=0.02)

    def test_reject_when_uncertainty_exceeds_ceiling(self):
        """When 2-sigma uncertainty exceeds 0.90m ceiling, gate must hold."""
        drone_pos = np.array([0.5, 0.5, 2.0], dtype=np.float64)
        target_pos = np.array([0.0, 0.0, 0.0], dtype=np.float64)

        # Huge uncertainty: var = 0.36 -> sigma = 0.6 -> 2-sigma = 1.2m > 0.90m
        P_t = np.zeros(36)
        P_t[0] = 0.36
        P_t[7] = 0.36
        P_t[14] = 0.10

        res = self.gate.evaluate(
            drone_pos=drone_pos,
            target_pos=target_pos,
            drone_cov_36=None,
            target_cov_36=P_t,
            is_detected=True,
            measurement_age_s=0.05,
        )

        self.assertFalse(res.safe_to_land)
        self.assertIn("glide limit", res.reject_reason)

    def test_reject_when_vision_lost(self):
        """When vision is lost (is_detected=False), gate must hold."""
        drone_pos = np.array([0.0, 0.0, 1.0], dtype=np.float64)
        target_pos = np.array([0.0, 0.0, 0.0], dtype=np.float64)

        res = self.gate.evaluate(
            drone_pos=drone_pos,
            target_pos=target_pos,
            drone_cov_36=None,
            target_cov_36=None,
            is_detected=False,
            measurement_age_s=0.0,
        )

        self.assertFalse(res.safe_to_land)
        self.assertIn("not actively detected", res.reject_reason)

    def test_reject_when_out_of_horizontal_range(self):
        """When horizontal distance exceeds max_horizontal_dist (5.0m), gate must hold."""
        drone_pos = np.array([6.0, 0.0, 2.0], dtype=np.float64)
        target_pos = np.array([0.0, 0.0, 0.0], dtype=np.float64)

        res = self.gate.evaluate(
            drone_pos=drone_pos,
            target_pos=target_pos,
            drone_cov_36=None,
            target_cov_36=None,
            is_detected=True,
            measurement_age_s=0.05,
        )

        self.assertFalse(res.safe_to_land)
        self.assertIn("Horizontal distance", res.reject_reason)


if __name__ == '__main__':
    unittest.main()
