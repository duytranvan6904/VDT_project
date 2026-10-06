import unittest
import numpy as np
from simulation.control.covariance_gate import CovarianceGate, CovarianceStatusResult


class TestCovarianceGate(unittest.TestCase):
    def setUp(self):
        self.gate = CovarianceGate(
            pad_radius_m=0.25,
            confidence_sigma=2.0,
            max_measurement_age_s=0.5,
            max_horizontal_distance_m=5.0,
            rswitch_base_m=1.5,
            rswitch_alpha=1.0,
            sliding_beta=2.0,
            default_drone_eph_m=0.08,
            default_drone_epv_m=0.12,
        )

    def test_mathematical_eigenvalue_and_uncertainty_radius(self):
        """Verify 2D maximum eigenvalue and 2-sigma radius calculation."""
        # Known isotropic covariance: sigma_x = 0.05m, sigma_y = 0.05m
        sigma = 0.05
        P_target = np.zeros(36)
        P_target[0] = sigma ** 2
        P_target[7] = sigma ** 2
        P_target[14] = 0.1 ** 2

        P_drone = np.zeros(36)
        P_drone[0] = sigma ** 2
        P_drone[7] = sigma ** 2
        P_drone[14] = 0.1 ** 2

        drone_pos = np.array([5.0, 2.0, 1.0])
        target_pos = np.array([5.0, 2.0, 0.0])

        res = self.gate.evaluate(
            drone_pos=drone_pos,
            target_pos=target_pos,
            drone_cov_36=P_drone,
            target_cov_36=P_target,
            is_detected=True,
            measurement_age_s=0.05,
        )

        # Total variance in x and y: sigma^2 + sigma^2 = 2 * (0.05^2) = 0.005
        expected_lambda_max = 2.0 * (sigma ** 2)
        expected_r_unc = 2.0 * np.sqrt(expected_lambda_max)

        self.assertAlmostEqual(res.lambda_max_2d, expected_lambda_max, places=6)
        self.assertAlmostEqual(res.r_uncertainty_2sigma, expected_r_unc, places=4)
        self.assertTrue(res.safe_to_land)
        self.assertEqual(res.reject_reason, "OK")

    def test_rotated_correlated_covariance(self):
        """Test with non-diagonal cross-correlation terms."""
        # 2x2 covariance with correlation: [[0.008, 0.004], [0.004, 0.008]]
        # Eigenvalues are 0.012 and 0.004 -> lambda_max = 0.012
        P_target = np.zeros(36)
        P_target[0] = 0.004
        P_target[1] = 0.002
        P_target[6] = 0.002
        P_target[7] = 0.004

        P_drone = np.zeros(36)
        P_drone[0] = 0.004
        P_drone[1] = 0.002
        P_drone[6] = 0.002
        P_drone[7] = 0.004

        res = self.gate.evaluate(
            drone_pos=np.array([1.0, 1.0, 1.0]),
            target_pos=np.array([1.0, 1.0, 0.0]),
            drone_cov_36=P_drone,
            target_cov_36=P_target,
            is_detected=True,
            measurement_age_s=0.1,
        )

        expected_lambda = 0.012
        expected_r_unc = 2.0 * np.sqrt(expected_lambda)
        self.assertAlmostEqual(res.lambda_max_2d, expected_lambda, places=5)
        self.assertAlmostEqual(res.r_uncertainty_2sigma, expected_r_unc, places=4)

    def test_unsafe_large_covariance_gating(self):
        """When uncertainty radius exceeds pad radius, safe_to_land must be False."""
        # Large variance: sigma = 0.20m -> total var = 0.08 -> r_unc = 2*sqrt(0.08) = 0.565m > 0.25m
        sigma = 0.20
        P_large = np.zeros(36)
        P_large[0] = sigma ** 2
        P_large[7] = sigma ** 2

        res = self.gate.evaluate(
            drone_pos=np.array([5.0, 2.0, 2.0]),
            target_pos=np.array([5.0, 2.0, 0.0]),
            drone_cov_36=P_large,
            target_cov_36=P_large,
            is_detected=True,
            measurement_age_s=0.05,
        )

        self.assertFalse(res.safe_to_land)
        self.assertGreater(res.r_uncertainty_2sigma, self.gate.pad_radius)
        self.assertIn("Uncertainty radius", res.reject_reason)

    def test_unsafe_stale_measurement(self):
        """When measurement age exceeds timeout, safe_to_land must be False."""
        P_small = np.zeros(36)
        P_small[0] = 0.001
        P_small[7] = 0.001

        res = self.gate.evaluate(
            drone_pos=np.array([5.0, 2.0, 2.0]),
            target_pos=np.array([5.0, 2.0, 0.0]),
            drone_cov_36=P_small,
            target_cov_36=P_small,
            is_detected=True,
            measurement_age_s=1.2,  # > max_measurement_age 0.5s
        )

        self.assertFalse(res.safe_to_land)
        self.assertIn("Measurement age", res.reject_reason)

    def test_unsafe_lost_vision(self):
        """When target is not detected, safe_to_land must be False."""
        P_small = np.zeros(36)
        P_small[0] = 0.001
        P_small[7] = 0.001

        res = self.gate.evaluate(
            drone_pos=np.array([5.0, 2.0, 2.0]),
            target_pos=np.array([5.0, 2.0, 0.0]),
            drone_cov_36=P_small,
            target_cov_36=P_small,
            is_detected=False,
            measurement_age_s=0.1,
        )

        self.assertFalse(res.safe_to_land)
        self.assertIn("not actively detected", res.reject_reason)

    def test_unsafe_too_far_away(self):
        """When horizontal distance exceeds max approach distance, safe_to_land is False."""
        P_small = np.zeros(36)
        P_small[0] = 0.001
        P_small[7] = 0.001

        res = self.gate.evaluate(
            drone_pos=np.array([0.0, 0.0, 2.0]),
            target_pos=np.array([10.0, 0.0, 0.0]),  # 10m > 5m
            drone_cov_36=P_small,
            target_cov_36=P_small,
            is_detected=True,
            measurement_age_s=0.1,
        )

        self.assertFalse(res.safe_to_land)
        self.assertIn("Horizontal distance", res.reject_reason)

    def test_adaptive_rswitch_and_sliding_weight(self):
        """Verify adaptive phase transition distance and sliding weight scaling."""
        sigma = 0.05
        P_cov = np.zeros(36)
        P_cov[0] = sigma ** 2
        P_cov[7] = sigma ** 2

        res = self.gate.evaluate(
            drone_pos=np.array([5.0, 2.0, 1.0]),
            target_pos=np.array([5.0, 2.0, 0.0]),
            drone_cov_36=P_cov,
            target_cov_36=P_cov,
            is_detected=True,
            measurement_age_s=0.1,
        )

        # lambda_max = 2 * (0.05^2) = 0.005
        # rswitch = 1.5 * (1 + 1.0 * sqrt(0.005)) = 1.5 * (1 + 0.0707) = 1.606m
        expected_rswitch = 1.5 * (1.0 + 1.0 * np.sqrt(0.005))
        self.assertAlmostEqual(res.rswitch_adaptive, expected_rswitch, places=3)

        # sliding_weight = 1.0 / (1.0 + 2.0 * 0.005) = 1.0 / 1.01 = 0.9901
        expected_weight = 1.0 / (1.0 + 2.0 * 0.005)
        self.assertAlmostEqual(res.sliding_weight, expected_weight, places=4)


if __name__ == '__main__':
    unittest.main()
