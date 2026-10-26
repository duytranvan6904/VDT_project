import math
import sys
from pathlib import Path
import unittest
import numpy as np

# Ensure ekf_adapter package is on sys.path
pkg_path = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(pkg_path))

from ekf_adapter.target_state_ekf import TargetStateEKF


class TestTargetStateEKFv24(unittest.TestCase):
    def setUp(self):
        self.ekf = TargetStateEKF(
            process_accel_variance=(1.0, 1.0, 0.5),
            gate_threshold=16.27,
            v_max=2.5,
            vz_max=1.5,
            max_dv_per_step=0.40,
        )
        self.ekf.initialize(
            position=(0.0, 0.0, 1.0),
            timestamp=0.0,
            velocity=(0.0, 0.0, 0.0),
            position_variance=(0.04, 0.04, 0.04),
            velocity_variance=(0.25, 0.25, 0.25),
        )

    def test_cwna_propagation(self):
        """Test continuous white noise acceleration prediction model."""
        dt = 0.1
        self.ekf.predict(dt)
        state = self.ekf.state
        cov = self.ekf.covariance

        # Position should remain at origin since initial vel was 0
        np.testing.assert_allclose(state[:3], [0.0, 0.0, 1.0], atol=1e-5)
        # Covariance must remain symmetric and positive definite
        np.testing.assert_allclose(cov, cov.T, atol=1e-7)
        eigenvalues = np.linalg.eigvalsh(cov)
        self.assertTrue(np.all(eigenvalues > 0.0))
        # Variance should have grown due to Q(dt)
        self.assertGreater(cov[0, 0], 0.04)
        self.assertGreater(cov[3, 3], 0.25)

    def test_mahalanobis_gating_rejects_huge_outlier(self):
        """Outlier with huge NIS must be rejected without altering state."""
        self.ekf.predict(0.1)
        state_before = self.ekf.state.copy()
        cov_before = self.ekf.covariance.copy()

        # Wild measurement 50 meters away
        wild_pos = (50.0, -40.0, 10.0)
        R = np.diag([0.04, 0.04, 0.04])
        accepted, nis = self.ekf.update(wild_pos, R)

        self.assertFalse(accepted)
        self.assertGreater(nis, 16.27)
        # State and covariance must remain strictly unchanged
        np.testing.assert_allclose(self.ekf.state, state_before)
        np.testing.assert_allclose(self.ekf.covariance, cov_before)

    def test_mahalanobis_gating_accepts_valid_measurement(self):
        """Consistent measurement within Chi-square gate must be fused."""
        self.ekf.predict(0.1)
        meas_pos = (0.05, -0.02, 1.01)
        R = np.diag([0.01, 0.01, 0.01])
        accepted, nis = self.ekf.update(meas_pos, R)

        self.assertTrue(accepted)
        self.assertLessEqual(nis, 16.27)
        # Position should move towards measurement
        self.assertGreater(self.ekf.state[0], 0.0)
        # Covariance must decrease after measurement update
        self.assertLess(self.ekf.covariance[0, 0], 0.05)

    def test_max_dv_per_step_clamping(self):
        """Sudden step within gate cannot cause velocity jump exceeding max_dv_per_step (0.40 m/s)."""
        self.ekf.predict(0.1)
        vel_before = self.ekf.state[3:].copy()

        # A jump of ~0.5m that passes gate (nis ~ 5-10) but would cause large delta_v
        # With high measurement confidence (R very small), standard Kalman gain creates large delta_v
        jump_pos = (0.45, 0.0, 1.0)
        R = np.diag([0.001, 0.001, 0.001])
        accepted, nis = self.ekf.update(jump_pos, R)

        if accepted:
            vel_after = self.ekf.state[3:]
            delta_v = vel_after - vel_before
            delta_v_xy = math.hypot(delta_v[0], delta_v[1])
            self.assertLessEqual(delta_v_xy, 0.40001, f"Velocity jump {delta_v_xy:.3f} exceeded 0.40 m/s!")
            self.assertLessEqual(abs(delta_v[2]), 0.40001)

    def test_overall_velocity_limit_clamp(self):
        """Total velocity cannot exceed physical vehicle limits (v_max=2.5, vz_max=1.5)."""
        self.ekf.initialize(
            position=(0.0, 0.0, 1.0),
            timestamp=0.0,
            velocity=(5.0, 5.0, 4.0),
        )
        state = self.ekf.state
        v_xy = math.hypot(state[3], state[4])
        self.assertAlmostEqual(v_xy, 2.5, places=5)
        self.assertAlmostEqual(state[5], 1.5, places=5)


if __name__ == '__main__':
    unittest.main()
