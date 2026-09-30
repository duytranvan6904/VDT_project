#!/usr/bin/env python3
"""Unit tests for the hardware-independent target-state estimator."""

import unittest

import numpy as np

from vision.target_state_ekf import TargetStateEKF


class TestTargetStateEKF(unittest.TestCase):
    def setUp(self):
        self.ekf = TargetStateEKF(process_accel_variance=(0.5, 0.5, 0.5))
        self.ekf.initialize(
            position=(0.0, 0.0, 1.0),
            timestamp=0.0,
            velocity=(1.0, -0.5, 0.25),
            position_variance=(0.04, 0.04, 0.04),
            velocity_variance=(1.0, 1.0, 1.0),
        )

    def test_transition_and_process_covariance_are_cv_models(self):
        f = self.ekf.transition_matrix(0.2)
        np.testing.assert_allclose(f[:3, 3:], np.eye(3) * 0.2)
        q = self.ekf.process_covariance(0.2)
        self.assertTrue(np.allclose(q, q.T))
        self.assertTrue(np.all(np.linalg.eigvalsh(q) >= -1e-12))
        self.assertGreater(np.trace(self.ekf.process_covariance(0.4)), np.trace(q))

    def test_predict_constant_velocity(self):
        snapshot = self.ekf.predict(2.0)
        np.testing.assert_allclose(snapshot.state[:3], (2.0, -1.0, 1.5), atol=1e-12)
        np.testing.assert_allclose(snapshot.state[3:], (1.0, -0.5, 0.25), atol=1e-12)

    def test_update_reduces_position_uncertainty(self):
        self.ekf.predict(1.0)
        before = np.trace(self.ekf.covariance[:3, :3])
        accepted, nis = self.ekf.update((0.05, -0.03, 1.02), np.diag((0.02, 0.02, 0.02)) ** 2)
        self.assertTrue(accepted)
        self.assertLess(nis, self.ekf.gate_threshold)
        self.assertLess(np.trace(self.ekf.covariance[:3, :3]), before)

    def test_outlier_is_gated_without_changing_state(self):
        self.ekf.predict(1.0)
        state_before = self.ekf.state
        covariance_before = self.ekf.covariance
        accepted, nis = self.ekf.update((20.0, -20.0, 20.0), np.diag((0.05, 0.05, 0.05)) ** 2)
        self.assertFalse(accepted)
        self.assertGreater(nis, self.ekf.gate_threshold)
        np.testing.assert_allclose(self.ekf.state, state_before)
        np.testing.assert_allclose(self.ekf.covariance, covariance_before)

    def test_prediction_covariance_grows_during_dropout(self):
        before = np.trace(self.ekf.covariance[:3, :3])
        self.ekf.predict(1.0)
        after = np.trace(self.ekf.covariance[:3, :3])
        self.assertGreater(after, before)

    def test_bad_timestamp_and_covariance_are_rejected(self):
        with self.assertRaises(ValueError):
            self.ekf.predict(-1.0)
        with self.assertRaises(ValueError):
            self.ekf.update((0.0, 0.0, 1.0), np.zeros((3, 3)))


if __name__ == "__main__":
    unittest.main()
