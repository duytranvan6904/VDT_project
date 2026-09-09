#!/usr/bin/env python3
"""Tests for the coordinated-turn EKF used on curved target trajectories."""

import unittest

import numpy as np

from vision.target_state_ct_ekf import CoordinatedTurnEKF


class TestCoordinatedTurnEKF(unittest.TestCase):
    def test_zero_turn_rate_reduces_to_constant_velocity(self):
        state = np.array((0.0, 0.0, 1.0, 1.0, -0.5, 0.2, 0.0))
        predicted = CoordinatedTurnEKF.transition(state, 0.5)
        np.testing.assert_allclose(predicted[:3], (0.5, -0.25, 1.1), atol=1e-12)
        np.testing.assert_allclose(predicted[3:], state[3:], atol=1e-12)

    def test_constant_turn_preserves_horizontal_speed(self):
        state = np.array((0.0, 0.0, 1.0, 1.2, 0.0, 0.0, 0.6))
        predicted = CoordinatedTurnEKF.transition(state, 1.0)
        self.assertAlmostEqual(np.linalg.norm(predicted[3:5]), 1.2, places=10)
        self.assertGreater(predicted[1], 0.0)

    def test_measurement_update_and_covariance_are_valid(self):
        ekf = CoordinatedTurnEKF()
        ekf.initialize((0.0, 0.0, 1.0), 0.0, velocity=(1.0, 0.0, 0.0), turn_rate=0.6)
        ekf.predict(0.5)
        before = np.trace(ekf.covariance[:3, :3])
        accepted, nis = ekf.update((0.58, 0.09, 1.0), np.diag((0.05, 0.05, 0.05)) ** 2)
        self.assertTrue(accepted)
        self.assertLess(nis, ekf.gate_threshold)
        self.assertLess(np.trace(ekf.covariance[:3, :3]), before)
        self.assertTrue(np.all(np.linalg.eigvalsh(ekf.covariance) >= -1e-8))

    def test_out_of_order_timestamp_is_rejected(self):
        ekf = CoordinatedTurnEKF()
        ekf.initialize((0.0, 0.0, 1.0), 1.0)
        with self.assertRaises(ValueError):
            ekf.predict(0.5)


if __name__ == "__main__":
    unittest.main()
