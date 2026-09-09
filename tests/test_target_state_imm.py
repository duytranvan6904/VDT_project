#!/usr/bin/env python3
import unittest

import numpy as np

from vision.target_state_imm import TargetStateIMM


class TestTargetStateIMM(unittest.TestCase):
    def test_common_interface_and_probabilities(self):
        imm = TargetStateIMM()
        imm.initialize((0.0, 0.0, 1.0), 0.0, velocity=(1.0, 0.0, 0.0))
        imm.predict(0.1)
        accepted, nis = imm.update((0.1, 0.0, 1.0), np.diag((0.05, 0.05, 0.05)) ** 2)
        self.assertTrue(accepted)
        self.assertTrue(np.isfinite(nis))
        self.assertEqual(imm.state.shape, (6,))
        self.assertEqual(imm.covariance.shape, (6, 6))
        self.assertAlmostEqual(float(np.sum(imm.model_probabilities)), 1.0)
        self.assertTrue(np.all(np.linalg.eigvalsh(imm.covariance) >= -1e-8))

    def test_outlier_does_not_make_state_nonfinite(self):
        imm = TargetStateIMM()
        imm.initialize((0.0, 0.0, 1.0), 0.0)
        imm.predict(0.1)
        imm.update((100.0, -100.0, 100.0), np.diag((0.05, 0.05, 0.05)) ** 2)
        self.assertTrue(np.all(np.isfinite(imm.state)))
        self.assertTrue(np.all(np.isfinite(imm.covariance)))


if __name__ == "__main__":
    unittest.main()
