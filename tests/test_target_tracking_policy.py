#!/usr/bin/env python3
"""Tests for estimator/FSM timeout behavior."""

import unittest

from vision.target_tracking_policy import (
    PREDICTION_HORIZON_S,
    SEARCH_TIMEOUT_S,
    TrackingMode,
    classify_tracking_mode,
)


class TestTargetTrackingPolicy(unittest.TestCase):
    def test_prediction_horizon_is_one_second(self):
        self.assertEqual(PREDICTION_HORIZON_S, 1.0)
        self.assertEqual(classify_tracking_mode(False, 1.0, "FOLLOW"), TrackingMode.PREDICTING)
        self.assertEqual(classify_tracking_mode(False, 1.01, "FOLLOW"), TrackingMode.PREDICTING_DEGRADED)

    def test_phase_timeouts(self):
        self.assertEqual(SEARCH_TIMEOUT_S, {"APPROACH": 2.0, "FOLLOW": 3.0})
        self.assertEqual(classify_tracking_mode(False, 2.0, "APPROACH"), TrackingMode.PREDICTING_DEGRADED)
        self.assertEqual(classify_tracking_mode(False, 2.01, "APPROACH"), TrackingMode.EXPIRED)
        self.assertEqual(classify_tracking_mode(False, 3.01, "FOLLOW"), TrackingMode.EXPIRED)

    def test_new_measurement_always_restores_tracking(self):
        self.assertEqual(classify_tracking_mode(True, 100.0, "FOLLOW"), TrackingMode.TRACKING)


if __name__ == "__main__":
    unittest.main()
