#!/usr/bin/env python3
"""Integration tests for synthetic trajectory generation and EKF replay."""

import unittest

import numpy as np

from vision.target_state_ekf import TargetStateEKF
from vision.target_state_ct_ekf import CoordinatedTurnEKF
from vision.target_state_simulation import (
    MAX_SIMULATION_SPEED_MPS,
    MAX_OUTLIER_DISTANCE_M,
    SimulationConfig,
    calculate_metrics,
    generate_truth,
    generate_random_truth,
    iter_default_scenarios,
    replay_estimator,
    simulate_measurements,
)


class TestTargetStateSimulation(unittest.TestCase):
    def test_all_trajectories_have_finite_six_state_truth(self):
        for name in ("straight", "circle", "figure8", "zigzag", "stop_go"):
            timestamps, truth = generate_truth(name, duration_s=4.0, dt_s=1.0 / 30.0)
            self.assertEqual(truth.shape, (len(timestamps), 6))
            self.assertTrue(np.all(np.isfinite(truth)))
            self.assertLessEqual(np.max(np.linalg.norm(truth[:, 3:5], axis=1)), MAX_SIMULATION_SPEED_MPS)

    def test_dropout_is_nan_and_outlier_is_reproducible(self):
        config = SimulationConfig(
            duration_s=4.0,
            dropout_intervals_s=((1.0, 2.0),),
            outlier_probability=0.3,
            seed=123,
        )
        first = simulate_measurements("straight", config)
        second = simulate_measurements("straight", config)
        np.testing.assert_array_equal(first.detected, second.detected)
        np.testing.assert_array_equal(first.outlier, second.outlier)
        np.testing.assert_array_equal(np.isnan(first.measurements), np.isnan(second.measurements))
        self.assertTrue(np.all(np.isnan(first.measurements[~first.detected])))

    def test_outlier_distance_is_bounded_below_one_metre(self):
        data = simulate_measurements(
            "circle",
            SimulationConfig(duration_s=20.0, outlier_probability=1.0, outlier_std_m=10.0, seed=123),
        )
        offsets = data.measurements - data.truth_state[:, :3]
        distances = np.linalg.norm(offsets[data.outlier], axis=1)
        self.assertGreater(len(distances), 0)
        self.assertLessEqual(np.max(distances), MAX_OUTLIER_DISTANCE_M + 1e-12)
        self.assertLess(np.max(distances), 1.0)

    def test_random_trajectory_is_seeded_and_speed_limited(self):
        first = generate_random_truth("figure8", np.random.default_rng(10), duration_s=8.0)
        second = generate_random_truth("figure8", np.random.default_rng(10), duration_s=8.0)
        different = generate_random_truth("figure8", np.random.default_rng(11), duration_s=8.0)
        np.testing.assert_array_equal(first[1], second[1])
        self.assertFalse(np.array_equal(first[1], different[1]))
        self.assertLessEqual(np.max(np.linalg.norm(first[1][:, 3:5], axis=1)), MAX_SIMULATION_SPEED_MPS)

    def test_every_default_scenario_contains_a_one_second_dropout(self):
        scenarios = list(iter_default_scenarios(randomize=True, seed=123))
        self.assertEqual(len(scenarios), 5)
        for name, config in scenarios:
            self.assertTrue(config.dropout_intervals_s, msg=f"{name} has no dropout interval")
            self.assertAlmostEqual(config.dropout_intervals_s[0][1] - config.dropout_intervals_s[0][0], 1.0)

    def test_filter_improves_measurement_rmse_on_each_scenario(self):
        for name, seed in (("straight", 1), ("circle", 2), ("figure8", 3), ("zigzag", 4), ("stop_go", 5)):
            data = simulate_measurements(
                name,
                SimulationConfig(
                    duration_s=12.0,
                    dropout_intervals_s=((4.0, 5.0),),
                    outlier_probability=0.01,
                    seed=seed,
                ),
            )
            replay = replay_estimator(
                data,
                TargetStateEKF(process_accel_variance=(1.0, 1.0, 0.5)),
            )
            metrics = calculate_metrics(data, replay)
            self.assertLess(
                metrics["filtered_position_rmse_m"],
                metrics["raw_position_rmse_m"],
                msg=f"filter did not improve {name}: {metrics}",
            )
            self.assertGreater(metrics["max_error_during_dropout_m"], 0.0)

    def test_outliers_are_rejected_by_end_to_end_replay(self):
        data = simulate_measurements(
            "straight",
            SimulationConfig(duration_s=10.0, outlier_probability=0.08, outlier_std_m=4.0, seed=77),
        )
        replay = replay_estimator(data, TargetStateEKF(process_accel_variance=(0.5, 0.5, 0.5)))
        outlier_indices = data.outlier & data.detected
        self.assertGreater(np.count_nonzero(outlier_indices), 0)
        self.assertGreater(calculate_metrics(data, replay)["outlier_rejection_rate"], 0.5)

    def test_coordinated_turn_improves_circle_dropout_over_cv(self):
        data = simulate_measurements(
            "circle",
            SimulationConfig(duration_s=12.0, dropout_intervals_s=((4.0, 5.0),), seed=21),
        )
        cv = replay_estimator(data, TargetStateEKF(process_accel_variance=(1.0, 1.0, 0.5)))
        ct = replay_estimator(data, CoordinatedTurnEKF(acceleration_variance=(0.8, 0.8, 0.4)))
        cv_metrics = calculate_metrics(data, cv)
        ct_metrics = calculate_metrics(data, ct)
        self.assertLess(ct_metrics["max_error_during_dropout_m"], cv_metrics["max_error_during_dropout_m"])


if __name__ == "__main__":
    unittest.main()
