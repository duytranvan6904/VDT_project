import math

import numpy as np
import pytest

from ekf_adapter.ekf_logic import (
    MeasurementOutcome,
    NoiseConfig,
    TargetTracker,
    TrackerConfig,
    build_world_covariance,
    quaternion_to_matrix,
    transform_point,
)
from ekf_adapter.target_state_ekf import TargetStateEKF
from ekf_adapter.target_state_simulation import SimulationConfig, simulate_measurements
from ekf_adapter.tracking_policy import TrackingMode, classify_tracking_mode

COVARIANCE = np.diag([0.10, 0.10, 0.08]) ** 2
DT = 1.0 / 30.0


def make_tracker(**overrides):
    return TargetTracker(TargetStateEKF(), TrackerConfig(**overrides))


def feed_constant_velocity(tracker, velocity, start=(0.0, 0.0, 1.5), steps=60, t0=1000.0):
    position = np.array(start, dtype=float)
    velocity = np.array(velocity, dtype=float)
    outcomes = []
    for index in range(steps):
        stamp = t0 + index * DT
        outcomes.append(tracker.process_measurement(position + velocity * index * DT, COVARIANCE, stamp))
    return outcomes, t0 + (steps - 1) * DT


class TestCore:
    def test_decay_velocity(self):
        estimator = TargetStateEKF()
        estimator.initialize([0, 0, 1], 0.0, velocity=[1.0, 2.0, 0.5])
        estimator.decay_velocity(0.5)
        assert np.allclose(estimator.state[3:], [0.5, 1.0, 0.25])

    @pytest.mark.parametrize("factor", [-0.1, 1.5, float("nan")])
    def test_decay_velocity_invalid(self, factor):
        estimator = TargetStateEKF()
        estimator.initialize([0, 0, 1], 0.0)
        with pytest.raises(ValueError):
            estimator.decay_velocity(factor)

    def test_vz_max_configurable(self):
        estimator = TargetStateEKF(vz_max=0.5)
        estimator.initialize([0, 0, 1], 0.0, velocity=[0.0, 0.0, 2.0])
        assert estimator.state[5] == pytest.approx(0.5)

    def test_vz_max_invalid(self):
        with pytest.raises(ValueError):
            TargetStateEKF(vz_max=0.0)


class TestGeometryHelpers:
    def test_identity_rotation(self):
        assert np.allclose(quaternion_to_matrix(0, 0, 0, 1), np.eye(3))

    def test_rotation_z_90(self):
        s = math.sin(math.pi / 4)
        rotation = quaternion_to_matrix(0, 0, s, math.cos(math.pi / 4))
        assert np.allclose(rotation @ [1, 0, 0], [0, 1, 0], atol=1e-9)

    def test_unnormalized_quaternion(self):
        assert np.allclose(quaternion_to_matrix(0, 0, 0, 2), np.eye(3))

    def test_zero_quaternion_rejected(self):
        with pytest.raises(ValueError):
            quaternion_to_matrix(0, 0, 0, 0)

    def test_transform_point(self):
        s = math.sin(math.pi / 4)
        rotation = quaternion_to_matrix(0, 0, s, math.cos(math.pi / 4))
        result = transform_point([1, 0, 0], rotation, [10, 20, 30])
        assert np.allclose(result, [10, 21, 30], atol=1e-9)


class TestWorldCovariance:
    def test_symmetric_positive_definite(self):
        rotation = quaternion_to_matrix(0.1, 0.2, 0.3, 0.9)
        covariance = build_world_covariance([0.3, -0.2, 2.5], rotation, NoiseConfig())
        assert np.allclose(covariance, covariance.T)
        assert np.min(np.linalg.eigvalsh(covariance)) > 0.0

    def test_grows_with_depth(self):
        near = build_world_covariance([0, 0, 1.0], np.eye(3), NoiseConfig())
        far = build_world_covariance([0, 0, 6.0], np.eye(3), NoiseConfig())
        assert far[2, 2] > near[2, 2] * 10.0

    def test_depth_axis_dominates_in_camera_frame(self):
        covariance = build_world_covariance([0, 0, 5.0], np.eye(3), NoiseConfig())
        assert covariance[2, 2] > covariance[0, 0]

    def test_rotation_moves_depth_uncertainty(self):
        s = math.sin(math.pi / 4)
        rotation = quaternion_to_matrix(s, 0, 0, math.cos(math.pi / 4))
        covariance = build_world_covariance([0, 0, 5.0], rotation, NoiseConfig())
        assert covariance[1, 1] > covariance[2, 2]


class TestTrackingPolicy:
    def test_tracking(self):
        assert classify_tracking_mode(True, 0.05, "FOLLOW") == TrackingMode.TRACKING

    def test_coasting(self):
        assert classify_tracking_mode(False, 0.8, "FOLLOW") == TrackingMode.COASTING

    def test_lost_follow(self):
        assert classify_tracking_mode(False, 2.5, "FOLLOW") == TrackingMode.LOST

    def test_phase_changes_limit(self):
        assert classify_tracking_mode(False, 1.5, "FOLLOW") == TrackingMode.COASTING
        assert classify_tracking_mode(False, 1.5, "APPROACH") == TrackingMode.LOST

    def test_unknown_phase_uses_follow(self):
        assert classify_tracking_mode(False, 1.5, "SOMETHING") == TrackingMode.COASTING

    def test_never_measured(self):
        assert classify_tracking_mode(False, math.inf, "FOLLOW") == TrackingMode.LOST


class TestTrackerConfig:
    def test_reacquire_frames_minimum(self):
        with pytest.raises(ValueError):
            TrackerConfig(reacquire_frames=1)

    def test_decay_tau_positive(self):
        with pytest.raises(ValueError):
            TrackerConfig(decay_tau_s=0.0)

    def test_gate_order(self):
        with pytest.raises(ValueError):
            TrackerConfig(gate_min_m=3.0, gate_max_m=1.0)


class TestTracker:
    def test_first_measurement_initializes(self):
        tracker = make_tracker()
        outcome = tracker.process_measurement([1, 2, 1.5], COVARIANCE, 100.0)
        assert outcome == MeasurementOutcome.INITIALIZED
        assert tracker.initialized
        assert tracker.last_valid_time == 100.0

    def test_age_before_init_is_infinite(self):
        tracker = make_tracker()
        assert math.isinf(tracker.age(10.0))
        assert not tracker.detected(10.0)
        assert tracker.snapshot_at(10.0) is None

    def test_constant_velocity_converges(self):
        tracker = make_tracker()
        outcomes, last = feed_constant_velocity(tracker, [0.8, -0.4, 0.0], steps=120)
        assert outcomes[0] == MeasurementOutcome.INITIALIZED
        assert all(item == MeasurementOutcome.ACCEPTED for item in outcomes[1:])
        state = tracker.snapshot_at(last).state
        assert state[3] == pytest.approx(0.8, abs=0.15)
        assert state[4] == pytest.approx(-0.4, abs=0.15)

    def test_out_of_order_rejected(self):
        tracker = make_tracker()
        tracker.process_measurement([0, 0, 1.5], COVARIANCE, 100.0)
        tracker.process_measurement([0, 0, 1.5], COVARIANCE, 100.1)
        outcome = tracker.process_measurement([0, 0, 1.5], COVARIANCE, 99.9)
        assert outcome == MeasurementOutcome.REJECTED_ORDER

    def test_single_outlier_rejected_and_track_kept(self):
        tracker = make_tracker()
        _, last = feed_constant_velocity(tracker, [0.0, 0.0, 0.0], steps=30)
        outcome = tracker.process_measurement([4.0, 4.0, 1.5], COVARIANCE, last + DT)
        assert outcome == MeasurementOutcome.REJECTED_GATE
        outcome = tracker.process_measurement([0.0, 0.0, 1.5], COVARIANCE, last + 2 * DT)
        assert outcome == MeasurementOutcome.ACCEPTED
        assert np.linalg.norm(tracker.snapshot_at(last + 2 * DT).state[:2]) < 0.2

    def test_two_consistent_far_frames_reacquire(self):
        tracker = make_tracker()
        _, last = feed_constant_velocity(tracker, [0.0, 0.0, 0.0], steps=30)
        first = tracker.process_measurement([5.0, 5.0, 1.5], COVARIANCE, last + DT)
        second = tracker.process_measurement([5.02, 5.0, 1.5], COVARIANCE, last + 2 * DT)
        assert first == MeasurementOutcome.REJECTED_GATE
        assert second == MeasurementOutcome.REACQUIRED
        state = tracker.snapshot_at(last + 2 * DT).state
        assert np.allclose(state[:2], [5.02, 5.0], atol=0.05)
        assert np.allclose(state[3:], 0.0)

    def test_reacquire_uses_velocity_with_long_baseline(self):
        tracker = make_tracker(reacquire_frames=2, candidate_window_s=2.0, candidate_step_min_m=1.0)
        _, last = feed_constant_velocity(tracker, [0.0, 0.0, 0.0], steps=30)
        tracker.process_measurement([5.0, 5.0, 1.5], COVARIANCE, last + 0.1)
        outcome = tracker.process_measurement([5.5, 5.0, 1.5], COVARIANCE, last + 0.5)
        assert outcome == MeasurementOutcome.REACQUIRED
        state = tracker.snapshot_at(last + 0.5).state
        assert state[3] == pytest.approx(0.5 / 0.4, abs=0.2)

    def test_inconsistent_far_frames_do_not_reacquire(self):
        tracker = make_tracker()
        _, last = feed_constant_velocity(tracker, [0.0, 0.0, 0.0], steps=30)
        first = tracker.process_measurement([5.0, 5.0, 1.5], COVARIANCE, last + DT)
        second = tracker.process_measurement([-5.0, 5.0, 1.5], COVARIANCE, last + 2 * DT)
        assert first == MeasurementOutcome.REJECTED_GATE
        assert second == MeasurementOutcome.REJECTED_GATE

    def test_candidate_expires_after_window(self):
        tracker = make_tracker()
        _, last = feed_constant_velocity(tracker, [0.0, 0.0, 0.0], steps=30)
        tracker.process_measurement([5.0, 5.0, 1.5], COVARIANCE, last + DT)
        outcome = tracker.process_measurement([5.0, 5.0, 1.5], COVARIANCE, last + 1.5)
        assert outcome == MeasurementOutcome.REJECTED_GATE

    def test_nis_reject_does_not_reset_age(self):
        tracker = make_tracker(gate_min_m=3.0, gate_max_m=3.0)
        _, last = feed_constant_velocity(tracker, [0.0, 0.0, 0.0], steps=60)
        valid_before = tracker.last_valid_time
        outcome = tracker.process_measurement([1.2, 0.0, 1.5], COVARIANCE, last + DT)
        assert outcome == MeasurementOutcome.REJECTED_GATE
        assert tracker.last_valid_time == valid_before

    def test_snapshot_does_not_mutate_filter(self):
        tracker = make_tracker()
        _, last = feed_constant_velocity(tracker, [0.5, 0.0, 0.0], steps=30)
        before = tracker.snapshot_at(last)
        tracker.snapshot_at(last + 5.0)
        after = tracker.snapshot_at(last)
        assert np.allclose(before.state, after.state)
        assert before.timestamp == after.timestamp

    def test_measurement_after_snapshot_is_not_out_of_order(self):
        tracker = make_tracker()
        _, last = feed_constant_velocity(tracker, [0.5, 0.0, 0.0], steps=30)
        tracker.snapshot_at(last + 0.5)
        outcome = tracker.process_measurement([0.5 * (last - 1000.0 + DT), 0.0, 1.5], COVARIANCE, last + DT)
        assert outcome == MeasurementOutcome.ACCEPTED

    def test_extrapolation_moves_with_velocity(self):
        tracker = make_tracker()
        _, last = feed_constant_velocity(tracker, [1.0, 0.0, 0.0], steps=90)
        now = tracker.snapshot_at(last).state[0]
        later = tracker.snapshot_at(last + 0.5).state[0]
        assert later - now == pytest.approx(0.5, abs=0.15)

    def test_velocity_decays_in_long_dropout(self):
        tracker = make_tracker()
        _, last = feed_constant_velocity(tracker, [1.0, 0.0, 0.0], steps=90)
        speed_before = tracker.snapshot_at(last).state[3]
        speed_late = tracker.snapshot_at(last + 6.0).state[3]
        assert abs(speed_late) < 0.1 * abs(speed_before)

    def test_no_decay_before_threshold(self):
        tracker = make_tracker()
        _, last = feed_constant_velocity(tracker, [1.0, 0.0, 0.0], steps=90)
        speed_before = tracker.snapshot_at(last).state[3]
        speed_soon = tracker.snapshot_at(last + 0.8).state[3]
        assert speed_soon == pytest.approx(speed_before, rel=1e-6)

    def test_covariance_grows_during_dropout(self):
        tracker = make_tracker()
        _, last = feed_constant_velocity(tracker, [0.5, 0.0, 0.0], steps=60)
        small = np.trace(tracker.snapshot_at(last).covariance)
        large = np.trace(tracker.snapshot_at(last + 2.0).covariance)
        assert large > small

    def test_detected_and_age(self):
        tracker = make_tracker()
        _, last = feed_constant_velocity(tracker, [0.0, 0.0, 0.0], steps=10)
        assert tracker.detected(last + 0.1)
        assert not tracker.detected(last + 1.0)
        assert tracker.age(last + 1.0) == pytest.approx(1.0)

    def test_track_recovers_after_dropout_with_moving_target(self):
        tracker = make_tracker()
        _, last = feed_constant_velocity(tracker, [0.8, 0.0, 0.0], steps=90)
        resume = last + 1.0
        true_x = 0.8 * (resume - 1000.0)
        outcome = tracker.process_measurement([true_x, 0.0, 1.5], COVARIANCE, resume)
        assert outcome == MeasurementOutcome.ACCEPTED


class TestSimulationReplay:
    @staticmethod
    def replay(trajectory, config):
        data = simulate_measurements(trajectory, config)
        tracker = TargetTracker(TargetStateEKF())
        base = 1000.0
        outcomes = []
        estimates = np.full((len(data.timestamps), 3), np.nan)
        for index, timestamp in enumerate(data.timestamps):
            stamp = base + float(timestamp)
            if data.detected[index]:
                outcomes.append(
                    tracker.process_measurement(data.measurements[index], COVARIANCE, stamp)
                )
            else:
                outcomes.append(None)
            snapshot = tracker.snapshot_at(stamp)
            if snapshot is not None:
                estimates[index] = snapshot.state[:3]
        return data, outcomes, estimates

    @pytest.mark.parametrize("trajectory", ["straight", "circle", "figure8", "zigzag", "stop_go"])
    def test_filter_beats_raw_measurements(self, trajectory):
        config = SimulationConfig(dropout_intervals_s=((6.0, 7.0),), outlier_probability=0.02, seed=3)
        data, _, estimates = self.replay(trajectory, config)
        truth = data.truth_state[:, :3]
        valid = ~np.isnan(estimates[:, 0]) & data.detected
        raw = np.sqrt(np.mean(np.sum((data.measurements[valid] - truth[valid]) ** 2, axis=1)))
        filtered = np.sqrt(np.mean(np.sum((estimates[valid] - truth[valid]) ** 2, axis=1)))
        assert filtered < raw

    def test_outliers_mostly_rejected(self):
        config = SimulationConfig(outlier_probability=0.05, seed=5, duration_s=30.0)
        data, outcomes, _ = self.replay("straight", config)
        accepted_outliers = sum(
            1
            for index in np.flatnonzero(data.outlier)
            if outcomes[index] in (MeasurementOutcome.ACCEPTED, MeasurementOutcome.INITIALIZED)
        )
        total_outliers = int(np.count_nonzero(data.outlier))
        assert total_outliers > 0
        assert accepted_outliers <= max(1, total_outliers // 2)

    def test_no_loss_of_track_after_dropout(self):
        config = SimulationConfig(dropout_intervals_s=((5.0, 6.5),), seed=9)
        data, outcomes, estimates = self.replay("straight", config)
        resume = int(np.searchsorted(data.timestamps, 6.5)) + 5
        error = np.linalg.norm(estimates[resume] - data.truth_state[resume, :3])
        assert error < 0.5
        assert outcomes[resume] == MeasurementOutcome.ACCEPTED
