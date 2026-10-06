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
from ekf_adapter.tracking_policy import TrackingMode, classify_tracking_mode

COV = np.eye(3) * 1e-4
S45 = math.sqrt(0.5)


def make_tracker(**config):
    estimator = TargetStateEKF()
    return estimator, TargetTracker(estimator, TrackerConfig(**config))


def started_tracker(**config):
    estimator, tracker = make_tracker(**config)
    tracker.process_measurement((0.0, 0.0, 3.0), COV, 0.0)
    return estimator, tracker


@pytest.mark.parametrize(
    "kwargs",
    [
        {"process_accel_variance": (1.0, 1.0)},
        {"process_accel_variance": (1.0, 0.0, 1.0)},
        {"gate_threshold": 0.0},
        {"v_max": 0.0},
        {"vz_max": -1.0},
    ],
)
def test_estimator_rejects_invalid_parameters(kwargs):
    with pytest.raises(ValueError):
        TargetStateEKF(**kwargs)


def test_initialize_sets_state_and_covariance():
    estimator = TargetStateEKF()
    assert not estimator.initialized
    assert estimator.timestamp is None
    estimator.initialize((1.0, 2.0, 3.0), 5.0)
    assert estimator.initialized
    assert estimator.timestamp == 5.0
    np.testing.assert_allclose(estimator.state, [1, 2, 3, 0, 0, 0])
    np.testing.assert_allclose(np.diag(estimator.covariance)[:3], 0.25)


def test_initialize_clamps_velocity():
    estimator = TargetStateEKF(v_max=2.5, vz_max=1.5)
    estimator.initialize((0, 0, 0), 0.0, velocity=(6.0, 8.0, 5.0))
    state = estimator.state
    assert math.hypot(state[3], state[4]) == pytest.approx(2.5)
    assert state[5] == pytest.approx(1.5)


@pytest.mark.parametrize(
    "kwargs",
    [
        {"position": (math.nan, 0, 0)},
        {"position": (0, 0)},
        {"position_variance": (0.0, 1.0, 1.0)},
        {"velocity": (math.inf, 0, 0)},
        {"timestamp": math.nan},
    ],
)
def test_initialize_rejects_invalid_input(kwargs):
    arguments = {"position": (0, 0, 0), "timestamp": 0.0}
    arguments.update(kwargs)
    with pytest.raises(ValueError):
        TargetStateEKF().initialize(**arguments)


def test_decay_velocity_scales_only_velocity():
    estimator = TargetStateEKF()
    estimator.initialize((1, 2, 3), 0.0, velocity=(1.0, 1.0, 1.0))
    estimator.decay_velocity(0.5)
    np.testing.assert_allclose(estimator.state, [1, 2, 3, 0.5, 0.5, 0.5])


@pytest.mark.parametrize("factor", [-0.1, 1.1, math.nan, math.inf])
def test_decay_velocity_rejects_invalid_factor(factor):
    estimator = TargetStateEKF()
    estimator.initialize((0, 0, 0), 0.0)
    with pytest.raises(ValueError):
        estimator.decay_velocity(factor)


@pytest.mark.parametrize("dt", [-0.1, math.nan, math.inf])
def test_transition_and_process_covariance_reject_invalid_dt(dt):
    estimator = TargetStateEKF()
    with pytest.raises(ValueError):
        estimator.transition_matrix(dt)
    with pytest.raises(ValueError):
        estimator.process_covariance(dt)


def test_transition_matrix_structure():
    f = TargetStateEKF.transition_matrix(0.5)
    np.testing.assert_allclose(f[:3, :3], np.eye(3))
    np.testing.assert_allclose(f[:3, 3:], np.eye(3) * 0.5)
    np.testing.assert_allclose(f[3:, 3:], np.eye(3))
    np.testing.assert_allclose(f[3:, :3], 0.0)


def test_process_covariance_is_symmetric_positive_semidefinite():
    q = TargetStateEKF(process_accel_variance=(1.0, 2.0, 0.5)).process_covariance(0.1)
    np.testing.assert_allclose(q, q.T)
    assert np.min(np.linalg.eigvalsh(q)) >= -1e-12
    assert q[3, 3] == pytest.approx(0.1)
    assert q[4, 4] == pytest.approx(0.2)


def test_predict_requires_initialization():
    with pytest.raises(RuntimeError):
        TargetStateEKF().predict(1.0)


def test_predict_rejects_out_of_order_timestamp():
    estimator = TargetStateEKF()
    estimator.initialize((0, 0, 0), 5.0)
    with pytest.raises(ValueError):
        estimator.predict(4.0)


def test_predict_moves_position_and_grows_covariance():
    estimator = TargetStateEKF()
    estimator.initialize((0, 0, 0), 0.0, velocity=(1.0, 0.0, 0.0))
    before = np.trace(estimator.covariance)
    snapshot = estimator.predict(2.0)
    assert snapshot.state[0] == pytest.approx(2.0)
    assert snapshot.timestamp == 2.0
    assert np.trace(estimator.covariance) > before
    np.testing.assert_allclose(estimator.covariance, estimator.covariance.T)


def test_predict_same_timestamp_keeps_state():
    estimator = TargetStateEKF()
    estimator.initialize((1, 2, 3), 1.0, velocity=(1, 0, 0))
    estimator.predict(1.0)
    np.testing.assert_allclose(estimator.state, [1, 2, 3, 1, 0, 0])


def test_update_accepts_consistent_measurement():
    estimator = TargetStateEKF()
    estimator.initialize((0, 0, 0), 0.0, position_variance=(0.01, 0.01, 0.01))
    before = np.trace(estimator.covariance)
    accepted, nis = estimator.update((0.1, 0.0, 0.0), np.eye(3) * 0.01)
    assert accepted
    assert nis == pytest.approx(0.5)
    assert 0.0 < estimator.state[0] < 0.1
    assert np.trace(estimator.covariance) < before


def test_update_rejects_outlier_and_keeps_state():
    estimator = TargetStateEKF()
    estimator.initialize((0, 0, 0), 0.0, position_variance=(0.01, 0.01, 0.01))
    before = estimator.state
    accepted, nis = estimator.update((5.0, 0.0, 0.0), np.eye(3) * 0.01)
    assert not accepted
    assert nis > estimator.gate_threshold
    np.testing.assert_allclose(estimator.state, before)


@pytest.mark.parametrize(
    "covariance",
    [np.eye(2), -np.eye(3), np.full((3, 3), math.nan), np.zeros((3, 3))],
)
def test_update_rejects_invalid_covariance(covariance):
    estimator = TargetStateEKF()
    estimator.initialize((0, 0, 0), 0.0)
    with pytest.raises(ValueError):
        estimator.update((0, 0, 0), covariance)


def test_update_requires_initialization():
    with pytest.raises(RuntimeError):
        TargetStateEKF().update((0, 0, 0), np.eye(3))


def test_covariance_stays_symmetric_positive_definite_over_many_steps():
    estimator = TargetStateEKF()
    estimator.initialize((0, 0, 3), 0.0)
    for index in range(1, 200):
        estimator.predict(index / 30.0)
        estimator.update((0.01 * index, 0.0, 3.0), np.eye(3) * 0.01)
    covariance = estimator.covariance
    np.testing.assert_allclose(covariance, covariance.T, atol=1e-12)
    assert np.min(np.linalg.eigvalsh(covariance)) > 0.0


def test_quaternion_identity():
    np.testing.assert_allclose(quaternion_to_matrix(0, 0, 0, 1), np.eye(3), atol=1e-12)


def test_quaternion_rotation_about_z():
    rotation = quaternion_to_matrix(0, 0, S45, S45)
    np.testing.assert_allclose(rotation @ [1, 0, 0], [0, 1, 0], atol=1e-12)


def test_quaternion_is_normalized_and_orthonormal():
    rotation = quaternion_to_matrix(0.0, 0.0, 2 * S45, 2 * S45)
    np.testing.assert_allclose(rotation @ rotation.T, np.eye(3), atol=1e-12)
    assert np.linalg.det(rotation) == pytest.approx(1.0)


def test_quaternion_zero_norm_raises():
    with pytest.raises(ValueError):
        quaternion_to_matrix(0, 0, 0, 0)


def test_transform_point_applies_rotation_then_translation():
    rotation = quaternion_to_matrix(0, 0, S45, S45)
    np.testing.assert_allclose(transform_point((1, 0, 0), rotation, (10, 20, 30)), [10, 21, 30], atol=1e-12)


def test_covariance_matches_noise_model_for_identity_rotation():
    noise = NoiseConfig(attitude_std_rad=0.0, vehicle_position_std_m=0.0)
    covariance = build_world_covariance((0.0, 0.0, 4.0), np.eye(3), noise)
    np.testing.assert_allclose(np.diag(covariance), [0.02**2, 0.02**2, 0.16**2])


def test_covariance_uses_floor_at_short_range():
    noise = NoiseConfig(attitude_std_rad=0.0, vehicle_position_std_m=0.0)
    covariance = build_world_covariance((0.0, 0.0, 0.5), np.eye(3), noise)
    np.testing.assert_allclose(np.diag(covariance), [0.02**2, 0.02**2, 0.02**2])


def test_covariance_is_symmetric_positive_definite():
    rotation = quaternion_to_matrix(0.1, 0.2, 0.3, 0.9)
    covariance = build_world_covariance((0.3, -0.2, 3.0), rotation, NoiseConfig())
    np.testing.assert_allclose(covariance, covariance.T)
    assert np.min(np.linalg.eigvalsh(covariance)) > 0.0


def test_covariance_grows_with_depth():
    small = build_world_covariance((0, 0, 1.0), np.eye(3), NoiseConfig())
    large = build_world_covariance((0, 0, 5.0), np.eye(3), NoiseConfig())
    assert np.trace(large) > np.trace(small)


def test_covariance_rotates_with_transform():
    noise = NoiseConfig(attitude_std_rad=0.0, vehicle_position_std_m=0.0)
    rotation = quaternion_to_matrix(S45, 0, 0, S45)
    covariance = build_world_covariance((0.0, 0.0, 4.0), rotation, noise)
    np.testing.assert_allclose(np.diag(covariance), [0.02**2, 0.16**2, 0.02**2], atol=1e-12)


@pytest.mark.parametrize(
    "kwargs",
    [{"reacquire_frames": 1}, {"decay_tau_s": 0.0}, {"gate_min_m": 3.0, "gate_max_m": 2.0}],
)
def test_tracker_config_validation(kwargs):
    with pytest.raises(ValueError):
        TrackerConfig(**kwargs)


@pytest.mark.parametrize(
    "phase,age,detected,expected",
    [
        ("FOLLOW", 0.10, True, "TRACKING"),
        ("FOLLOW", 0.25, True, "TRACKING"),
        ("FOLLOW", 0.30, True, "PREDICTING"),
        ("FOLLOW", 0.10, False, "PREDICTING"),
        ("FOLLOW", 1.00, False, "PREDICTING"),
        ("FOLLOW", 1.50, False, "PREDICTING_DEGRADED"),
        ("FOLLOW", 2.00, False, "PREDICTING_DEGRADED"),
        ("FOLLOW", 2.10, False, "EXPIRED"),
        ("APPROACH", 0.40, False, "PREDICTING"),
        ("APPROACH", 0.80, False, "PREDICTING_DEGRADED"),
        ("APPROACH", 1.10, False, "EXPIRED"),
        ("UNKNOWN", 1.50, False, "PREDICTING_DEGRADED"),
        ("UNKNOWN", 2.50, False, "EXPIRED"),
        ("FOLLOW", math.inf, False, "EXPIRED"),
    ],
)
def test_classify_tracking_mode(phase, age, detected, expected):
    assert classify_tracking_mode(detected, age, phase) is TrackingMode[expected]


def test_tracking_mode_names_match_state_cache_contract():
    assert {mode.value for mode in TrackingMode} == {
        "TRACKING",
        "PREDICTING",
        "PREDICTING_DEGRADED",
        "EXPIRED",
    }


def test_tracker_first_measurement_initializes():
    estimator, tracker = make_tracker()
    assert not tracker.initialized
    assert tracker.last_valid_time is None
    assert tracker.age(1.0) == math.inf
    assert not tracker.detected(1.0)
    assert tracker.snapshot_at(1.0) is None
    outcome = tracker.process_measurement((1.0, 2.0, 3.0), COV, 1.0)
    assert outcome is MeasurementOutcome.INITIALIZED
    assert tracker.initialized
    assert tracker.last_valid_time == 1.0
    np.testing.assert_allclose(estimator.state[:3], [1, 2, 3])


def test_tracker_age_and_detected():
    _, tracker = started_tracker()
    assert tracker.age(0.1) == pytest.approx(0.1)
    assert tracker.age(-1.0) == 0.0
    assert tracker.detected(0.25)
    assert not tracker.detected(0.26)


def test_tracker_accepts_nearby_measurement():
    _, tracker = started_tracker()
    outcome = tracker.process_measurement((0.01, 0.0, 3.0), COV, 0.05)
    assert outcome is MeasurementOutcome.ACCEPTED
    assert tracker.last_valid_time == 0.05


def test_tracker_velocity_converges_on_constant_velocity_target():
    estimator, tracker = make_tracker()
    for index in range(91):
        stamp = index / 30.0
        tracker.process_measurement((0.5 * stamp, 0.2 * stamp, 3.0), COV, stamp)
    state = estimator.state
    assert state[3] == pytest.approx(0.5, abs=0.05)
    assert state[4] == pytest.approx(0.2, abs=0.05)


def test_tracker_rejects_out_of_order_stamp():
    _, tracker = started_tracker()
    tracker.process_measurement((0.0, 0.0, 3.0), COV, 1.0)
    assert tracker.process_measurement((0.0, 0.0, 3.0), COV, 0.5) is MeasurementOutcome.REJECTED_ORDER
    assert tracker.last_valid_time == 1.0


def test_tracker_gate_is_tight_for_short_gap():
    _, tracker = started_tracker()
    outcome = tracker.process_measurement((1.5, 0.0, 3.0), COV, 0.1)
    assert outcome is MeasurementOutcome.REJECTED_GATE


def test_tracker_gate_widens_for_long_gap():
    _, tracker = started_tracker()
    outcome = tracker.process_measurement((1.5, 0.0, 3.0), COV, 1.0)
    assert outcome is MeasurementOutcome.ACCEPTED


def test_single_outlier_is_rejected_and_track_continues():
    estimator, tracker = started_tracker()
    assert tracker.process_measurement((5.0, 0.0, 3.0), COV, 0.1) is MeasurementOutcome.REJECTED_GATE
    assert abs(estimator.state[0]) < 0.5
    assert tracker.process_measurement((0.0, 0.0, 3.0), COV, 0.2) is MeasurementOutcome.ACCEPTED
    assert tracker.last_valid_time == 0.2


def test_reacquire_after_two_consistent_candidates_has_zero_velocity():
    estimator, tracker = started_tracker()
    assert tracker.process_measurement((6.0, 0.0, 3.0), COV, 0.1) is MeasurementOutcome.REJECTED_GATE
    assert tracker.process_measurement((6.02, 0.0, 3.0), COV, 0.2) is MeasurementOutcome.REACQUIRED
    state = estimator.state
    assert state[0] == pytest.approx(6.02)
    np.testing.assert_allclose(state[3:], 0.0)
    assert tracker.last_valid_time == 0.2


def test_reacquire_estimates_velocity_when_baseline_is_long_enough():
    estimator, tracker = started_tracker()
    tracker.process_measurement((6.0, 0.0, 3.0), COV, 0.1)
    assert tracker.process_measurement((6.2, 0.0, 3.0), COV, 0.5) is MeasurementOutcome.REACQUIRED
    assert estimator.state[3] == pytest.approx(0.5)


def test_reacquire_requires_configured_frame_count():
    _, tracker = started_tracker(reacquire_frames=3)
    assert tracker.process_measurement((6.0, 0.0, 3.0), COV, 0.1) is MeasurementOutcome.REJECTED_GATE
    assert tracker.process_measurement((6.0, 0.0, 3.0), COV, 0.2) is MeasurementOutcome.REJECTED_GATE
    assert tracker.process_measurement((6.0, 0.0, 3.0), COV, 0.3) is MeasurementOutcome.REACQUIRED


def test_inconsistent_candidates_do_not_reacquire():
    _, tracker = started_tracker()
    assert tracker.process_measurement((6.0, 0.0, 3.0), COV, 0.1) is MeasurementOutcome.REJECTED_GATE
    assert tracker.process_measurement((-6.0, 0.0, 3.0), COV, 0.2) is MeasurementOutcome.REJECTED_GATE


def test_candidate_expires_after_window():
    _, tracker = started_tracker()
    assert tracker.process_measurement((6.0, 0.0, 3.0), COV, 0.1) is MeasurementOutcome.REJECTED_GATE
    assert tracker.process_measurement((6.0, 0.0, 3.0), COV, 1.5) is MeasurementOutcome.REJECTED_GATE


def test_nis_rejection_does_not_reset_age():
    _, tracker = started_tracker()
    outcome = tracker.process_measurement((0.5, 0.0, 3.0), COV, 0.02)
    assert outcome is MeasurementOutcome.REJECTED_GATE
    assert tracker.last_valid_time == 0.0
    assert tracker.age(0.02) == pytest.approx(0.02)


def test_snapshot_at_does_not_modify_filter():
    estimator, tracker = started_tracker()
    estimator.initialize((0.0, 0.0, 3.0), 0.0, velocity=(1.0, 0.0, 0.0))
    before_state = estimator.state
    before_time = estimator.timestamp
    snapshot = tracker.snapshot_at(3.0)
    assert snapshot.timestamp == 3.0
    np.testing.assert_allclose(estimator.state, before_state)
    assert estimator.timestamp == before_time


def test_snapshot_without_decay_before_decay_window():
    estimator, tracker = started_tracker()
    estimator.initialize((0.0, 0.0, 3.0), 0.0, velocity=(1.0, 0.0, 0.0))
    snapshot = tracker.snapshot_at(0.5)
    assert snapshot.state[0] == pytest.approx(0.5)
    assert snapshot.state[3] == pytest.approx(1.0)


def test_snapshot_applies_exponential_velocity_decay():
    estimator, tracker = started_tracker()
    estimator.initialize((0.0, 0.0, 3.0), 0.0, velocity=(1.0, 0.0, 0.0))
    snapshot = tracker.snapshot_at(3.0)
    factor = math.exp(-2.0 / 1.3)
    assert snapshot.state[3] == pytest.approx(factor)
    assert snapshot.state[0] == pytest.approx(1.0 + 2.0 * factor)
    assert 0.0 < snapshot.state[3] < 1.0


def test_tracker_recovers_after_marker_loss():
    estimator, tracker = make_tracker()
    for index in range(31):
        stamp = index / 30.0
        tracker.process_measurement((0.5 * stamp, 0.0, 3.0), COV, stamp)
    for index in range(60, 91):
        stamp = index / 30.0
        outcome = tracker.process_measurement((0.5 * stamp, 0.0, 3.0), COV, stamp)
    assert outcome is MeasurementOutcome.ACCEPTED
    assert estimator.state[0] == pytest.approx(0.5 * 3.0, abs=0.1)