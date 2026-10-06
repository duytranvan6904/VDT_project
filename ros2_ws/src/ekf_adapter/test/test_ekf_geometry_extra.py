import numpy as np
import pytest

from ekf_adapter.ekf_logic import (
    NoiseConfig,
    build_world_covariance,
    quaternion_to_matrix,
    transform_point,
)


def unit_quaternion(seed):
    q = np.random.default_rng(seed).normal(size=4)
    return q / np.linalg.norm(q)


@pytest.mark.parametrize("seed", range(10))
def test_rotation_is_orthonormal(seed):
    rotation = quaternion_to_matrix(*unit_quaternion(seed))
    assert np.allclose(rotation @ rotation.T, np.eye(3), atol=1e-9)
    assert np.linalg.det(rotation) == pytest.approx(1.0, abs=1e-9)


@pytest.mark.parametrize("seed", range(5))
def test_quaternion_sign_does_not_matter(seed):
    q = unit_quaternion(seed)
    assert np.allclose(quaternion_to_matrix(*q), quaternion_to_matrix(*(-q)), atol=1e-12)


@pytest.mark.parametrize("seed", range(5))
def test_transform_preserves_distance(seed):
    rng = np.random.default_rng(seed)
    rotation = quaternion_to_matrix(*unit_quaternion(seed))
    a, b, shift = rng.normal(size=3), rng.normal(size=3), rng.normal(size=3)
    moved = np.linalg.norm(transform_point(a, rotation, shift) - transform_point(b, rotation, shift))
    assert moved == pytest.approx(np.linalg.norm(a - b), abs=1e-9)


def test_transform_origin_returns_translation():
    rotation = quaternion_to_matrix(*unit_quaternion(3))
    assert np.allclose(transform_point([0, 0, 0], rotation, [1, 2, 3]), [1, 2, 3])


def test_depth_minus_lateral_matches_model():
    cfg = NoiseConfig()
    z = 5.0
    covariance = build_world_covariance([0.0, 0.0, z], np.eye(3), cfg)
    depth = max(cfg.depth_floor_m, cfg.depth_per_m2 * z**2) ** 2
    lateral = max(cfg.lateral_floor_m, cfg.lateral_per_m * z) ** 2
    assert covariance[2, 2] - covariance[0, 0] == pytest.approx(depth - lateral, rel=1e-6)


def test_floor_dominates_near_range():
    cfg = NoiseConfig()
    covariance = build_world_covariance([0.0, 0.0, 0.5], np.eye(3), cfg)
    assert covariance[0, 0] >= cfg.lateral_floor_m**2 + cfg.vehicle_position_std_m**2


def test_attitude_term_is_isotropic_and_scales_with_distance():
    low = NoiseConfig(attitude_std_rad=1e-6)
    high = NoiseConfig(attitude_std_rad=0.05)
    point = [0.0, 0.0, 8.0]
    delta = build_world_covariance(point, np.eye(3), high) - build_world_covariance(point, np.eye(3), low)
    assert np.allclose(delta, delta[0, 0] * np.eye(3), atol=1e-9)
    assert delta[0, 0] == pytest.approx((0.05 * 8.0) ** 2, rel=1e-4)


@pytest.mark.parametrize("seed", range(5))
def test_eigenvalues_are_rotation_invariant(seed):
    point = [0.4, -0.3, 4.0]
    plain = build_world_covariance(point, np.eye(3), NoiseConfig())
    rotated = build_world_covariance(
        point, quaternion_to_matrix(*unit_quaternion(seed)), NoiseConfig()
    )
    assert np.allclose(np.linalg.eigvalsh(plain), np.linalg.eigvalsh(rotated), atol=1e-12)


@pytest.mark.parametrize("seed", range(5))
def test_vehicle_position_floor(seed):
    cfg = NoiseConfig()
    rng = np.random.default_rng(seed)
    point = [rng.uniform(-2, 2), rng.uniform(-2, 2), rng.uniform(0.3, 10.0)]
    rotation = quaternion_to_matrix(*unit_quaternion(seed))
    covariance = build_world_covariance(point, rotation, cfg)
    assert np.min(np.linalg.eigvalsh(covariance)) >= cfg.vehicle_position_std_m**2 - 1e-12