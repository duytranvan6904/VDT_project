import math
import random

import numpy as np
import pytest

from apf_planner.pointcloud_generator import generate_forest


def forest(seed=1, num_obs=20, map_size=25.0, height=4.0, resolution=0.15, clear_radius=2.0):
    return generate_forest(random.Random(seed), num_obs, map_size, height, resolution, clear_radius)


def test_dtype_and_shape():
    pts = forest()
    assert pts.dtype == np.float32
    assert pts.ndim == 2 and pts.shape[1] == 3
    assert len(pts) > 0


def test_same_seed_same_output():
    assert np.array_equal(forest(seed=7), forest(seed=7))


def test_different_seed_different_output():
    assert not np.array_equal(forest(seed=1), forest(seed=2))


def test_zero_obstacles_is_empty():
    pts = forest(num_obs=0)
    assert pts.shape == (0, 3) and pts.dtype == np.float32


def test_clear_radius_removes_everything_when_huge():
    assert len(forest(clear_radius=1000.0)) == 0


def test_points_within_bounds():
    pts = forest(num_obs=50, map_size=20.0, height=4.0)
    assert np.abs(pts[:, 0]).max() <= 10.0 + 0.8 + 1e-3
    assert np.abs(pts[:, 1]).max() <= 10.0 + 0.8 + 1e-3
    assert pts[:, 2].min() >= 0.0
    assert pts[:, 2].max() <= 4.0 + 1e-3


def test_clear_zone_has_no_obstacle_axes_near_origin():
    pts = forest(num_obs=100, clear_radius=3.0)
    radial = np.hypot(pts[:, 0], pts[:, 1])
    assert radial.min() >= 3.0 - 0.8 - 1e-3


def test_resolution_controls_density():
    coarse = forest(resolution=0.3)
    fine = forest(resolution=0.1)
    assert len(fine) > len(coarse)


def test_all_values_finite():
    assert np.isfinite(forest()).all()