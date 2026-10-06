from types import SimpleNamespace

import numpy as np
import pytest
from builtin_interfaces.msg import Time
from visualization_msgs.msg import Marker

from apf_planner.markers import build_markers

PARAMS = SimpleNamespace(k_att=10.0, k_rep=250.0)


def result(tangent=False, vel=(1.0, 0.0, 0.0)):
    return SimpleNamespace(
        f_att=np.array([200.0, 0.0, 0.0]),
        f_rep=np.array([-30.0, 5.0, 0.0]),
        velocity=np.array(vel),
        tangent_active=tangent,
        f_tan=np.array([0.0, 1.0, 0.0]),
    )


def by_ns(arr):
    return {(m.ns, m.id): m for m in arr.markers}


def length(m):
    a, b = m.points
    return float(np.linalg.norm([b.x - a.x, b.y - a.y, b.z - a.z]))


def test_minimal_array_has_only_obstacle_points():
    arr = build_markers(Time(), 'world', None, None, None, PARAMS, [], [])
    assert [(m.ns, m.id) for m in arr.markers] == [('apf_obs_points', 6)]
    assert arr.markers[0].type == Marker.SPHERE_LIST
    assert len(arr.markers[0].points) == 0


def test_drone_and_goal_spheres_without_result():
    arr = build_markers(Time(), 'world', np.array([1.0, 2.0, 3.0]), np.array([4.0, 5.0, 6.0]),
                        None, PARAMS, [], [])
    m = by_ns(arr)
    assert ('apf_drone', 3) in m and ('apf_goal', 4) in m
    assert not any(ns in ('apf_att', 'apf_rep', 'apf_total') for ns, _ in m)
    assert m[('apf_drone', 3)].pose.position.x == 1.0
    assert m[('apf_goal', 4)].pose.position.z == 6.0


def test_arrows_present_with_result():
    arr = build_markers(Time(), 'world', np.zeros(3), np.ones(3), result(), PARAMS, [], [])
    m = by_ns(arr)
    for key in (('apf_att', 0), ('apf_rep', 1), ('apf_total', 2)):
        assert m[key].type == Marker.ARROW
    assert ('apf_tangent', 5) not in m


def test_tangent_arrow_only_when_active():
    arr = build_markers(Time(), 'world', np.zeros(3), np.ones(3), result(True), PARAMS, [], [])
    assert ('apf_tangent', 5) in by_ns(arr)


def test_arrow_length_capped():
    arr = build_markers(Time(), 'world', np.zeros(3), np.ones(3), result(), PARAMS, [], [])
    m = by_ns(arr)
    assert length(m[('apf_att', 0)]) <= 2.0 + 1e-6
    assert length(m[('apf_total', 2)]) == pytest.approx(1.0)


def test_slow_velocity_gives_zero_length_arrow():
    arr = build_markers(Time(), 'world', np.zeros(3), np.ones(3), result(vel=(0.01, 0, 0)),
                        PARAMS, [], [])
    assert length(by_ns(arr)[('apf_total', 2)]) == pytest.approx(0.0)


def test_cylinders_and_points():
    cyls = [(1.0, 2.0, 0.5, 4.0, 2.0), (3.0, 4.0, 0.3, 2.0, 1.0)]
    pts = [(1.0, 1.0, 1.0), (2.0, 2.0, 2.0), (3.0, 3.0, 3.0)]
    arr = build_markers(Time(), 'world', np.zeros(3), np.ones(3), None, PARAMS, cyls, pts)
    m = by_ns(arr)
    c0 = m[('apf_obstacles', 10)]
    assert c0.type == Marker.CYLINDER
    assert (c0.scale.x, c0.scale.y, c0.scale.z) == (1.0, 1.0, 4.0)
    assert (c0.pose.position.x, c0.pose.position.y, c0.pose.position.z) == (1.0, 2.0, 2.0)
    assert ('apf_obstacles', 11) in m
    assert len(m[('apf_obs_points', 6)].points) == 3


def test_frame_and_stamp_propagate():
    stamp = Time(sec=5, nanosec=7)
    arr = build_markers(stamp, 'world', np.zeros(3), np.ones(3), result(True), PARAMS,
                        [(0.0, 0.0, 0.5, 1.0, 0.5)], [(1.0, 1.0, 1.0)])
    for mk in arr.markers:
        assert mk.header.frame_id == 'world'
        assert mk.header.stamp.sec == 5 and mk.header.stamp.nanosec == 7
        assert mk.action == Marker.ADD


def test_unique_namespace_id_pairs():
    arr = build_markers(Time(), 'world', np.zeros(3), np.ones(3), result(True), PARAMS,
                        [(0.0, 0.0, 0.5, 1.0, 0.5), (1.0, 1.0, 0.5, 1.0, 0.5)], [(1.0, 1.0, 1.0)])
    keys = [(m.ns, m.id) for m in arr.markers]
    assert len(keys) == len(set(keys))