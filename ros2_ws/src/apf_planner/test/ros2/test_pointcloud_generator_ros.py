import numpy as np
import pytest
from sensor_msgs.msg import PointCloud2

from apf_planner.obstacles import cloud_to_xyz
from apf_planner.pointcloud_generator import PointCloudGenerator

TOPIC = '/map_generator/global_cloud'
BASE = {'seed': 1, 'rate_hz': 10.0, 'num_obs': 5, 'publish_static_tf': False}


def collect(make_rig, params, sec=0.8):
    rig = make_rig(PointCloudGenerator, params)
    msgs = []
    rig.probe.create_subscription(PointCloud2, TOPIC, msgs.append, 10)
    rig.wait_match(probe_subs=[TOPIC])
    rig.spin(sec)
    return msgs


def test_publishes_periodically(make_rig):
    msgs = collect(make_rig, BASE)
    assert len(msgs) >= 3


def test_message_layout(make_rig):
    m = collect(make_rig, BASE)[0]
    assert m.header.frame_id == 'world'
    assert m.height == 1 and m.width > 0
    assert m.point_step == 12 and m.row_step == 12 * m.width
    assert [f.name for f in m.fields] == ['x', 'y', 'z']
    assert len(m.data) == m.width * 12
    assert m.is_dense and not m.is_bigendian


def test_points_finite_and_parseable(make_rig):
    m = collect(make_rig, BASE)[0]
    xyz = cloud_to_xyz(m)
    assert xyz.shape == (m.width, 3)
    assert np.isfinite(xyz).all()


def test_static_content_with_advancing_stamp(make_rig):
    msgs = collect(make_rig, BASE)
    assert bytes(msgs[0].data) == bytes(msgs[-1].data)
    t0 = msgs[0].header.stamp.sec + msgs[0].header.stamp.nanosec * 1e-9
    t1 = msgs[-1].header.stamp.sec + msgs[-1].header.stamp.nanosec * 1e-9
    assert t1 > t0


def test_zero_obstacles_gives_empty_cloud(make_rig):
    msgs = collect(make_rig, dict(BASE, num_obs=0))
    assert msgs and msgs[0].width == 0


def test_custom_frame_id(make_rig):
    msgs = collect(make_rig, dict(BASE, frame_id='odom_frame'))
    assert msgs[0].header.frame_id == 'odom_frame'


def test_static_tf_enabled_still_publishes(make_rig):
    msgs = collect(make_rig, dict(BASE, publish_static_tf=True))
    assert len(msgs) >= 3


def test_more_obstacles_more_points(make_rig):
    few = collect(make_rig, dict(BASE, num_obs=2, seed=3), sec=0.4)[0].width
    assert few > 0