import math

import numpy as np
import pytest
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Float64, String
from visualization_msgs.msg import MarkerArray

from apf_planner.planner_node import APFPlannerNode
from ros2_ws.ros_utils import make_cloud, make_odom

BASE = {'planner_type': 'apf', 'data_timeout_sec': 0.3}


class Bench:
    def __init__(self, rig, with_cloud=True):
        p = rig.probe
        self.rig = rig
        best_effort = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=5)
        self.odom = p.create_publisher(Odometry, '/odom', best_effort)
        self.target = p.create_publisher(Odometry, '/hpad/state_filtered', 10)
        self.phase = p.create_publisher(String, '/mission/phase', 10)
        self.mode = p.create_publisher(String, '/ekf/tracking_mode', 10)
        self.cloud = p.create_publisher(PointCloud2, '/map_generator/global_cloud', 10)
        self.vel, self.yaw, self.markers = [], [], []
        p.create_subscription(
            Twist, '/apf/velocity_cmd',
            lambda m: self.vel.append((m.linear.x, m.linear.y, m.linear.z)), 10)
        p.create_subscription(Float64, '/apf/yaw_cmd', lambda m: self.yaw.append(m.data), 10)
        p.create_subscription(MarkerArray, '/apf/force_markers', self.markers.append, 10)
        pubs = ['/odom', '/hpad/state_filtered', '/mission/phase', '/ekf/tracking_mode']
        if with_cloud:
            pubs.append('/map_generator/global_cloud')
        rig.wait_match(pubs, ['/apf/velocity_cmd', '/apf/force_markers'])

    def push(self, drone=(-8.0, -8.0, 3.0), target=(8.0, 8.0, 0.0), phase='FOLLOW',
             mode='TRACKING', tvel=(0.0, 0.0, 0.0), odom_frame='world', send_odom=True):
        if send_odom:
            self.odom.publish(make_odom(drone, frame=odom_frame))
        self.target.publish(make_odom(target, tvel))
        ph = String()
        ph.data = phase
        self.phase.publish(ph)
        md = String()
        md.data = mode
        self.mode.publish(md)

    def run(self, sec, **kw):
        self.vel.clear()
        self.yaw.clear()
        self.markers.clear()
        self.rig.drive(sec, lambda: self.push(**kw))

    def last(self):
        return np.array(self.vel[-1])

    def speed(self):
        return float(np.linalg.norm(self.last()))

    def publish_cloud(self, pts):
        self.cloud.publish(make_cloud(pts))
        self.rig.spin(0.3)


@pytest.fixture
def bench(make_rig):
    def factory(params=None, with_cloud=True):
        merged = dict(BASE)
        merged.update(params or {})
        return Bench(make_rig(APFPlannerNode, merged), with_cloud)
    return factory


def test_idle_phase_publishes_zero_and_no_yaw(bench):
    b = bench()
    b.run(0.8, phase='IDLE')
    assert len(b.vel) >= 15
    assert all(np.allclose(v, 0.0) for v in b.vel)
    assert not b.yaw


def test_follow_moves_toward_goal_and_locks_yaw(bench):
    b = bench()
    b.run(0.8)
    vx, vy, vz = b.last()
    assert vx > 0.3 and vy > 0.3
    assert abs(vz) < 1e-6
    assert b.speed() <= 1.2 + 1e-6
    assert b.yaw and b.yaw[-1] == pytest.approx(math.pi / 4, abs=1e-3)


def test_approach_descends_toward_target(bench):
    b = bench()
    b.run(0.8, phase='APPROACH')
    vx, vy, vz = b.last()
    assert vx > 0 and vy > 0 and vz < -0.05


def test_follow_holds_altitude(bench):
    b = bench()
    b.run(0.8, drone=(-8.0, -8.0, 1.0))
    assert 0.05 < b.last()[2] <= 0.5 + 1e-6


@pytest.mark.parametrize('mode', ['TRACKING', 'PREDICTING'])
def test_allowed_modes_produce_motion(bench, mode):
    b = bench()
    b.run(0.8, mode=mode)
    assert b.speed() > 0.1


@pytest.mark.parametrize('mode', ['EXPIRED', 'PREDICTING_DEGRADED', 'LOST', 'COASTING'])
def test_blocked_modes_produce_zero(bench, mode):
    b = bench()
    b.run(0.8, mode=mode)
    assert b.vel and b.speed() == 0.0
    assert not b.yaw


@pytest.mark.parametrize('phase', ['SEARCH', 'LAND', 'COMPLETE', 'IDLE'])
def test_other_phases_produce_zero(bench, phase):
    b = bench()
    b.run(0.8, phase=phase)
    assert b.vel and b.speed() == 0.0


def test_stale_odom_stops_motion(bench):
    b = bench()
    b.run(0.6)
    assert b.speed() > 0.1
    b.run(0.8, send_odom=False)
    assert b.speed() == 0.0


def test_wrong_frame_is_ignored(bench):
    b = bench()
    b.run(0.8, odom_frame='map')
    assert b.vel and all(np.allclose(v, 0.0) for v in b.vel)


def test_nan_odom_is_ignored(bench):
    b = bench()
    b.run(0.8, drone=(float('nan'), 0.0, 3.0))
    assert all(np.allclose(v, 0.0) for v in b.vel)


def test_inactive_markers_have_only_obstacle_points(bench):
    b = bench()
    b.run(0.6, phase='IDLE')
    namespaces = {m.ns for m in b.markers[-1].markers}
    assert 'apf_obs_points' in namespaces
    assert 'apf_total' not in namespaces


def test_active_markers_include_force_arrows(bench):
    b = bench()
    b.run(0.6)
    namespaces = {m.ns for m in b.markers[-1].markers}
    assert {'apf_att', 'apf_rep', 'apf_total', 'apf_drone', 'apf_goal'} <= namespaces


def test_pointcloud_obstacle_deflects_path(bench):
    b = bench()
    kw = dict(drone=(0.0, 0.0, 3.0), target=(10.0, 0.0, 0.0))
    b.run(0.6, **kw)
    assert abs(b.last()[1]) < 1e-6
    b.publish_cloud([[1.5, 0.0, 3.0]])
    b.run(0.8, **kw)
    assert b.last()[1] < -0.05


def test_sdf_cylinder_deflects_path(bench, tmp_path):
    sdf = tmp_path / 'w.sdf'
    sdf.write_text(
        '<sdf><world name="w"><model name="cyl_1"><pose>1.5 0 3 0 0 0</pose><link><collision>'
        '<geometry><cylinder><radius>0.3</radius><length>6</length></cylinder></geometry>'
        '</collision></link></model></world></sdf>')
    b = bench({'obstacle_source': 'sdf', 'world_sdf': str(sdf)}, with_cloud=False)
    b.run(0.8, drone=(0.0, 0.0, 3.0), target=(10.0, 0.0, 0.0))
    assert b.last()[1] < -0.05


def test_target_velocity_feedforward(bench):
    b = bench()
    kw = dict(drone=(0.0, 0.0, 3.0), target=(3.0, 0.0, 0.0))
    b.run(1.0, **kw)
    base_vy = b.last()[1]
    b.run(1.2, tvel=(0.0, 1.0, 0.0), **kw)
    assert abs(base_vy) < 0.05
    assert b.last()[1] > 0.3


def test_iapf_variant_moves_toward_goal(bench):
    b = bench({'planner_type': 'iapf'})
    b.run(0.8)
    vx, vy, vz = b.last()
    assert vx > 0.3 and vy > 0.3


def test_continuous_publish_rate_when_inactive(bench):
    b = bench()
    b.run(1.0, phase='IDLE')
    assert len(b.vel) >= 20


def test_phase_change_resets_iapf_state(bench):
    b = bench({'planner_type': 'iapf'})
    b.run(0.6)
    b.run(0.6, phase='APPROACH')
    assert b.speed() > 0.05