import math

import pytest
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64, String

try:
    from vdt_msgs.msg import PlannerOutput
except ImportError:
    from offboard_manager.msg import PlannerOutput

from apf_planner.planner_merge_node import PlannerMergeNode

VEL = (1.0, 0.5, -0.2)


class MergeBench:
    def __init__(self, rig):
        p = rig.probe
        self.rig = rig
        self.vel_pub = p.create_publisher(Twist, '/apf/velocity_cmd', 10)
        self.apf_pub = p.create_publisher(Float64, '/apf/yaw_cmd', 10)
        self.ibvs_pub = p.create_publisher(Float64, '/ibvs/yaw_cmd', 10)
        self.phase_pub = p.create_publisher(String, '/mission/phase', 10)
        self.out = []
        p.create_subscription(PlannerOutput, '/planner/velocity_setpoint', self.out.append, 10)
        rig.wait_match(
            ['/apf/velocity_cmd', '/apf/yaw_cmd', '/ibvs/yaw_cmd', '/mission/phase'],
            ['/planner/velocity_setpoint'])

    def push(self, phase='FOLLOW', vel=VEL, send_vel=True, send_phase=True, apf=None, ibvs=None):
        if send_phase:
            ph = String()
            ph.data = phase
            self.phase_pub.publish(ph)
        if send_vel:
            t = Twist()
            t.linear.x, t.linear.y, t.linear.z = vel
            self.vel_pub.publish(t)
        if apf is not None:
            self.apf_pub.publish(Float64(data=apf))
        if ibvs is not None:
            self.ibvs_pub.publish(Float64(data=ibvs))

    def run(self, sec, **kw):
        self.out.clear()
        self.rig.drive(sec, lambda: self.push(**kw))

    def last(self):
        m = self.out[-1]
        return m.vx, m.vy, m.vz, m.yaw


@pytest.fixture
def bench(make_rig):
    def factory(params=None):
        return MergeBench(make_rig(PlannerMergeNode, params))
    return factory


def test_publishes_zero_continuously_without_any_input(bench):
    b = bench()
    b.rig.spin(1.0)
    assert len(b.out) >= 15
    vx, vy, vz, yaw = b.last()
    assert (vx, vy, vz) == (0.0, 0.0, 0.0) and math.isnan(yaw)


@pytest.mark.parametrize('phase', ['FOLLOW', 'APPROACH'])
def test_active_phase_forwards_velocity(bench, phase):
    b = bench()
    b.run(0.8, phase=phase)
    vx, vy, vz, _ = b.last()
    assert (vx, vy, vz) == pytest.approx(VEL)


@pytest.mark.parametrize('phase', ['IDLE', 'SEARCH', 'LAND', 'COMPLETE'])
def test_other_phases_publish_zero_and_nan_yaw(bench, phase):
    b = bench()
    b.run(0.8, phase=phase, apf=0.5, ibvs=0.3)
    vx, vy, vz, yaw = b.last()
    assert (vx, vy, vz) == (0.0, 0.0, 0.0) and math.isnan(yaw)


def test_yaw_prefers_ibvs_over_apf(bench):
    b = bench()
    b.run(0.8, apf=0.5, ibvs=0.3)
    assert b.last()[3] == pytest.approx(0.3)


def test_yaw_falls_back_to_apf(bench):
    b = bench()
    b.run(0.8, apf=0.5)
    assert b.last()[3] == pytest.approx(0.5)


def test_yaw_nan_ibvs_falls_back_to_apf(bench):
    b = bench()
    b.run(0.8, apf=0.5, ibvs=float('nan'))
    assert b.last()[3] == pytest.approx(0.5)


def test_yaw_nan_without_sources(bench):
    b = bench()
    b.run(0.8)
    assert math.isnan(b.last()[3])


def test_yaw_source_hold(bench):
    b = bench({'yaw_source': 'hold'})
    b.run(0.8, apf=0.5, ibvs=0.3)
    assert math.isnan(b.last()[3])


def test_yaw_source_ibvs_only(bench):
    b = bench({'yaw_source': 'ibvs'})
    b.run(0.8, apf=0.5)
    assert math.isnan(b.last()[3])
    b.run(0.8, apf=0.5, ibvs=0.3)
    assert b.last()[3] == pytest.approx(0.3)


def test_yaw_source_apf_only(bench):
    b = bench({'yaw_source': 'apf'})
    b.run(0.8, ibvs=0.3)
    assert math.isnan(b.last()[3])
    b.run(0.8, ibvs=0.3, apf=0.5)
    assert b.last()[3] == pytest.approx(0.5)


def test_unknown_yaw_source_falls_back_to_ibvs_apf(bench):
    b = bench({'yaw_source': 'weird'})
    b.run(0.8, apf=0.5, ibvs=0.3)
    assert b.last()[3] == pytest.approx(0.3)


def test_stale_yaw_is_dropped(bench):
    b = bench({'yaw_timeout_sec': 0.3})
    b.run(0.6, ibvs=0.3)
    assert b.last()[3] == pytest.approx(0.3)
    b.run(0.8)
    assert math.isnan(b.last()[3])


def test_non_finite_velocity_is_zeroed(bench):
    b = bench()
    b.run(0.8, vel=(float('nan'), 0.0, 0.0), apf=0.5)
    vx, vy, vz, yaw = b.last()
    assert (vx, vy, vz) == (0.0, 0.0, 0.0) and math.isnan(yaw)


def test_stale_phase_falls_back_to_idle(bench):
    b = bench({'phase_timeout_sec': 0.4})
    b.run(0.6)
    assert b.last()[:3] == pytest.approx(VEL)
    b.run(0.9, send_phase=False)
    assert b.last()[:3] == (0.0, 0.0, 0.0)


def test_stale_upstream_hover_then_silence(bench):
    b = bench({'upstream_timeout_sec': 0.3, 'stale_hover_sec': 0.5})
    b.run(0.6)
    assert b.last()[:3] == pytest.approx(VEL)
    b.out.clear()
    b.run(0.55, send_vel=False)
    assert b.out and b.last()[:3] == (0.0, 0.0, 0.0)
    b.run(0.4, send_vel=False)
    assert len(b.out) == 0


def test_upstream_recovery_resumes_forwarding(bench):
    b = bench({'upstream_timeout_sec': 0.3, 'stale_hover_sec': 0.5})
    b.run(0.4)
    b.run(1.2, send_vel=False)
    b.run(0.6)
    assert b.last()[:3] == pytest.approx(VEL)


def test_idle_keeps_publishing_when_upstream_missing(bench):
    b = bench()
    b.run(1.5, phase='IDLE', send_vel=False)
    assert len(b.out) >= 20


def test_publish_rate_about_20hz(bench):
    b = bench()
    b.run(2.0, phase='IDLE')
    assert 25 <= len(b.out) <= 55