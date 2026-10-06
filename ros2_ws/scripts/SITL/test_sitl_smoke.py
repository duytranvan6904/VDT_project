import time

import rclpy
from rclpy.qos import DurabilityPolicy, ReliabilityPolicy
from sitl_common import DISARMED, SEARCH, ros_param, wait_until

NODES = [
    'input_cache_node', 'fsm_node', 'gimbal_node', 'offboard_node',
    'px4_state_bridge', 'vision_interface_bridge', 'safety_monitor_node',
    'xrce_bridge_node', 'ekf_node', 'ibvs_controller', 'apf_planner',
    'planner_merge']

MIN_HZ = {
    'input_cache/snapshot': 15.0,
    'odom': 10.0,
    'fsm/state': 8.0,
    'mission/phase': 8.0,
    'planner/velocity_setpoint': 15.0,
    'alt_estimator/state': 15.0,
    'gimbal/target_angle_deg': 15.0,
    'ekf/tracking_mode': 15.0,
}

SKIP_TOPICS = ('/rosout', '/parameter_events', '/tf', '/tf_static', '/clock')


def test_nodes_present(probe):
    assert wait_until(lambda: set(NODES) <= set(probe.get_node_names()), 60), \
        f'thieu node: {sorted(set(NODES) - set(probe.get_node_names()))}'


def test_topic_rates(probe):
    c0 = dict(probe.cnt)
    t0 = time.monotonic()
    time.sleep(6.0)
    dt = time.monotonic() - t0
    low = {t: (probe.cnt[t] - c0.get(t, 0)) / dt for t in MIN_HZ
           if (probe.cnt[t] - c0.get(t, 0)) / dt < MIN_HZ[t]}
    assert not low, f'tan so qua thap: {low}'


def test_params_loaded(probe):
    assert float(ros_param('/input_cache_node', 'planner_timeout_sec')) == 1.0
    assert int(ros_param('/fsm_node', 'enter_follow_cycles')) == 10
    assert float(ros_param('/fsm_node', 'follow_lost_timeout')) == 2.5
    assert float(ros_param('/offboard_node', 'min_engage_altitude_m')) == 2.0
    assert int(ros_param('/safety_monitor_node', 'offboard_hold_timeout') .split('.')[0]) == 1


def test_tf_chain(probe):
    assert wait_until(
        lambda: probe.buf.can_transform('base_link', 'world', rclpy.time.Time()), 30)
    assert wait_until(
        lambda: probe.buf.can_transform('camera_optical_frame', 'world', rclpy.time.Time()), 30)


def test_tf_single_source(probe):
    names = {i.node_name for i in probe.get_publishers_info_by_topic('/tf')}
    assert 'px4_state_bridge' in names
    assert not names & {'ekf_node', 'odom_tf_node'}, f'TF bi trung nguon: {names}'


def test_qos_compatible(probe):
    bad = []
    for topic, _ in probe.get_topic_names_and_types():
        if topic.startswith(SKIP_TOPICS):
            continue
        pubs = [p for p in probe.get_publishers_info_by_topic(topic) if p.node_name != 'sitl_probe']
        subs = [s for s in probe.get_subscriptions_info_by_topic(topic) if s.node_name != 'sitl_probe']
        for p in pubs:
            for s in subs:
                rel = (s.qos_profile.reliability == ReliabilityPolicy.RELIABLE
                       and p.qos_profile.reliability == ReliabilityPolicy.BEST_EFFORT)
                dur = (s.qos_profile.durability == DurabilityPolicy.TRANSIENT_LOCAL
                       and p.qos_profile.durability == DurabilityPolicy.VOLATILE)
                if rel or dur:
                    bad.append((topic, p.node_name, s.node_name))
    assert not bad, f'QoS khong tuong thich: {bad}'


def test_ground_state(probe):
    assert wait_until(lambda: probe.snap is not None and probe.snap.valid, 30)
    assert probe.state == SEARCH
    assert probe.arming == DISARMED
    assert probe.snap.altitude < 0.5
    assert not probe.snap.marker_detected
    assert probe.killed in (None, False)
    assert wait_until(lambda: probe.offb is not None, 10)
    assert not probe.offb.offboard_active