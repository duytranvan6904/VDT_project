#!/usr/bin/env python3
import argparse
import sys
import time


class Debouncer:
    def __init__(self, hold_s):
        self.hold_s = hold_s
        self.since = None
        self.out = False

    def update(self, raw, now):
        if not raw:
            self.since = None
            self.out = False
            return False
        if self.since is None:
            self.since = now
        if now - self.since >= self.hold_s:
            self.out = True
        return self.out


class AutoLand:
    def __init__(self, after_s, lock_s):
        self.after_s = after_s
        self.lock_s = lock_s
        self.follow_since = None
        self.lock_since = None
        self.latched = False

    def cancel(self, now):
        self.latched = False
        self.follow_since = now
        self.lock_since = None

    def update(self, now, phase, lock_ok):
        if self.after_s <= 0.0:
            return False
        if phase == 'IDLE':
            self.latched = False
            self.follow_since = None
            self.lock_since = None
            return False
        if self.latched:
            return True
        if phase != 'FOLLOW':
            self.follow_since = None
            self.lock_since = None
            return False
        if self.follow_since is None:
            self.follow_since = now
        if lock_ok:
            if self.lock_since is None:
                self.lock_since = now
        else:
            self.lock_since = None
        if (now - self.follow_since >= self.after_s
                and self.lock_since is not None
                and now - self.lock_since >= self.lock_s):
            self.latched = True
        return self.latched


def main():
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, qos_profile_sensor_data
    from rosidl_runtime_py.utilities import get_message
    from std_msgs.msg import Bool, String
    from vdt_msgs.msg import InputSnapshot

    ap = argparse.ArgumentParser()
    ap.add_argument('--output', default='/rc/fsm_input')
    ap.add_argument('--rc-input', default='/rc/fsm_input_rc')
    ap.add_argument('--operator-input', default='/virtual_rc/land')
    ap.add_argument('--cancel-input', default='/virtual_rc/cancel')
    ap.add_argument('--phase-topic', default='/mission/phase')
    ap.add_argument('--snap-topic', default='/input_cache/snapshot')
    ap.add_argument('--msg-type', default='')
    ap.add_argument('--hold', type=float, default=0.10)
    ap.add_argument('--rc-stale', type=float, default=0.3)
    ap.add_argument('--auto-after', type=float, default=0.0)
    ap.add_argument('--auto-lock', type=float, default=3.0)
    ap.add_argument('--rate', type=float, default=50.0)
    a = ap.parse_args()

    rclpy.init()
    node = Node('land_arbiter')

    msg_type = a.msg_type
    deadline = time.monotonic() + 10.0
    while not msg_type and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.2)
        for name, types in node.get_topic_names_and_types():
            if name == a.output and types:
                msg_type = types[0]
    if not msg_type:
        print(f'Could not detect the message type of {a.output}, use --msg-type')
        sys.exit(1)

    cls = get_message(msg_type)
    probe = cls()
    if not (hasattr(probe, 'land_switch') and hasattr(probe, 'kill_switch')):
        print(f'{msg_type} has no land_switch/kill_switch fields')
        sys.exit(1)

    q = QoSProfile(depth=10)
    pub = node.create_publisher(cls, a.output, q)
    deb_rc = Debouncer(a.hold)
    deb_op = Debouncer(a.hold)
    auto = AutoLand(a.auto_after, a.auto_lock)
    st = {'rc_land': False, 'rc_kill': False, 'rc_t': -1e9, 'op': False,
          'phase': 'IDLE', 'lock': False, 'lock_t': -1e9, 'src': '', 'warn': 0.0}

    def on_rc(m):
        st['rc_land'] = bool(m.land_switch)
        st['rc_kill'] = bool(m.kill_switch)
        st['rc_t'] = time.monotonic()

    def on_op(m):
        st['op'] = bool(m.data)

    def on_cancel(m):
        if m.data:
            auto.cancel(time.monotonic())
            node.get_logger().info('auto-land cancelled, timers restarted')

    def on_phase(m):
        st['phase'] = m.data

    def on_snap(m):
        st['lock'] = bool(m.valid) and bool(m.marker_detected)
        st['lock_t'] = time.monotonic()

    node.create_subscription(cls, a.rc_input, on_rc, q)
    node.create_subscription(Bool, a.operator_input, on_op, q)
    node.create_subscription(Bool, a.cancel_input, on_cancel, q)
    node.create_subscription(String, a.phase_topic, on_phase, qos_profile_sensor_data)
    node.create_subscription(InputSnapshot, a.snap_topic, on_snap, qos_profile_sensor_data)

    def tick():
        now = time.monotonic()
        rc_fresh = now - st['rc_t'] < a.rc_stale
        rc_out = deb_rc.update(st['rc_land'] if rc_fresh else False, now)
        op_out = deb_op.update(st['op'], now)
        lock_ok = st['lock'] and now - st['lock_t'] < 0.5
        auto_out = auto.update(now, st['phase'], lock_ok)
        land = rc_out or op_out or auto_out

        m = cls()
        m.land_switch = land
        m.kill_switch = bool(st['rc_kill']) if rc_fresh else False
        pub.publish(m)

        src = ','.join(n for n, v in (('rc', rc_out), ('operator', op_out), ('auto', auto_out)) if v)
        if src != st['src']:
            node.get_logger().info(f'land_switch={land} source=[{src}] phase={st["phase"]}')
            st['src'] = src
        if now - st['warn'] > 5.0:
            st['warn'] = now
            if node.count_publishers(a.output) > 1:
                node.get_logger().warn(
                    f'Another publisher is active on {a.output} (rc_node not remapped?), signal will flicker')

    node.create_timer(1.0 / a.rate, tick)
    node.get_logger().info(
        f'land_arbiter: rc={a.rc_input} operator={a.operator_input} -> {a.output} '
        f'({msg_type}), debounce {a.hold:.2f}s, auto-after {a.auto_after:.0f}s')
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
