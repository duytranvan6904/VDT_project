import math
import subprocess
import threading
import time
from collections import defaultdict

import rclpy
import tf2_ros
from geometry_msgs.msg import PointStamped
from nav_msgs.msg import Odometry
from px4_msgs.msg import BatteryStatus, VehicleCommand, VehicleStatus
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, QoSProfile, ReliabilityPolicy,
                       qos_profile_sensor_data)
from std_msgs.msg import Bool, Float32, String, UInt8
from tf2_geometry_msgs import do_transform_point
from vdt_msgs.msg import (AltEstimate, InputSnapshot, OffboardStatus,
                          PlannerOutput, RcChannelsRaw, RcFsmInput,
                          TimeoutFlags)
from vision_msgs.msg import BoundingBox2D

SEARCH, FOLLOW, APPROACH, LAND, COMPLETE = range(5)
ARMED, DISARMED = 2, 1
NAV_OFFBOARD, NAV_LOITER, NAV_RTL = 14, 4, 5
FX = FY = 466.0
CX, CY, IMG_W, IMG_H = 320.0, 240.0, 640, 480
MARKER_SIZE_M = 0.15
MAX_RANGE_M = 12.0


def wait_until(cond, timeout, act=None, act_period=1.0):
    end = time.monotonic() + timeout
    next_act = 0.0
    while time.monotonic() < end:
        if cond():
            return True
        now = time.monotonic()
        if act is not None and now >= next_act:
            act()
            next_act = now + act_period
        time.sleep(0.05)
    return cond()


def is_subsequence(expected, seq):
    it = iter(seq)
    return all(any(x == y for y in it) for x in expected)


def pkill(pattern):
    subprocess.run(['pkill', '-f', pattern], check=False)


def pgrep(pattern):
    return subprocess.run(['pgrep', '-f', pattern], capture_output=True).returncode == 0


def ros_param(node, name):
    out = subprocess.run(['ros2', 'param', 'get', node, name],
                         capture_output=True, text=True, timeout=15).stdout
    return out.strip().split()[-1]


class Probe(Node):

    def __init__(self, pad=(6.0, 0.0, 0.0)):
        super().__init__('sitl_probe')
        self.pad = pad
        self.mock_on = False
        self.blackout = False
        self.outlier = False
        self.land_switch = False
        self.force_land = False
        self.low_batt = False
        self.rc_raw_on = False
        self.kill_pwm = 1000

        self.cnt = defaultdict(int)
        self.state = None
        self.history = []
        self.snap = None
        self.snap_count = 0
        self.max_gap = 0.0
        self.max_gap_sp = 0.0
        self._last_snap = None
        self._last_sp = None
        self.odom = None
        self.filt = None
        self.mode = None
        self.phase = None
        self.arming = None
        self.nav = None
        self.sp = None
        self.gimbal = math.nan
        self.flags = None
        self.offb = None
        self.killed = None
        self.inhibit = None
        self.log = []

        self.buf = tf2_ros.Buffer()
        self.tfl = tf2_ros.TransformListener(self.buf, self)

        latched = QoSProfile(
            depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE)
        sd = qos_profile_sensor_data

        self._sub(UInt8, 'fsm/state', self._on_state)
        self._sub(InputSnapshot, 'input_cache/snapshot', self._on_snap)
        self._sub(Odometry, 'odom', lambda m: setattr(self, 'odom', m), sd)
        self._sub(Odometry, 'hpad/state_filtered', lambda m: setattr(self, 'filt', m))
        self._sub(String, 'ekf/tracking_mode', lambda m: setattr(self, 'mode', m.data))
        self._sub(String, 'mission/phase', lambda m: setattr(self, 'phase', m.data))
        self._sub(PlannerOutput, 'planner/velocity_setpoint', self._on_sp)
        self._sub(AltEstimate, 'alt_estimator/state')
        self._sub(Float32, 'gimbal/target_angle_deg', lambda m: setattr(self, 'gimbal', m.data))
        self._sub(TimeoutFlags, 'input_cache/timeout_flags', lambda m: setattr(self, 'flags', m))
        self._sub(OffboardStatus, 'offboard/status', lambda m: setattr(self, 'offb', m))
        self._sub(Bool, 'system/killed', lambda m: setattr(self, 'killed', m.data), latched)
        self._sub(Bool, 'safety/inhibit_offboard', lambda m: setattr(self, 'inhibit', m.data))
        self._sub(VehicleStatus, '/fmu/out/vehicle_status', self._on_status, sd)

        self.pub_det = self.create_publisher(Bool, 'hpad/detected', 10)
        self.pub_bbox = self.create_publisher(BoundingBox2D, 'hpad/bbox', 10)
        self.pub_pos = self.create_publisher(PointStamped, 'hpad/position_camera', 10)
        self.pub_rc = self.create_publisher(RcFsmInput, 'rc/fsm_input', 10)
        self.pub_rc_raw = self.create_publisher(RcChannelsRaw, 'rc/channels_raw', 10)
        self.pub_force = self.create_publisher(Bool, 'safety/force_land', 10)
        self.pub_killed = self.create_publisher(Bool, 'system/killed', latched)
        self.pub_cmd = self.create_publisher(VehicleCommand, '/fmu/in/vehicle_command', 10)
        self.pub_batt = self.create_publisher(BatteryStatus, '/fmu/out/battery_status', sd)

        self.pub_killed.publish(Bool(data=False))
        self.create_timer(1.0 / 30.0, self._vision)
        self.create_timer(0.1, self._rc)
        self.create_timer(0.02, self._batt)
        self.create_timer(0.05, self._log)

    def _sub(self, msg, topic, cb=None, qos=10):
        def f(m):
            self.cnt[topic] += 1
            if cb is not None:
                cb(m)
        return self.create_subscription(msg, topic, f, qos)

    def _on_state(self, m):
        if m.data != self.state:
            self.state = m.data
            self.history.append((time.monotonic(), m.data))

    def _on_snap(self, m):
        now = time.monotonic()
        if self._last_snap is not None:
            self.max_gap = max(self.max_gap, now - self._last_snap)
        self._last_snap = now
        self.snap = m
        self.snap_count += 1

    def _on_sp(self, m):
        now = time.monotonic()
        if self._last_sp is not None:
            self.max_gap_sp = max(self.max_gap_sp, now - self._last_sp)
        self._last_sp = now
        self.sp = m

    def _on_status(self, m):
        self.arming = m.arming_state
        self.nav = m.nav_state

    def states(self):
        return [s for _, s in self.history]

    def reset_stats(self):
        self.max_gap = 0.0
        self.max_gap_sp = 0.0
        self._last_snap = None
        self._last_sp = None

    def drop_killed_pub(self):
        self.destroy_publisher(self.pub_killed)

    def _rc(self):
        rc = RcFsmInput()
        rc.land_switch = self.land_switch
        rc.kill_switch = False
        self.pub_rc.publish(rc)
        if self.force_land:
            self.pub_force.publish(Bool(data=True))
        if self.rc_raw_on:
            raw = RcChannelsRaw()
            ch = [1500] * 16
            ch[5] = int(self.kill_pwm)
            raw.ch = ch
            raw.valid = True
            raw.failsafe = False
            self.pub_rc_raw.publish(raw)

    def _batt(self):
        if not self.low_batt:
            return
        m = BatteryStatus()
        m.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        m.connected = True
        m.remaining = 0.08
        m.voltage_v = 13.0
        m.warning = 2
        self.pub_batt.publish(m)

    def _log(self):
        if self.odom is None or self.state is None:
            return
        v = self.odom.twist.twist.linear
        sp = self.sp
        self.log.append(dict(
            state=self.state, gimbal=self.gimbal, z=self.z(),
            vx=v.x, vy=v.y, vz=v.z,
            sx=sp.vx if sp else math.nan,
            sy=sp.vy if sp else math.nan,
            sz=sp.vz if sp else math.nan))

    def _project(self):
        try:
            tf = self.buf.lookup_transform(
                'camera_optical_frame', 'world', rclpy.time.Time())
        except Exception:
            return None
        ps = PointStamped()
        ps.header.frame_id = 'world'
        ps.header.stamp = tf.header.stamp
        ps.point.x, ps.point.y, ps.point.z = self.pad
        pc = do_transform_point(ps, tf)
        x, y, z = pc.point.x, pc.point.y, pc.point.z
        if self.blackout or z < 0.3 or math.sqrt(x * x + y * y + z * z) > MAX_RANGE_M:
            return None
        u, v = FX * x / z + CX, FY * y / z + CY
        if not (0.0 <= u < IMG_W and 0.0 <= v < IMG_H):
            return None
        return tf.header.stamp, x, y, z, u, v

    def _vision(self):
        if not self.mock_on:
            return
        hit = self._project()
        self.pub_det.publish(Bool(data=hit is not None))
        if hit is None:
            return
        stamp, x, y, z, u, v = hit
        pos = PointStamped()
        pos.header.frame_id = 'camera_optical_frame'
        pos.header.stamp = stamp
        pos.point.x = x + (3.0 if self.outlier else 0.0)
        pos.point.y, pos.point.z = y, z
        self.pub_pos.publish(pos)
        box = BoundingBox2D()
        box.center.position.x, box.center.position.y = u, v
        box.size_x = box.size_y = max(12.0, FX * MARKER_SIZE_M / z)
        self.pub_bbox.publish(box)

    def send_cmd(self, command, **params):
        m = VehicleCommand()
        m.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        m.command = command
        for i in range(1, 8):
            setattr(m, f'param{i}', params.get(f'p{i}', math.nan))
        m.target_system, m.target_component = 1, 1
        m.source_system, m.source_component = 255, 1
        m.from_external = True
        self.pub_cmd.publish(m)

    def arm(self):
        self.send_cmd(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM,
                      p1=1.0, p2=0.0, p3=0.0, p4=0.0, p5=0.0, p6=0.0, p7=0.0)

    def takeoff(self):
        self.send_cmd(VehicleCommand.VEHICLE_CMD_NAV_TAKEOFF)

    def z(self):
        return self.odom.pose.pose.position.z if self.odom else math.nan

    def hspeed(self):
        v = self.odom.twist.twist.linear
        return math.hypot(v.x, v.y)

    def vspeed(self):
        return abs(self.odom.twist.twist.linear.z)

    def pad_error(self):
        p = self.filt.pose.pose.position
        return math.sqrt((p.x - self.pad[0]) ** 2 + (p.y - self.pad[1]) ** 2
                         + (p.z - self.pad[2]) ** 2)


class Runner:
    def __init__(self):
        rclpy.init()
        self.probe = Probe()
        self.ex = MultiThreadedExecutor(num_threads=3)
        self.ex.add_node(self.probe)
        self.th = threading.Thread(target=self.ex.spin, daemon=True)
        self.th.start()

    def close(self):
        self.ex.shutdown()
        self.probe.destroy_node()
        rclpy.shutdown()


def ensure_airborne(p):
    assert wait_until(
        lambda: p.state is not None and p.snap is not None and p.snap.valid, 60), \
        'stack chua san sang (fsm/state, snapshot.valid)'
    assert wait_until(lambda: p.nav is not None, 20), 'khong nhan vehicle_status'
    assert wait_until(lambda: p.arming == ARMED, 30, act=p.arm), 'arm that bai'
    assert wait_until(lambda: p.z() >= 2.0, 60, act=p.takeoff), 'khong len duoc 2 m'
    assert wait_until(lambda: p.nav == NAV_OFFBOARD, 40), 'offboard chua engage'
    assert p.state == SEARCH


def ensure_follow(p):
    ensure_airborne(p)
    p.mock_on = True
    assert wait_until(lambda: p.state == FOLLOW, 90), \
        f'khong vao FOLLOW, mode={p.mode}, phase={p.phase}'