#!/usr/bin/env python3
import argparse
import math
import sys
import threading
import time
from collections import Counter, deque

import cv2
import numpy as np
import rclpy
from px4_msgs.msg import (BatteryStatus, OffboardControlMode, SensorGps, TrajectorySetpoint,
                          VehicleCommand, VehicleLandDetected, VehicleLocalPosition,
                          VehicleOdometry, VehicleStatus)
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Bool, String, UInt8
from vdt_msgs.msg import RcFsmInput

from obstacle_world import make_cylinders

NAV_MANUAL = getattr(VehicleStatus, 'NAVIGATION_STATE_MANUAL', 0)
NAV_RTL = getattr(VehicleStatus, 'NAVIGATION_STATE_AUTO_RTL', 5)
NAV_LOITER = getattr(VehicleStatus, 'NAVIGATION_STATE_AUTO_LOITER', 4)
NAV_OFFBOARD = getattr(VehicleStatus, 'NAVIGATION_STATE_OFFBOARD', 14)
NAV_TAKEOFF = getattr(VehicleStatus, 'NAVIGATION_STATE_AUTO_TAKEOFF', 17)
NAV_LAND = getattr(VehicleStatus, 'NAVIGATION_STATE_AUTO_LAND', 18)
ARM_ON = getattr(VehicleStatus, 'ARMING_STATE_ARMED', 2)
ARM_OFF = getattr(VehicleStatus, 'ARMING_STATE_DISARMED', 1)
CMD_ARM = getattr(VehicleCommand, 'VEHICLE_CMD_COMPONENT_ARM_DISARM', 400)
CMD_MODE = getattr(VehicleCommand, 'VEHICLE_CMD_DO_SET_MODE', 176)
CMD_TAKEOFF = getattr(VehicleCommand, 'VEHICLE_CMD_NAV_TAKEOFF', 22)
CMD_LAND = getattr(VehicleCommand, 'VEHICLE_CMD_NAV_LAND', 21)
CMD_RTL = getattr(VehicleCommand, 'VEHICLE_CMD_NAV_RETURN_TO_LAUNCH', 20)
CMD_ACT = getattr(VehicleCommand, 'VEHICLE_CMD_ACTUATOR_TEST', 310)
CMD_NAMES = {CMD_ARM: 'ARM_DISARM', CMD_MODE: 'DO_SET_MODE', CMD_TAKEOFF: 'NAV_TAKEOFF',
             CMD_LAND: 'NAV_LAND', CMD_RTL: 'NAV_RTL', CMD_ACT: 'ACTUATOR_TEST'}
POSE_NED = getattr(VehicleOdometry, 'POSE_FRAME_NED', 1)
VEL_NED = getattr(VehicleOdometry, 'VELOCITY_FRAME_NED', 1)
RLO = np.array([[0.0, 0.0, 1.0], [-1.0, 0.0, 0.0], [0.0, -1.0, 0.0]])

RING = [(3.0, 35.0, 0.4, 3.5), (3.0, -35.0, 0.4, 3.5), (4.2, 15.0, 0.5, 3.0),
        (2.6, 150.0, 0.4, 3.5), (2.6, 215.0, 0.4, 2.5)]
SCENARIOS = {
    'nominal': dict(num_obs=15, ring=False, blind=[]),
    'pad_ring': dict(num_obs=0, ring=True, blind=[]),
    'blind_high': dict(num_obs=8, ring=False, blind=[(2.6, 1.9)]),
    'blind_low': dict(num_obs=8, ring=False, blind=[(1.0, 0.0)]),
    'blind_multi': dict(num_obs=8, ring=True, blind=[(2.8, 2.3), (1.6, 1.2), (0.7, 0.0)]),
}

def pair(s):
    return [float(v) for v in s.split(',')]


def bands(s):
    out = []
    for b in s.split(','):
        hi, lo = (float(v) for v in b.split(':'))
        out.append((max(hi, lo), min(hi, lo)))
    return out


def wrap(x):
    return math.atan2(math.sin(x), math.cos(x))


def fill(msg, skipped, **kw):
    for k, v in kw.items():
        if hasattr(msg, k):
            setattr(msg, k, v)
        else:
            skipped.add(type(msg).__name__ + '.' + k)


def clearance(pos, cyl):
    best = math.inf
    for cx, cy, r, h in cyl:
        dh = max(0.0, math.hypot(pos[0] - cx, pos[1] - cy) - r)
        dv = max(0.0, -pos[2], pos[2] - h)
        best = min(best, math.hypot(dh, dv))
    return best


def build_obstacles(a, sc):
    cyl = make_cylinders(a.seed, sc['num_obs'], a.map_size, a.obs_height,
                         a.clear_radius, a.start, [])
    if sc['ring']:
        for rr, ang, r, h in RING:
            t = math.radians(ang)
            cyl.append((a.pad[0] + rr * math.cos(t), a.pad[1] + rr * math.sin(t), r, h))
    cyl += [tuple(e) for e in a.extra or []]
    return cyl


def img_msg(arr, enc, step, stamp, frame):
    m = Image()
    m.header.stamp = stamp
    m.header.frame_id = frame
    m.height, m.width = arr.shape
    m.encoding = enc
    m.is_bigendian = False
    m.step = step
    m.data = arr.tobytes()
    return m

class Sim(Node):
    def __init__(self, a, cyl, sc):
        super().__init__('sim_hardware_data')
        self.a = a
        self.cyl = cyl
        self.blind = bands(a.blind) if a.blind else sc['blind']
        self.lock = threading.RLock()
        self.rng = np.random.default_rng(a.seed)
        self.skipped = set()
        self.cmds = Counter()
        self.vals = {}
        self.t0 = time.monotonic()
        self.t_last = self.t0
        self.pos = np.array([a.start[0], a.start[1], 0.0])
        self.vel = np.zeros(3)
        self.yaw_ned = wrap(math.pi / 2.0 - math.radians(a.start_yaw))
        self.gimbal = 0.0
        self.servo_target = 0.0
        self.servo_deadline = math.inf
        self.servo_live = False
        self.servo_timeouts = 0
        self.servo_mismatch = 0
        self.act_last = None
        self.act_gap_max = 0.0
        self.armed = False
        self.nav = NAV_MANUAL
        self.failsafe = False
        self.failsafe_seen = False
        self.force_seen = False
        self.sp = None
        self.t_sp = -1e9
        self.t_hb = -1e9
        self.hb_n = self.sp_n = 0
        self.hb_t = [None, None]
        self.sp_t = [None, None]
        self.takeoff_z = a.takeoff_alt
        self.pad = list(a.pad)
        self.hist = deque(maxlen=300)
        self.odom_q = deque()
        self.n = 0
        self.phase = 'IDLE'
        self.fsm = None
        self.t_follow = None
        self.t_arm = self.t_offboard = self.t_td = None
        self.airborne = False
        self.flew = False
        self.landed = True
        self.impact_vz = None
        self.touch_dxy = None
        self.min_clr = math.inf
        self.clr_by = {}
        self.collision = False
        self.t_end = None
        self.done = False
        self.cam_flags = (False, False, 0.0)
        self.cam_n = 0
        self.cam_ms = 0.0
        self.det_view_n = self.det_view_hit = self.det_false = 0
        self.det_min_px = math.inf
        self.pivot = np.array(a.pivot)
        self.cam_off = np.array(a.cam_offset)

        self.W, self.H = a.width, a.height
        self.fx = (self.W / 2.0) / math.tan(math.radians(a.hfov) / 2.0)
        self.fy = self.fx
        self.cx, self.cy = self.W / 2.0, self.H / 2.0
        s = a.depth_stride
        uu, vv = np.meshgrid(np.arange(0, self.W, s), np.arange(0, self.H, s))
        self.grid = uu.shape
        self.dirs = np.column_stack([((uu - self.cx) / self.fx).ravel(),
                                     ((vv - self.cy) / self.fy).ravel(),
                                     np.ones(uu.size)])
        dic = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_50)
        if hasattr(cv2.aruco, 'generateImageMarker'):
            mk = cv2.aruco.generateImageMarker(dic, a.marker_id, 192)
        else:
            mk = cv2.aruco.drawMarker(dic, a.marker_id, 192)
        self.tex = np.full((240, 240), 255, np.uint8)
        self.tex[24:216, 24:216] = mk
        self.tex_mask = np.full((240, 240), 255, np.uint8)
        self.tex_c = np.float32([[0, 0], [240, 0], [240, 240], [0, 240]])
        self.pad_side = a.marker_size * 10.0 / 8.0
        self.noise = self.rng.normal(0.0, a.ir_noise, (6, self.H, self.W)).astype(np.int16)

        sd = qos_profile_sensor_data
        rel = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, depth=5)
        self.ga = MutuallyExclusiveCallbackGroup()
        self.gb = MutuallyExclusiveCallbackGroup()
        self.gc = MutuallyExclusiveCallbackGroup()
        self.p_odom = self.create_publisher(VehicleOdometry, a.odom_topic, sd)
        self.p_local = self.create_publisher(VehicleLocalPosition, a.local_topic, sd)
        self.p_status = self.create_publisher(VehicleStatus, a.status_topic, sd)
        self.p_batt = self.create_publisher(BatteryStatus, a.battery_topic, sd)
        self.p_land = self.create_publisher(VehicleLandDetected, a.land_topic, sd)
        self.p_gps = self.create_publisher(SensorGps, a.gps_topic, sd)
        self.p_ir = self.create_publisher(Image, a.ir_topic, sd)
        self.p_depth = self.create_publisher(Image, a.depth_topic, sd)
        self.p_info1 = self.create_publisher(CameraInfo, a.info_topic, rel)
        self.p_info2 = self.create_publisher(CameraInfo, '/camera_info', rel)
        self.p_rc = self.create_publisher(RcFsmInput, '/rc/fsm_input', QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE, depth=10))
        self.create_subscription(OffboardControlMode, '/fmu/in/offboard_control_mode',
                                 self.on_hb, sd, callback_group=self.ga)
        self.create_subscription(TrajectorySetpoint, '/fmu/in/trajectory_setpoint',
                                 self.on_sp, sd, callback_group=self.ga)
        self.create_subscription(VehicleCommand, '/fmu/in/vehicle_command',
                                 self.on_cmd, sd, callback_group=self.ga)
        mon = {'/mission/phase': (String, 'phase'), '/fsm/state': (UInt8, 'fsm'),
               '/landing/touchdown': (Bool, 'td'), '/landing/safe_to_land': (Bool, 'safe'),
               '/landing/active': (Bool, 'active'), '/landing/sub_phase': (String, 'sub'),
               '/ekf/tracking_mode': (String, 'mode'), '/safety/force_land': (Bool, 'force')}
        for topic, (typ, key) in mon.items():
            self.create_subscription(typ, topic, lambda m, k=key: self.on_val(k, m),
                                     sd, callback_group=self.gc)
        self.create_subscription(Bool, '/hpad/detected', self.on_det, sd, callback_group=self.gc)
        self.create_timer(0.02, self.step, callback_group=self.ga)
        self.create_timer(1.0 / a.cam_hz, self.cam_tick, callback_group=self.gb)
        self.create_timer(1.0, self.log, callback_group=self.gc)
        if not a.no_rc:
            self.create_timer(0.05, self.rc_tick, callback_group=self.gc)

    def t(self):
        return time.monotonic() - self.t0

    def ts(self):
        return int(self.get_clock().now().nanoseconds // 1000)

    def event(self, msg):
        print(f'[t={self.t():6.1f}] {msg}', flush=True)

    def on_hb(self, m):
        now = time.monotonic()
        self.t_hb = now
        self.hb_n += 1
        self.hb_t = [now if self.hb_t[0] is None else self.hb_t[0], now]

    def on_sp(self, m):
        now = time.monotonic()
        v = m.velocity
        self.sp = (float(v[0]), float(v[1]), float(v[2]), float(m.yaw), float(m.yawspeed))
        self.t_sp = now
        self.sp_n += 1
        self.sp_t = [now if self.sp_t[0] is None else self.sp_t[0], now]

    def on_cmd(self, m):
        now = time.monotonic()
        c = m.command
        a = self.a
        with self.lock:
            self.cmds[c] += 1
            if c == CMD_ARM:
                if m.param1 >= 0.5:
                    self.arm()
                else:
                    self.disarm(abs(m.param2 - 21196.0) < 0.5)
            elif c == CMD_MODE:
                self.set_mode(int(m.param2), int(m.param3), now)
            elif c == CMD_TAKEOFF:
                if self.armed and self.pos[2] < 0.5:
                    self.nav = NAV_TAKEOFF
                    self.event(f'NAV_TAKEOFF -> len {self.takeoff_z:.1f} m')
                else:
                    self.event('NAV_TAKEOFF bi bo qua (chua arm hoac dang bay)')
            elif c == CMD_LAND:
                self.nav = NAV_LAND
                self.event('NAV_LAND')
            elif c == CMD_RTL:
                self.nav = NAV_RTL
                self.event('NAV_RTL (mo phong thanh ha canh tai cho)')
            elif c == CMD_ACT:
                self.on_servo(m, now)

    def on_servo(self, m, now):
        a = self.a
        if abs(m.param5 - a.servo_function) > 0.5:
            self.servo_mismatch += 1
        elif math.isnan(m.param1):
            self.servo_target = 0.0
            self.servo_deadline = math.inf
            self.servo_live = False
        else:
            if self.act_last is not None:
                self.act_gap_max = max(self.act_gap_max, now - self.act_last)
            self.act_last = now
            self.servo_target = float(np.clip(m.param1, -1.0, 1.0)) * a.servo_scale
            self.servo_deadline = now + m.param2 if m.param2 > 0.0 else math.inf
            self.servo_live = True

    def arm(self):
        if not self.a.preflight_ok:
            self.event('ARM bi tu choi: preflight fail')
            return
        if not self.armed:
            self.armed = True
            self.t_arm = time.monotonic()
            self.event('ARMED')

    def disarm(self, force):
        if self.pos[2] > 0.15 and not force:
            self.event('DISARM tren khong bi tu choi')
            return
        if self.armed:
            self.armed = False
            self.failsafe = False
            self.event('FORCE DISARM' + (' tren khong' if self.pos[2] > 0.15 else '')
                       if force else 'DISARMED')

    def set_mode(self, main, sub, now):
        if main == 6:
            if now - self.t_hb < 0.5 and now - self.t_sp < 0.5:
                if self.nav != NAV_OFFBOARD:
                    self.event('OFFBOARD duoc chap nhan')
                self.nav = NAV_OFFBOARD
                self.failsafe = False
                if self.t_offboard is None:
                    self.t_offboard = now
            else:
                self.cmds['offboard_refused'] += 1
                self.event('OFFBOARD bi tu choi: thieu heartbeat/setpoint')
        elif main == 4:
            self.nav = {2: NAV_TAKEOFF, 3: NAV_LOITER, 5: NAV_RTL, 6: NAV_LAND}.get(sub, NAV_LOITER)
            self.failsafe = False
            self.event(f'doi mode AUTO sub={sub} nav={self.nav}')

    def step(self):
        now = time.monotonic()
        dt = min(now - self.t_last, 0.1)
        self.t_last = now
        with self.lock:
            self.dynamics(now, dt)
            self.hist.append((now, self.pos.copy(), self.yaw_ned, self.gimbal))
            self.track(now)
        self.n += 1
        self.pub_odom(now)
        if self.n % 2 == 0:
            self.pub_local()
        if self.n % 5 == 0:
            self.pub_status()
            self.pub_land()
        if self.n % 10 == 0:
            self.pub_battery(now)
            self.pub_gps()

    def dynamics(self, now, dt):
        a = self.a
        if a.pad_vel:
            t = now - self.t0
            self.pad = [a.pad[0] + a.pad_vel[0] * t, a.pad[1] + a.pad_vel[1] * t]
        v_cmd = np.zeros(3)
        yaw_rate = 0.0
        yaw_tgt = None
        if self.armed:
            if self.nav == NAV_OFFBOARD:
                if now - self.t_hb > a.offboard_loss or now - self.t_sp > a.offboard_loss:
                    self.nav = NAV_LOITER
                    self.failsafe = True
                    self.failsafe_seen = True
                    self.event('mat heartbeat/setpoint offboard -> LOITER, failsafe')
                elif self.sp is not None:
                    vn, ve, vd, yaw, yr = self.sp
                    v_cmd = np.nan_to_num(np.array([ve, vn, -vd]))
                    yaw_rate = yr if math.isfinite(yr) else 0.0
                    yaw_tgt = yaw if math.isfinite(yaw) else None
            elif self.nav == NAV_TAKEOFF:
                err = self.takeoff_z - self.pos[2]
                v_cmd[2] = float(np.clip(1.5 * err, -0.5, a.climb_rate))
                if err < 0.1:
                    self.nav = NAV_LOITER
                    self.event('takeoff xong -> LOITER')
            elif self.nav in (NAV_LAND, NAV_RTL):
                v_cmd[2] = -a.land_rate
        if not self.armed and self.pos[2] > 0.0:
            self.vel[2] -= 9.81 * dt
            self.vel[:2] *= 0.99
        elif self.armed:
            acc = (v_cmd - self.vel) / a.tau
            n = float(np.linalg.norm(acc))
            if n > a.amax:
                acc *= a.amax / n
            self.vel += acc * dt
        else:
            self.vel[:] = 0.0
        self.pos += self.vel * dt
        if self.pos[2] > 0.3:
            self.airborne = True
            self.flew = True
        if self.pos[2] <= 0.0:
            if self.airborne:
                self.airborne = False
                self.impact_vz = float(self.vel[2])
                self.touch_dxy = math.hypot(self.pad[0] - self.pos[0], self.pad[1] - self.pos[1])
                self.event(f'cham dat vz={self.impact_vz:.2f} m/s dxy_pad={self.touch_dxy:.2f} m')
            self.pos[2] = 0.0
            self.vel[2] = max(self.vel[2], 0.0)
            if v_cmd[2] <= 0.0:
                self.vel[:2] = 0.0
        self.landed = self.pos[2] <= 0.03
        if self.armed:
            if yaw_tgt is not None:
                d = wrap(yaw_tgt - self.yaw_ned)
                lim = a.yaw_rate_max * dt
                self.yaw_ned = wrap(self.yaw_ned + max(-lim, min(lim, d)))
            else:
                self.yaw_ned = wrap(self.yaw_ned + yaw_rate * dt)
        if self.servo_live and now > self.servo_deadline:
            self.servo_target = 0.0
            self.servo_live = False
            self.servo_timeouts += 1
            self.event('servo output het han (thieu keepalive)')
        step = a.servo_rate * dt
        self.gimbal += max(-step, min(step, self.servo_target - self.gimbal))

    def track(self, now):
        clr = clearance(self.pos, self.cyl)
        if self.pos[2] > 0.02 and self.armed:
            self.min_clr = min(self.min_clr, clr)
            key = self.phase if self.phase in ('FOLLOW', 'APPROACH', 'LAND') else 'OTHER'
            self.clr_by[key] = min(self.clr_by.get(key, math.inf), clr)
        if clr < self.a.drone_radius and not self.collision:
            self.collision = True
            self.event(f'VA CHAM: clr={clr:.2f} m')
        ended = (self.fsm == 4 or self.t_td is not None) and self.landed
        if ended or (self.flew and not self.armed and self.landed):
            if self.t_end is None:
                self.t_end = now
            elif now - self.t_end > self.a.end_hold:
                self.done = True

    def pub_odom(self, now):
        a = self.a
        pos = self.pos + self.rng.normal(0.0, a.pos_noise, 3)
        vel = self.vel + self.rng.normal(0.0, a.vel_noise, 3)
        yaw = self.yaw_ned + float(self.rng.normal(0.0, a.yaw_noise))
        ts = self.ts()
        m = VehicleOdometry()
        fill(m, self.skipped, timestamp=ts, timestamp_sample=ts, pose_frame=POSE_NED,
             position=[float(pos[1]), float(pos[0]), float(-pos[2])],
             q=[math.cos(yaw / 2.0), 0.0, 0.0, math.sin(yaw / 2.0)],
             velocity_frame=VEL_NED,
             velocity=[float(vel[1]), float(vel[0]), float(-vel[2])],
             angular_velocity=[0.0, 0.0, 0.0],
             position_variance=[a.pos_noise ** 2] * 3,
             orientation_variance=[a.yaw_noise ** 2] * 3,
             velocity_variance=[a.vel_noise ** 2] * 3)
        self.odom_q.append((now + a.delay, m))
        while self.odom_q and self.odom_q[0][0] <= now:
            self.p_odom.publish(self.odom_q.popleft()[1])

    def pub_local(self):
        ts = self.ts()
        m = VehicleLocalPosition()
        fill(m, self.skipped, timestamp=ts, timestamp_sample=ts, xy_valid=True, z_valid=True,
             v_xy_valid=True, v_z_valid=True, dead_reckoning=False,
             x=float(self.pos[1]), y=float(self.pos[0]), z=float(-self.pos[2]),
             vx=float(self.vel[1]), vy=float(self.vel[0]), vz=float(-self.vel[2]),
             heading=float(self.yaw_ned), eph=0.2, epv=0.3)
        self.p_local.publish(m)

    def pub_status(self):
        m = VehicleStatus()
        fill(m, self.skipped, timestamp=self.ts(),
             arming_state=ARM_ON if self.armed else ARM_OFF, nav_state=self.nav,
             failsafe=self.failsafe, pre_flight_checks_pass=self.a.preflight_ok)
        self.p_status.publish(m)

    def pub_land(self):
        m = VehicleLandDetected()
        fill(m, self.skipped, timestamp=self.ts(), landed=self.landed,
             ground_contact=self.landed, maybe_landed=self.landed, freefall=False)
        self.p_land.publish(m)

    def pub_battery(self, now):
        rem = max(0.05, self.a.battery - 0.0005 * (now - self.t0))
        m = BatteryStatus()
        fill(m, self.skipped, timestamp=self.ts(), connected=True, voltage_v=15.8,
             current_a=12.0, remaining=float(rem))
        self.p_batt.publish(m)

    def pub_gps(self):
        m = SensorGps()
        fill(m, self.skipped, timestamp=self.ts(), fix_type=3, satellites_used=14,
             eph=0.8, epv=1.2, hdop=0.7, vdop=1.0)
        self.p_gps.publish(m)

    def cam_pose(self, pos, yaw_ned, gim):
        yaw = math.pi / 2.0 - yaw_ned
        cy, sy = math.cos(yaw), math.sin(yaw)
        rwb = np.array([[cy, -sy, 0.0], [sy, cy, 0.0], [0.0, 0.0, 1.0]])
        g = math.radians(-gim)
        ry = np.array([[math.cos(g), 0.0, math.sin(g)], [0.0, 1.0, 0.0],
                       [-math.sin(g), 0.0, math.cos(g)]])
        return rwb @ ry @ RLO, pos + rwb @ (self.pivot + ry @ self.cam_off)

    def raycast(self, R, C):
        a = self.a
        d = self.dirs @ R.T
        dx, dy, dz = d[:, 0], d[:, 1], d[:, 2]
        t = np.full(len(d), np.inf)
        hit = np.zeros(len(d), bool)
        with np.errstate(all='ignore'):
            tg = np.where(dz < -1e-6, -C[2] / dz, np.inf)
            t = np.where(tg > 0.0, tg, np.inf)
            for cx, cy, r, h in self.cyl:
                ox, oy = C[0] - cx, C[1] - cy
                if ox * ox + oy * oy > (a.range_max + r + 2.0) ** 2:
                    continue
                qa = dx * dx + dy * dy
                qb = 2.0 * (ox * dx + oy * dy)
                qc = ox * ox + oy * oy - r * r
                disc = qb * qb - 4.0 * qa * qc
                ts = (-qb - np.sqrt(np.where(disc >= 0.0, disc, np.nan))) / (2.0 * qa)
                zs = C[2] + ts * dz
                side = (qa > 1e-9) & (disc >= 0.0) & (ts > 0.0) & (zs >= 0.0) & (zs <= h)
                tt = np.where(side, ts, np.inf)
                if C[2] > h:
                    tc = (h - C[2]) / dz
                    px, py = ox + tc * dx, oy + tc * dy
                    top = (dz < -1e-6) & (tc > 0.0) & (px * px + py * py <= r * r)
                    tt = np.minimum(tt, np.where(top, tc, np.inf))
                hit |= tt < t
                t = np.minimum(t, tt)
        ok = np.isfinite(t) & (t >= a.range_min) & (t <= a.range_max)
        depth = np.where(ok, t, 0.0)
        depth = depth + self.rng.normal(0.0, 1.0, depth.shape) * a.depth_noise * depth
        return depth.reshape(self.grid), (hit & ok).reshape(self.grid)

    def up(self, g):
        s = self.a.depth_stride
        return np.repeat(np.repeat(g, s, 0), s, 1)[:self.H, :self.W]

    def render_ir(self, R, C, hit, pad, blind):
        img = np.full((self.H, self.W), 90, np.int16)
        cyl_mask = self.up(hit)
        img[cyl_mask] = 45
        s = self.pad_side
        cw = np.array([[pad[0] - s / 2, pad[1] + s / 2, 0.0], [pad[0] + s / 2, pad[1] + s / 2, 0.0],
                       [pad[0] + s / 2, pad[1] - s / 2, 0.0], [pad[0] - s / 2, pad[1] - s / 2, 0.0]])
        pc = (cw - C) @ R
        in_view = False
        px = 0.0
        if np.all(pc[:, 2] > 0.05):
            uv = np.column_stack([self.fx * pc[:, 0] / pc[:, 2] + self.cx,
                                  self.fy * pc[:, 1] / pc[:, 2] + self.cy]).astype(np.float32)
            in_view = bool(np.all((uv[:, 0] >= 0) & (uv[:, 0] < self.W) &
                                  (uv[:, 1] >= 0) & (uv[:, 1] < self.H)))
            px = self.fx * self.a.marker_size / float(np.mean(pc[:, 2]))
            if not blind:
                hm = cv2.getPerspectiveTransform(self.tex_c, uv)
                warped = cv2.warpPerspective(self.tex, hm, (self.W, self.H))
                m = (cv2.warpPerspective(self.tex_mask, hm, (self.W, self.H)) > 127) & ~cyl_mask
                img[m] = warped[m]
        img = img + self.noise[self.cam_n % len(self.noise)]
        return np.clip(img, 0, 255).astype(np.uint8), in_view, px

    def cam_tick(self):
        a = self.a
        c0 = time.monotonic()
        with self.lock:
            if not self.hist:
                return
            target = c0 - a.cam_latency
            _, pos, yaw, gim = min(self.hist, key=lambda h: abs(h[0] - target))
            pad = list(self.pad)
        R, C = self.cam_pose(pos, yaw, gim)
        depth, hit = self.raycast(R, C)
        blind = any(lo <= pos[2] <= hi for hi, lo in self.blind)
        ir, in_view, px = self.render_ir(R, C, hit, pad, blind)
        with self.lock:
            self.cam_flags = (in_view, blind, px)
        stamp = Time(nanoseconds=self.get_clock().now().nanoseconds
                     - int(a.cam_latency * 1e9)).to_msg()
        full = self.up(depth) if False else np.repeat(np.repeat(depth, a.depth_stride, 0),
                                                       a.depth_stride, 1)[:self.H, :self.W]
        if a.depth_enc == '16UC1':
            d = np.clip(full * 1000.0, 0, 65535).astype('<u2')
            dm = img_msg(d, '16UC1', self.W * 2, stamp, 'camera_optical_frame')
        else:
            d = np.where(full > 0.0, full, np.nan).astype('<f4')
            dm = img_msg(d, '32FC1', self.W * 4, stamp, 'camera_optical_frame')
        self.p_ir.publish(img_msg(ir, 'mono8', self.W, stamp, 'camera_optical_frame'))
        self.p_depth.publish(dm)
        info = CameraInfo()
        info.header.stamp = stamp
        info.header.frame_id = 'camera_optical_frame'
        info.height, info.width = self.H, self.W
        info.distortion_model = 'plumb_bob'
        info.d = [0.0] * 5
        info.k = [self.fx, 0.0, self.cx, 0.0, self.fy, self.cy, 0.0, 0.0, 1.0]
        info.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        info.p = [self.fx, 0.0, self.cx, 0.0, 0.0, self.fy, self.cy, 0.0, 0.0, 0.0, 1.0, 0.0]
        self.p_info1.publish(info)
        self.p_info2.publish(info)
        self.cam_n += 1
        self.cam_ms += (time.monotonic() - c0) * 1000.0

    def on_val(self, key, m):
        v = m.data
        if self.vals.get(key) != v:
            self.event(f'{key}: {self.vals.get(key, "-")} -> {v}')
            self.vals[key] = v
        if key == 'phase':
            self.phase = v
            if v == 'FOLLOW' and self.t_follow is None:
                self.t_follow = time.monotonic()
        elif key == 'fsm':
            self.fsm = v
        elif key == 'td' and v and self.t_td is None:
            self.t_td = time.monotonic()
        elif key == 'force' and v:
            self.force_seen = True

    def on_det(self, m):
        with self.lock:
            in_view, blind, px = self.cam_flags
        if in_view and not blind:
            self.det_view_n += 1
            if m.data:
                self.det_view_hit += 1
                self.det_min_px = min(self.det_min_px, px)
        elif m.data:
            self.det_false += 1

    def rc_tick(self):
        on = (self.t_follow is not None and self.a.land_switch_after >= 0.0
              and time.monotonic() - self.t_follow >= self.a.land_switch_after)
        m = RcFsmInput()
        fill(m, self.skipped, land_switch=bool(on), kill_switch=False)
        self.p_rc.publish(m)

    def log(self):
        with self.lock:
            p = self.pos.copy()
            gim = self.gimbal
            fl = self.cam_flags
        v = self.vals
        print(f't={self.t():6.1f} nav={self.nav:2d} arm={int(self.armed)} '
              f'pos=({p[0]:5.2f},{p[1]:5.2f},{p[2]:4.2f}) gim={gim:6.1f} ph={self.phase:<8} '
              f'fsm={self.fsm} mode={v.get("mode", "-")} safe={v.get("safe", "-")} '
              f'act={v.get("active", "-")} sub={v.get("sub", "-")} '
              f'clr={clearance(p, self.cyl):4.2f} view={int(fl[0])} blind={int(fl[1])} '
              f'td={v.get("td", "-")}', flush=True)

    @staticmethod
    def rate(n, ts):
        if ts[0] is None or ts[1] - ts[0] < 1.0:
            return 0.0
        return (n - 1) / (ts[1] - ts[0])

    def summary(self):
        a = self.a
        print('\n=== TONG KET SIM HARDWARE ===')
        print(f'kich ban={a.scenario} seed={a.seed} so tru={len(self.cyl)} blind={self.blind}')
        if self.skipped:
            print('truong px4_msgs khong ton tai (da bo qua):', sorted(self.skipped))
        print('lenh nhan:', {CMD_NAMES.get(k, k): v for k, v in self.cmds.items()})
        print(f'heartbeat offboard={self.rate(self.hb_n, self.hb_t):.1f} Hz '
              f'setpoint={self.rate(self.sp_n, self.sp_t):.1f} Hz')
        print(f'servo: gap lon nhat giua 2 lenh={self.act_gap_max:.2f}s '
              f'het han={self.servo_timeouts} sai function={self.servo_mismatch}')
        if self.cam_n:
            print(f'camera: {self.cam_n} khung, render tb {self.cam_ms / self.cam_n:.1f} ms/khung')
        rate = self.det_view_hit / self.det_view_n if self.det_view_n else 0.0
        mn = f'{self.det_min_px:.1f}' if math.isfinite(self.det_min_px) else '-'
        print(f'marker: ty le detect khi pad trong anh={rate:.2f} ({self.det_view_hit}/{self.det_view_n}) '
              f'detect nham={self.det_false} marker nho nhat da detect={mn} px')
        print('clr toi thieu theo phase:', {k: round(v, 2) for k, v in self.clr_by.items()})
        checks = [
            ('da arm va vao OFFBOARD', self.t_arm is not None and self.t_offboard is not None, ''),
            (f'clr toi thieu >= {a.clr_min} m', self.min_clr >= a.clr_min, f'{self.min_clr:.2f}'),
            ('khong va cham', not self.collision, ''),
            ('touchdown duoc xac nhan', self.t_td is not None, ''),
            (f'dxy luc cham dat <= {a.land_dxy} m',
             self.touch_dxy is not None and self.touch_dxy <= a.land_dxy,
             '-' if self.touch_dxy is None else f'{self.touch_dxy:.2f}'),
            (f'|vz| luc cham dat <= {a.land_vz} m/s',
             self.impact_vz is not None and abs(self.impact_vz) <= a.land_vz,
             '-' if self.impact_vz is None else f'{self.impact_vz:.2f}'),
            ('khong failsafe offboard', not self.failsafe_seen, ''),
            ('khong force_land', not self.force_seen, ''),
            ('servo khong het han', self.servo_timeouts == 0, ''),
        ]
        ok = True
        for name, passed, val in checks:
            ok &= passed
            print(f'{"PASS" if passed else "FAIL"} {name} {val}')
        print('KET LUAN:', 'PASS' if ok else 'FAIL')

    def main():
        ap = argparse.ArgumentParser()
        ap.add_argument('--scenario', default='nominal', choices=sorted(SCENARIOS))
        ap.add_argument('--seed', type=int, default=0)
        ap.add_argument('--duration', type=float, default=300.0)
        ap.add_argument('--start', type=pair, default=[5.0, 0.0])
        ap.add_argument('--start-yaw', type=float, default=180.0)
        ap.add_argument('--pad', type=pair, default=[0.0, 0.0])
        ap.add_argument('--pad-vel', type=pair, default=None)
        ap.add_argument('--takeoff-alt', type=float, default=3.0)
        ap.add_argument('--blind', default='')
        ap.add_argument('--extra', type=pair, action='append')
        ap.add_argument('--map-size', type=float, default=25.0)
        ap.add_argument('--obs-height', type=float, default=4.0)
        ap.add_argument('--clear-radius', type=float, default=3.5)
        ap.add_argument('--marker-id', type=int, default=42)
        ap.add_argument('--marker-size', type=float, default=0.15)
        ap.add_argument('--width', type=int, default=640)
        ap.add_argument('--height', type=int, default=480)
        ap.add_argument('--hfov', type=float, default=87.0)
        ap.add_argument('--cam-hz', type=float, default=15.0)
        ap.add_argument('--cam-latency', type=float, default=0.04)
        ap.add_argument('--depth-stride', type=int, default=8)
        ap.add_argument('--depth-enc', default='16UC1', choices=['16UC1', '32FC1'])
        ap.add_argument('--range-min', type=float, default=0.3)
        ap.add_argument('--range-max', type=float, default=8.0)
        ap.add_argument('--depth-noise', type=float, default=0.005)
        ap.add_argument('--ir-noise', type=float, default=2.0)
        ap.add_argument('--pivot', type=pair, default=[0.0, 0.0, 0.0])
        ap.add_argument('--cam-offset', type=pair, default=[0.0, 0.0, 0.0])
        ap.add_argument('--servo-function', type=float, default=33.0)
        ap.add_argument('--servo-scale', type=float, default=90.0)
        ap.add_argument('--servo-rate', type=float, default=200.0)
        ap.add_argument('--pos-noise', type=float, default=0.02)
        ap.add_argument('--vel-noise', type=float, default=0.02)
        ap.add_argument('--yaw-noise', type=float, default=0.005)
        ap.add_argument('--delay', type=float, default=0.02)
        ap.add_argument('--tau', type=float, default=0.25)
        ap.add_argument('--amax', type=float, default=3.0)
        ap.add_argument('--climb-rate', type=float, default=1.5)
        ap.add_argument('--land-rate', type=float, default=0.7)
        ap.add_argument('--yaw-rate-max', type=float, default=1.5)
        ap.add_argument('--offboard-loss', type=float, default=1.0)
        ap.add_argument('--battery', type=float, default=0.9)
        ap.add_argument('--no-preflight', dest='preflight_ok', action='store_false')
        ap.add_argument('--land-switch-after', type=float, default=3.0)
        ap.add_argument('--no-rc', action='store_true')
        ap.add_argument('--drone-radius', type=float, default=0.25)
        ap.add_argument('--clr-min', type=float, default=0.5)
        ap.add_argument('--land-dxy', type=float, default=0.35)
        ap.add_argument('--land-vz', type=float, default=0.6)
        ap.add_argument('--end-hold', type=float, default=3.0)
        ap.add_argument('--odom-topic', default='/fmu/out/vehicle_odometry')
        ap.add_argument('--local-topic', default='/fmu/out/vehicle_local_position')
        ap.add_argument('--status-topic', default='/fmu/out/vehicle_status_v1')
        ap.add_argument('--battery-topic', default='/fmu/out/battery_status')
        ap.add_argument('--land-topic', default='/fmu/out/vehicle_land_detected')
        ap.add_argument('--gps-topic', default='/fmu/out/vehicle_gps_position')
        ap.add_argument('--ir-topic', default='/camera/infra1/image_rect_raw')
        ap.add_argument('--depth-topic', default='/depth_camera')
        ap.add_argument('--info-topic', default='/camera/infra1/camera_info')
        a = ap.parse_args()

        rclpy.init()
        probe = rclpy.create_node('sim_hw_probe')
        for _ in range(20):
            rclpy.spin_once(probe, timeout_sec=0.05)
        busy = [t for t in (a.status_topic, a.odom_topic) if probe.count_publishers(t) > 0]
        probe.destroy_node()
        if busy:
            print('PX4 that dang publish', busy, ', tu choi chay mo phong.')
            rclpy.shutdown()
            sys.exit(1)

        sc = SCENARIOS[a.scenario]
        node = Sim(a, build_obstacles(a, sc), sc)
        ex = MultiThreadedExecutor(num_threads=4)
        ex.add_node(node)
        th = threading.Thread(target=ex.spin, daemon=True)
        node.t0 = time.monotonic()
        th.start()
        print(f'Mo phong hardware: kich ban={a.scenario} seed={a.seed} start={a.start} pad={a.pad}', flush=True)
        try:
            while rclpy.ok() and node.t() < a.duration and not node.done:
                time.sleep(0.1)
        except KeyboardInterrupt:
            pass
        ex.shutdown()
        node.summary()
        node.destroy_node()
        rclpy.shutdown()

    if __name__ == '__main__':
        main()