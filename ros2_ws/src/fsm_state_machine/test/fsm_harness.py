"""Hạ tầng dùng chung cho test tích hợp FSM (black-box qua topic ROS 2).

Harness làm 4 việc:
  1. Chạy `fsm_node` thật trong subprocess (tham số tùy chỉnh theo từng test).
  2. Đóng vai các node upstream: input_state_cache (snapshot), rc_parser (rc/fsm_input),
     safety_monitor (safety/force_land), kill_switch (system/killed, transient_local).
  3. Ghi lại mọi topic FSM publish (kèm timestamp) để test phân tích sau.
  4. Cung cấp helper `drive_to(state)` và `common_violations()` dùng lại ở mọi file test.

Kiểu message của các topic ĐẦU RA được suy ra từ graph ROS lúc chạy, nên không phụ thuộc
tên package. Kiểu message của ĐẦU VÀO (InputSnapshot, RcFsmInput) có thể ép bằng biến môi trường
FSM_SNAPSHOT_TYPE / FSM_RC_TYPE.
"""
from __future__ import annotations

import functools
import itertools
import math
import os
import shlex
import signal
import subprocess
import tempfile
import threading
import time

from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rosidl_runtime_py.utilities import get_message
from std_msgs.msg import Bool

SEARCH, FOLLOW, APPROACH, LAND, COMPLETE = 0, 1, 2, 3, 4
STATE_IDS = ["SEARCH", "FOLLOW", "APPROACH", "LAND", "COMPLETE"]

SNAPSHOT_TOPIC = "/input_cache/snapshot"
RC_TOPIC = "/rc/fsm_input"
FORCE_LAND_TOPIC = "/safety/force_land"
KILLED_TOPIC = "/system/killed"

OUT_TOPICS = {
    "state": "/fsm/state",
    "planner_mode": "/planner/mode",
    "gimbal_state": "/gimbal/state_request",
    "apf_gain": "/planner/apf_gain",
    "align_cmd": "/gimbal/align_error_cmd",
    "vrate": "/cmd/vertical_descent_rate",
    "disarm": "/cmd/disarm_request",
}

FSM_NODE_NAME = os.environ.get("FSM_NODE_NAME", "fsm_node")

DEFAULTS = dict(
    land_entry_height=0.5, align_threshold=0.3, land_entry_cycles=5,
    enter_follow_cycles=10, yaw_settle_rate=0.1, follow_lost_timeout=2.5,
    approach_lost_timeout=1.5, approach_descent_rate=0.3, land_descent_rate=0.4,
    land_requires_rearm=True, ignore_planner_timeout=False,
)

ALLOWED = {
    SEARCH: {FOLLOW, LAND},
    FOLLOW: {SEARCH, APPROACH, LAND},
    APPROACH: {FOLLOW, SEARCH, LAND},
    LAND: {COMPLETE},
    COMPLETE: set(),
}

SNAPSHOT_CANDIDATES = ("input_state_cache/msg/InputSnapshot", "vdt_msgs/msg/InputSnapshot")
RC_CANDIDATES = ("rc_parser/msg/RcFsmInput", "vdt_msgs/msg/RcFsmInput")

_counter = itertools.count()


@functools.lru_cache(maxsize=None)
def _resolve(env_name: str, candidates: tuple):
    names = [os.environ[env_name]] if os.environ.get(env_name) else list(candidates)
    errors = []
    for name in names:
        try:
            return name, get_message(name)
        except Exception as exc:  # noqa: BLE001
            errors.append(f"{name}: {exc}")
    raise RuntimeError(
        f"Không import được message ({env_name}). Đã thử:\n  " + "\n  ".join(errors)
        + f"\nĐặt biến môi trường {env_name}=<pkg>/msg/<Tên> cho đúng package."
    )

def snapshot_type():
    return _resolve("FSM_SNAPSHOT_TYPE", SNAPSHOT_CANDIDATES)

def rc_type():
    return _resolve("FSM_RC_TYPE", RC_CANDIDATES)

def build_msg(cls, values: dict):
    msg = cls()
    fields = msg.get_fields_and_field_types()
    for key, val in values.items():
        if key not in fields:
            raise AttributeError(f"{cls.__name__} không có field '{key}' (có: {sorted(fields)})")
        ftype = fields[key]
        if ftype in ("float", "double"):
            val = float(val)
        elif ftype == "boolean":
            val = bool(val)
        setattr(msg, key, val)
    return msg

def msg_value(msg):
    """Giá trị của message một-field (std_msgs/*): field đầu tiên, thường là `.data`."""
    first = next(iter(msg.get_fields_and_field_types()))
    val = getattr(msg, first)
    if isinstance(val, (bytes, bytearray)):
        val = val[0]
    return val

def healthy_snapshot(**override) -> dict:
    snap = dict(valid=True, marker_detected=True, align_error=0.1, altitude=2.0,
                delta_h=1.0, d_horiz=0.5, yaw_rate=0.0, touchdown=False,
                planner_timeout=False)
    snap.update(override)
    return snap

def approx(a, b, tol=1e-3) -> bool:
    return a is not None and b is not None and abs(float(a) - float(b)) <= tol

class Recorder:
    def __init__(self):
        self._lock = threading.Lock()
        self._data = {k: [] for k in OUT_TOPICS}

    def add(self, key, t, value):
        with self._lock:
            self._data[key].append((t, value))

    def get(self, key):
        with self._lock:
            return list(self._data[key])

    def last(self, key):
        with self._lock:
            d = self._data[key]
            return d[-1] if d else None

class Harness:
    def __init__(self, params=None, *, pre_kill=False, snapshots=True):
        self.params = dict(params or {})
        self.p = dict(DEFAULTS)
        self.p.update(self.params)
        self.pre_kill = pre_kill
        self.rec = Recorder()
        self.snap = healthy_snapshot(marker_detected=False)
        self.rc = {"land_switch": False, "kill_switch": False}
        self.force_land = False
        self.snapshot_enabled = snapshots
        self.rate_hz = 10.0
        self.model = None
        self.lock = threading.RLock()
        self.out_types: dict = {}
        self.missing_topics: set = set()
        self.upstream_error = None
        self.node = self.executor = self.proc = self.log = None
        self._exec_thread = self._up_thread = None
        self._stop = threading.Event()
        self.t_kill = None

    def start(self, wait_ready=None):
        if wait_ready is None:
            wait_ready = not self.pre_kill
        self.node = Node(f"fsm_it_{os.getpid()}_{next(_counter)}")
        qos = QoSProfile(depth=10)
        self._snap_cls = snapshot_type()[1]
        self._rc_cls = rc_type()[1]
        self.snap_pub = self.node.create_publisher(self._snap_cls, SNAPSHOT_TOPIC, qos)
        self.rc_pub = self.node.create_publisher(self._rc_cls, RC_TOPIC, qos)
        self.force_pub = self.node.create_publisher(Bool, FORCE_LAND_TOPIC, qos)
        self.killed_pub = self.node.create_publisher(
            Bool, KILLED_TOPIC,
            QoSProfile(depth=1, history=HistoryPolicy.KEEP_LAST,
                       reliability=ReliabilityPolicy.RELIABLE,
                       durability=DurabilityPolicy.TRANSIENT_LOCAL))
        if self.pre_kill:  # kill đã được latch trước khi FSM khởi động
            self.killed_pub.publish(Bool(data=True))
            self.t_kill = time.monotonic()
        self._launch()
        self._attach_outputs()
        self.executor = SingleThreadedExecutor()
        self.executor.add_node(self.node)
        self._exec_thread = threading.Thread(target=self.executor.spin, daemon=True)
        self._exec_thread.start()
        self.t_attach = time.monotonic()
        self._up_thread = threading.Thread(target=self._upstream_loop, daemon=True)
        self._up_thread.start()
        if wait_ready and not self.wait_for(lambda: self.rec.last("state") is not None, 8.0):
            raise RuntimeError("Không nhận được /fsm/state từ fsm_node\n" + self.log_tail())

    def stop(self):
        self._stop.set()
        if self._up_thread:
            self._up_thread.join(2.0)
        if self.proc and self.proc.poll() is None:
            try:
                os.killpg(self.proc.pid, signal.SIGINT)
                self.proc.wait(5)
            except (ProcessLookupError, subprocess.TimeoutExpired):
                try:
                    os.killpg(self.proc.pid, signal.SIGKILL)
                except ProcessLookupError:
                    pass
                self.proc.wait()
        if self.executor:
            self.executor.shutdown()
        if self._exec_thread:
            self._exec_thread.join(2.0)
        if self.node:
            self.node.destroy_node()
        if self.log:
            self.log.close()

    def _launch(self):
        cmd = shlex.split(os.environ.get("FSM_NODE_CMD", "ros2 run fsm_state_machine fsm_node"))
        cmd += ["--ros-args", "-p", "debug_enabled:=true"]
        for key, val in self.params.items():
            sval = str(val).lower() if isinstance(val, bool) else repr(val) if isinstance(val, float) else str(val)
            cmd += ["-p", f"{key}:={sval}"]
        self.log = tempfile.NamedTemporaryFile("w+", suffix="_fsm.log", delete=False)
        self.proc = subprocess.Popen(cmd, stdout=self.log, stderr=subprocess.STDOUT,
                                     start_new_session=True)

    def _attach_outputs(self, timeout=20.0):
        be = QoSProfile(depth=100, reliability=ReliabilityPolicy.BEST_EFFORT)
        pending = set(OUT_TOPICS)
        end = time.monotonic() + timeout
        while pending and time.monotonic() < end:
            if self.proc.poll() is not None:
                raise RuntimeError(f"fsm_node thoát sớm (rc={self.proc.returncode})\n" + self.log_tail())
            known = dict(self.node.get_topic_names_and_types())
            for key in list(pending):
                types = known.get(OUT_TOPICS[key])
                if types:
                    cls = get_message(types[0])
                    self.node.create_subscription(
                        cls, OUT_TOPICS[key],
                        lambda m, k=key: self.rec.add(k, time.monotonic(), msg_value(m)), be)
                    self.out_types[key] = types[0]
                    pending.discard(key)
            if pending and "state" in self.out_types and time.monotonic() > end - timeout + 6.0:
                break  # state đã có, các topic còn lại sau 6 s coi là thiếu
            if pending:
                time.sleep(0.2)
        self.missing_topics = pending
        if "state" in pending:
            raise RuntimeError("Không thấy topic /fsm/state trong graph\n" + self.log_tail())

    def _build(self, cls, values):
        return build_msg(cls, values)

    def _upstream_loop(self):
        period_of = lambda: 1.0 / self.rate_hz  # noqa: E731
        nxt = time.monotonic()
        while not self._stop.is_set():
            period = period_of()
            nxt += period
            try:
                with self.lock:
                    if self.model:
                        self.model(period, self)
                    snap, rc = dict(self.snap), dict(self.rc)
                    enabled, fl = self.snapshot_enabled, self.force_land
                if enabled:
                    self.snap_pub.publish(self._build(self._snap_cls, snap))
                self.rc_pub.publish(self._build(self._rc_cls, rc))
                self.force_pub.publish(Bool(data=fl))
            except Exception as exc:  # noqa: BLE001
                self.upstream_error = exc
            delay = nxt - time.monotonic()
            if delay > 0:
                self._stop.wait(delay)
            else:
                nxt = time.monotonic()

    def set_snapshot(self, **kw):
        with self.lock:
            self.snap.update(kw)

    def set_rc(self, **kw):
        with self.lock:
            self.rc.update(kw)

    def set_force_land(self, value: bool):
        with self.lock:
            self.force_land = bool(value)

    def set_model(self, fn):
        with self.lock:
            self.model = fn

    def set_rate(self, hz):
        self.rate_hz = float(hz)

    def pause_snapshots(self):
        self.snapshot_enabled = False

    def resume_snapshots(self):
        self.snapshot_enabled = True

    def kill(self):
        self.killed_pub.publish(Bool(data=True))
        self.t_kill = time.monotonic()
        return self.t_kill

    def state(self):
        v = self.rec.last("state")
        return None if v is None else v[1]

    def fresh(self, key, max_age):
        v = self.rec.last(key)
        if v and time.monotonic() - v[0] <= max_age:
            return v[1]
        return None

    def window(self, key, t0, t1=None):
        t1 = time.monotonic() if t1 is None else t1
        return [v for t, v in self.rec.get(key) if t0 <= t <= t1]

    def window_all(self, t0, t1=None):
        return {k: self.window(k, t0, t1) for k in OUT_TOPICS}

    def timeline(self):
        out = []
        for t, s in self.rec.get("state"):
            if not out or out[-1][1] != s:
                out.append((t, s))
        return out

    def states_in(self, t0, t1=None):
        t1 = time.monotonic() if t1 is None else t1
        seen, before = set(), None
        for t, s in self.rec.get("state"):
            if t < t0:
                before = s
            elif t <= t1:
                seen.add(s)
        if before is not None:
            seen.add(before)
        return seen

    def wait_for(self, pred, timeout, period=0.02):
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            if pred():
                return True
            time.sleep(period)
        return pred()

    def wait_state(self, target, timeout):
        return self.wait_for(lambda: self.state() == target, timeout)

    def log_tail(self, n=1500):
        try:
            self.log.flush()
            self.log.seek(0)
            return "--- fsm_node log (cuối) ---\n" + self.log.read()[-n:]
        except Exception:  
            return ""

    def diag(self):
        tl = " -> ".join(f"{STATE_IDS[s]}@{t - self.t_attach:.1f}s" for t, s in self.timeline())
        err = f"\nupstream_error={self.upstream_error!r}" if self.upstream_error else ""
        return f"timeline: {tl}{err}\n{self.log_tail(800)}"

def drive_to(h: Harness, target: int):
    if target == SEARCH:
        h.set_snapshot(**healthy_snapshot(marker_detected=False))
        assert h.wait_state(SEARCH, 2.0), h.diag()
        return
    h.set_snapshot(**healthy_snapshot())
    assert h.wait_state(FOLLOW, 4.0), "không vào được FOLLOW\n" + h.diag()
    if target == FOLLOW:
        return
    h.set_rc(land_switch=True)
    assert h.wait_state(APPROACH, 3.0), "không vào được APPROACH\n" + h.diag()
    if target == APPROACH:
        return
    h.set_snapshot(altitude=0.4, delta_h=0.3, align_error=0.15)
    assert h.wait_state(LAND, 3.0), "không vào được LAND\n" + h.diag()
    if target == LAND:
        return
    h.set_snapshot(touchdown=True, altitude=0.05, delta_h=0.0, align_error=0.0)
    assert h.wait_state(COMPLETE, 3.0), "không vào được COMPLETE\n" + h.diag()

def common_violations(h: Harness, slack=0.15) -> list:
    v = []
    tl = h.timeline()
    for (_, a), (_, b) in zip(tl, tl[1:]):
        if b not in ALLOWED.get(a, set()):
            v.append(f"chuyển trạng thái trái luật: {a} -> {b}")
    for _, s in h.rec.get("state"):
        if s not in (SEARCH, FOLLOW, APPROACH, LAND, COMPLETE):
            v.append(f"state ngoài miền: {s}")
    rates = (0.0, h.p["approach_descent_rate"], h.p["land_descent_rate"])
    for t, x in h.rec.get("vrate"):
        if not math.isfinite(x) or not any(approx(x, r) for r in rates):
            v.append(f"vertical_descent_rate lạ: {x} @{t - h.t_attach:.2f}s")
        elif x > 0 and not (h.states_in(t - slack, t + slack) & {APPROACH, LAND}):
            v.append(f"hạ độ cao {x} ngoài APPROACH/LAND @{t - h.t_attach:.2f}s")
    for key in ("planner_mode", "gimbal_state"):
        for t, x in h.rec.get(key):
            if x not in h.states_in(t - slack, t + slack):
                v.append(f"{key}={x} không khớp fsm/state @{t - h.t_attach:.2f}s")
    for t, x in h.rec.get("apf_gain"):
        if not any(approx(x, g) for g in (0.0, 0.5, 1.0)):
            v.append(f"apf_gain lạ: {x}")
    for t, x in h.rec.get("align_cmd"):
        if not math.isfinite(x):
            v.append(f"align_error_cmd không hữu hạn: {x}")
    for t, x in h.rec.get("disarm"):
        if x is True and COMPLETE not in h.states_in(t - slack, t + slack):
            v.append(f"disarm_request=true ngoài COMPLETE @{t - h.t_attach:.2f}s")
    return v