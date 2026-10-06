"""Mô hình mô phỏng vòng kín cho test tích hợp FSM.

MissionSim chạy trong vòng upstream 10 Hz của Harness và đóng vai:
  - input_state_cache : đóng gói snapshot (marker, align_error, altitude, delta_h, yaw_rate, ...)
  - IBVS              : quét yaw khi planner/mode = SEARCH, phanh yaw khi thấy marker
  - planner           : align_error hội tụ về 0 khi mode là FOLLOW/APPROACH/LAND; `planner_alive`
                        = False -> snapshot.planner_timeout = True
  - plant độ cao      : altitude giảm theo /cmd/vertical_descent_rate mới nhất do FSM publish
                        (lệnh cũ hơn `cmd_max_age` coi như bằng 0)

Nó đọc lệnh FSM publish (planner/mode, cmd/vertical_descent_rate) nên đây là vòng kín thật:
FSM sai lệnh -> plant phản ứng sai -> snapshot sai -> test bắt được.
"""
from __future__ import annotations

import time

from fsm_harness import APPROACH, FOLLOW, LAND, SEARCH


class MissionSim:
    def __init__(self, h, *, alt0=2.0, platform=0.1, sweep_rate=0.4, yaw_brake="instant",
                 marker_from=1.0, align0=0.6, cmd_max_age=0.5):
        self.h = h
        self.t = 0.0
        self.alt = alt0
        self.platform = platform
        self.sweep = sweep_rate
        self.yaw_brake = yaw_brake          # "instant" | "slow" (giảm 20%/chu kỳ)
        self.marker_from = marker_from      # thời điểm sim (s) marker bắt đầu xuất hiện
        self.align = align0
        self.yaw_rate = sweep_rate
        self.cmd_max_age = cmd_max_age
        self.force_marker = None            # None: theo lịch; True/False: ép
        self.planner_alive = True
        self.log = []                       # [(t_monotonic, snapshot dict)]
        h.set_model(self)

    def marker_visible(self) -> bool:
        if self.force_marker is not None:
            return self.force_marker
        return self.t >= self.marker_from

    def __call__(self, dt, h):  # gọi trong vòng upstream, đang giữ h.lock
        self.t += dt
        last_mode = h.rec.last("planner_mode")
        mode = None if last_mode is None else last_mode[1]
        vis = self.marker_visible()

        # IBVS: quét ở SEARCH, phanh khi thấy marker, đứng yên ở các mode khác
        if vis:
            if self.yaw_brake == "instant":
                self.yaw_rate = 0.0
            else:
                self.yaw_rate *= 0.8
                if abs(self.yaw_rate) < 0.005:
                    self.yaw_rate = 0.0
        elif mode in (None, SEARCH):
            self.yaw_rate = self.sweep
        else:
            self.yaw_rate = 0.0

        # planner: căn chỉnh hội tụ khi đang bám marker
        if vis and mode in (FOLLOW, APPROACH, LAND):
            self.align *= 0.95

        # plant: hạ theo lệnh FSM
        rate = h.fresh("vrate", self.cmd_max_age)
        self.alt = max(self.platform, self.alt - (rate or 0.0) * dt)

        snap = dict(
            valid=True, marker_detected=vis, align_error=self.align, altitude=self.alt,
            delta_h=max(self.alt - self.platform, 0.0), d_horiz=0.5 if vis else 0.0,
            yaw_rate=self.yaw_rate, touchdown=self.alt <= self.platform + 0.02,
            planner_timeout=not self.planner_alive,
        )
        h.snap.update(snap)
        self.log.append((time.monotonic(), snap))