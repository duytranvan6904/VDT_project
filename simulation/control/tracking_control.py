"""Pure control helpers shared by the tracking pipeline and regression tests."""

from __future__ import annotations

import math

import numpy as np


def compose_follow_velocity(
    guidance_xy,
    target_velocity_ff_xy,
    alignment_scale: float,
    max_speed: float,
) -> np.ndarray:
    """Scale obstacle guidance while preserving target-motion feedforward.

    Both vectors are in world-frame XY.  ``alignment_scale`` is applied only
    to APF guidance; the target velocity feedforward remains active so yaw
    alignment cannot make the vehicle fall behind a moving target.
    """
    guidance = np.asarray(guidance_xy, dtype=float)
    feedforward = np.asarray(target_velocity_ff_xy, dtype=float)
    if guidance.shape != (2,) or feedforward.shape != (2,):
        raise ValueError('guidance_xy and target_velocity_ff_xy must be 2-vectors')

    scale = float(np.clip(alignment_scale, 0.0, 1.0))
    speed_limit = max(0.0, float(max_speed))
    velocity = scale * guidance + feedforward
    speed = float(np.linalg.norm(velocity))
    if speed > speed_limit and speed > 1e-9:
        velocity *= speed_limit / speed
    return velocity


def target_velocity_feedforward(target_velocity_xy, min_speed: float = 0.20) -> np.ndarray:
    """Keep target-motion compensation active even when APF is at its goal."""
    velocity = np.asarray(target_velocity_xy, dtype=float)
    if velocity.shape != (2,):
        raise ValueError('target_velocity_xy must be a 2-vector')
    if float(np.linalg.norm(velocity)) < max(0.0, float(min_speed)):
        return np.zeros(2, dtype=float)
    return velocity.copy()


class TargetMotionGate:
    """Require persistent EKF motion before FOLLOW uses velocity feed-forward."""

    def __init__(
        self,
        enter_speed: float = 0.35,
        exit_speed: float = 0.18,
        enter_confirm_frames: int = 3,
        exit_confirm_frames: int = 4,
    ):
        self.enter_speed = max(0.0, float(enter_speed))
        self.exit_speed = max(0.0, float(exit_speed))
        self.enter_confirm_frames = max(1, int(enter_confirm_frames))
        self.exit_confirm_frames = max(1, int(exit_confirm_frames))
        self.active = False
        self._enter_count = 0
        self._exit_count = 0

    def reset(self) -> None:
        self.active = False
        self._enter_count = 0
        self._exit_count = 0

    def update(self, velocity_xy) -> np.ndarray:
        velocity = np.asarray(velocity_xy, dtype=float)
        if velocity.shape != (2,) or not np.all(np.isfinite(velocity)):
            self.reset()
            return np.zeros(2, dtype=float)

        speed = float(np.linalg.norm(velocity))
        if self.active:
            self._enter_count = 0
            if speed <= self.exit_speed:
                self._exit_count += 1
                if self._exit_count >= self.exit_confirm_frames:
                    self.reset()
            else:
                self._exit_count = 0
        else:
            self._exit_count = 0
            if speed >= self.enter_speed:
                self._enter_count += 1
                if self._enter_count >= self.enter_confirm_frames:
                    self.active = True
                    self._enter_count = 0
            else:
                self._enter_count = 0

        return velocity.copy() if self.active else np.zeros(2, dtype=float)


def reacquire_candidate_is_valid(
    *,
    now: float,
    reacquire_started: float,
    last_bbox_time: float,
    bbox_timeout: float,
    bbox_in_roi: bool,
    detected: bool,
    tracking_mode: str,
    yaw_rate: float,
    max_yaw_rate: float,
    detection_recent: bool,
) -> bool:
    """Accept only a fresh post-trigger image and a settled live EKF lock."""
    bbox_age = now - last_bbox_time
    bbox_fresh_after_trigger = (
        last_bbox_time >= reacquire_started
        and 0.0 <= bbox_age <= bbox_timeout
    )
    return bool(
        detected
        and tracking_mode == 'TRACKING'
        and bbox_fresh_after_trigger
        and bbox_in_roi
        and abs(yaw_rate) <= max_yaw_rate
        and detection_recent
    )


def reacquire_should_resume_search(
    *, detected: bool, now: float, last_detection_time: float, lost_timeout: float,
) -> bool:
    """Resume the sweep only after a real detector loss, never a hold timeout."""
    if detected:
        return False
    if last_detection_time <= 0.0:
        return True
    return now - last_detection_time > max(0.0, lost_timeout)


def update_search_pitch_command(
    *,
    current_pitch: float,
    pixel_v: float | None,
    image_center_v: float,
    focal_y: float,
    pixel_gain: float,
    trim_gain: float,
    dt: float,
    pitch_rate_limit: float,
    pitch_limits: tuple[float, float],
    deadband_px: float = 6.0,
) -> float:
    """Center a first reacquired marker vertically without restarting yaw sweep."""
    if pixel_v is None or not math.isfinite(float(pixel_v)):
        return float(current_pitch)
    fy = max(1.0, float(focal_y))
    error = float(pixel_v) - float(image_center_v)
    if abs(error) <= max(0.0, float(deadband_px)):
        return float(current_pitch)
    effective_error = error - math.copysign(deadband_px, error)
    target = float(current_pitch) - float(pixel_gain) * (
        effective_error / fy
    ) * float(trim_gain)
    target = float(np.clip(target, float(pitch_limits[0]), float(pitch_limits[1])))
    max_step = max(0.0, float(pitch_rate_limit)) * max(0.0, float(dt))
    delta = float(np.clip(target - float(current_pitch), -max_step, max_step))
    return float(np.clip(
        float(current_pitch) + delta,
        float(pitch_limits[0]),
        float(pitch_limits[1]),
    ))


def target_bearing_rate(
    dx: float,
    dy: float,
    vx: float,
    vy: float,
    *,
    min_range: float = 1.0,
) -> float:
    """Return the target's horizontal line-of-sight angular rate (rad/s)."""
    range_sq = dx * dx + dy * dy
    if range_sq < min_range * min_range:
        return 0.0
    return (-dy * vx + dx * vy) / range_sq


def _wrap_angle(angle: float) -> float:
    return (float(angle) + math.pi) % (2.0 * math.pi) - math.pi


def _slew_angle(current: float, target: float, max_step: float) -> float:
    error = _wrap_angle(float(target) - float(current))
    if abs(error) <= max(0.0, float(max_step)):
        return _wrap_angle(float(target))
    return _wrap_angle(float(current) + math.copysign(max_step, error))


def update_tracking_yaw_command(
    *,
    yaw_command: float,
    drone_yaw: float,
    pixel_u: float | None,
    image_center_u: float,
    focal_x: float,
    pixel_gain: float,
    target_delta_xy: tuple[float, float] | None,
    target_velocity_xy: tuple[float, float],
    tracking_mode: str,
    target_state_age: float,
    target_state_timeout: float,
    dt: float,
    yaw_rate_limit: float,
    feedforward_min_speed: float = 0.30,
    feedforward_max_rate: float = 0.15,
    prediction_horizon: float = 0.25,
    deadband_px: float = 12.0,
) -> float:
    """Update the yaw setpoint from a live pixel or a fresh EKF bearing.

    Horizontal pixel error is an angular error in the camera image and is
    independent of gimbal pitch. When images briefly drop, a fresh predicted
    EKF target bearing keeps the vehicle steering toward the same target.
    Target motion is added as angular-rate feedforward, so stationary-target
    estimator noise cannot leave a persistent yaw bias.
    """
    dt = max(0.0, float(dt))
    usable_modes = ('TRACKING', 'PREDICTING', 'PREDICTING_DEGRADED')
    target_is_usable = (
        target_delta_xy is not None
        and tracking_mode in usable_modes
        and 0.0 <= float(target_state_age) <= max(0.0, float(target_state_timeout))
    )

    target_yaw = None
    if pixel_u is not None and focal_x > 0.0:
        pixel_error = float(pixel_u) - float(image_center_u)
        if abs(pixel_error) <= deadband_px:
            pixel_error = 0.0
        else:
            pixel_error -= math.copysign(deadband_px, pixel_error)
        target_yaw = _wrap_angle(
            float(drone_yaw) - float(pixel_gain) * math.atan(pixel_error / float(focal_x))
        )
    elif target_is_usable:
        dx, dy = target_delta_xy
        vx, vy = target_velocity_xy
        predicted_x = float(dx) + float(vx) * max(0.0, float(prediction_horizon))
        predicted_y = float(dy) + float(vy) * max(0.0, float(prediction_horizon))
        if math.hypot(predicted_x, predicted_y) > 0.3:
            target_yaw = math.atan2(predicted_y, predicted_x)

    if target_yaw is None:
        return _wrap_angle(float(yaw_command))

    command = _slew_angle(
        float(yaw_command), target_yaw, max(0.0, float(yaw_rate_limit)) * dt,
    )

    if target_is_usable and target_delta_xy is not None:
        dx, dy = target_delta_xy
        vx, vy = target_velocity_xy
        speed = math.hypot(float(vx), float(vy))
        if speed > max(0.0, float(feedforward_min_speed)):
            bearing_rate = target_bearing_rate(float(dx), float(dy), float(vx), float(vy))
            bounded_rate = max(
                -abs(float(feedforward_max_rate)),
                min(abs(float(feedforward_max_rate)), bearing_rate),
            )
            command = _wrap_angle(command + bounded_rate * dt)

    return command


def add_bearing_lead(yaw_target: float, bearing_rate: float, lead_time: float) -> float:
    """Lead a yaw target by a bounded time horizon without integrating bias."""
    angle = yaw_target + bearing_rate * max(0.0, lead_time)
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def landing_pitch_from_geometry(
    drone_altitude: float,
    target_altitude: float,
    horizontal_range: float,
) -> float:
    """Aim the camera at the landing target without forcing a nadir snap."""
    dz = max(0.1, float(drone_altitude) - float(target_altitude))
    return -math.atan2(dz, max(0.2, float(horizontal_range)))


def choose_landing_yaw(
    *,
    tracking_valid: bool,
    ibvs_yaw: float,
    drone_yaw: float,
    held_yaw: float,
    hold_initialized: bool,
) -> tuple[float, float, bool]:
    """Latch one heading for approach and hold it through landing."""
    if hold_initialized:
        yaw = _wrap_angle(float(held_yaw))
        return yaw, yaw, True
    if tracking_valid:
        yaw = _wrap_angle(float(ibvs_yaw))
        return yaw, yaw, True
    yaw = _wrap_angle(float(drone_yaw))
    return yaw, yaw, True
