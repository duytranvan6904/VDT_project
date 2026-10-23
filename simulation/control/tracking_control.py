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


def follow_motion_yaw(
    velocity_xy,
    fallback_yaw: float,
    min_speed: float = 0.10,
) -> float:
    """Face along the commanded FOLLOW motion, falling back to visual yaw at rest."""
    velocity = np.asarray(velocity_xy, dtype=float)
    if velocity.shape != (2,):
        raise ValueError('velocity_xy must be a 2-vector')
    if not np.all(np.isfinite(velocity)):
        return float(fallback_yaw)
    if float(np.linalg.norm(velocity)) < max(0.0, float(min_speed)):
        return float(fallback_yaw)
    return math.atan2(float(velocity[1]), float(velocity[0]))


def select_follow_yaw_reference(
    ibvs_yaw: float,
    target_velocity_ff_xy,
    *,
    min_target_speed: float = 0.30,
) -> float:
    """Use pad course only after the motion gate supplies clear velocity.

    APF guidance points toward the stand-off goal and can reverse or rotate as
    the vehicle corrects position around a stationary pad. It is therefore
    unsuitable as a body-heading reference. For a stationary or uncertain pad,
    retain the visual/world-bearing IBVS yaw command instead.
    """
    velocity = np.asarray(target_velocity_ff_xy, dtype=float)
    if velocity.shape != (2,) or not np.all(np.isfinite(velocity)):
        return float(ibvs_yaw)
    if float(np.linalg.norm(velocity)) < max(0.0, float(min_target_speed)):
        return float(ibvs_yaw)
    return math.atan2(float(velocity[1]), float(velocity[0]))


def slew_yaw_reference(
    current_yaw: float,
    target_yaw: float,
    max_rate: float,
    dt: float,
) -> float:
    """Bound course-reference changes so noisy velocity estimates cannot jerk yaw."""
    current = float(current_yaw)
    target = float(target_yaw)
    if not math.isfinite(current) or not math.isfinite(target):
        return current
    max_step = max(0.0, float(max_rate)) * max(0.0, float(dt))
    error = (target - current + math.pi) % (2.0 * math.pi) - math.pi
    return (current + float(np.clip(error, -max_step, max_step)) + math.pi) % (
        2.0 * math.pi
    ) - math.pi


def forward_heading_guard(
    velocity_xy,
    body_yaw: float,
    max_course_error: float = math.radians(85.0),
) -> np.ndarray:
    """Suppress XY translation until the body is pointed into the travel sector.

    Velocity commands are world-frame. If the commanded course is behind the
    vehicle, holding position while yaw slews is safer than translating in
    reverse with the camera still facing the target.
    """
    velocity = np.asarray(velocity_xy, dtype=float)
    if velocity.shape != (2,):
        raise ValueError('velocity_xy must be a 2-vector')
    if not np.all(np.isfinite(velocity)):
        return np.zeros(2, dtype=float)
    if float(np.linalg.norm(velocity)) < 1e-6:
        return velocity.copy()
    course_yaw = math.atan2(float(velocity[1]), float(velocity[0]))
    error = (course_yaw - float(body_yaw) + math.pi) % (2.0 * math.pi) - math.pi
    limit = min(math.pi / 2.0, max(0.0, float(max_course_error)))
    if abs(error) > limit:
        return np.zeros(2, dtype=float)
    return velocity.copy()


def target_velocity_feedforward(target_velocity_xy, min_speed: float = 0.20) -> np.ndarray:
    """Keep target-motion compensation active even when APF is at its goal."""
    velocity = np.asarray(target_velocity_xy, dtype=float)
    if velocity.shape != (2,):
        raise ValueError('target_velocity_xy must be a 2-vector')
    if float(np.linalg.norm(velocity)) < max(0.0, float(min_speed)):
        return np.zeros(2, dtype=float)
    return velocity.copy()


class TargetMotionGate:
    """Require persistent EKF motion before FOLLOW uses velocity feed-forward.

    On exit, the gate linearly ramps the feedforward velocity from the current
    value to zero over ``exit_confirm_frames`` frames (velocity tapering).
    This prevents the drone from overshooting when the target suddenly stops,
    because the feedforward contribution smoothly decreases to zero instead of
    cutting off after a hard delay.
    """

    def __init__(
        self,
        enter_speed: float = 0.38,
        exit_speed: float = 0.18,
        enter_confirm_frames: int = 15,
        exit_confirm_frames: int = 8,
    ):
        self.enter_speed = max(0.0, float(enter_speed))
        self.exit_speed = max(0.0, float(exit_speed))
        self.enter_confirm_frames = max(1, int(enter_confirm_frames))
        self.exit_confirm_frames = max(1, int(exit_confirm_frames))
        self.active = False
        self._enter_count = 0
        self._exit_count = 0
        # Tapering: scale ramps 1.0 → 0.0 during exit confirmation window
        self._taper_scale: float = 1.0

    def reset(self) -> None:
        self.active = False
        self._enter_count = 0
        self._exit_count = 0
        self._taper_scale = 1.0

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
                # Linear taper: ramp feedforward scale from 1.0 to 0.0
                # over the exit confirmation window before fully resetting.
                self._taper_scale = max(
                    0.0,
                    1.0 - self._exit_count / max(1, self.exit_confirm_frames),
                )
                if self._exit_count >= self.exit_confirm_frames:
                    self.reset()
            else:
                self._exit_count = 0
                self._taper_scale = 1.0
        else:
            self._exit_count = 0
            self._taper_scale = 1.0
            if speed >= self.enter_speed:
                self._enter_count += 1
                if self._enter_count >= self.enter_confirm_frames:
                    self.active = True
                    self._enter_count = 0
            else:
                self._enter_count = 0

        if self.active:
            return velocity.copy() * self._taper_scale
        return np.zeros(2, dtype=float)


class FollowYawReferenceSelector:
    """Choose a stable FOLLOW yaw source and hold it through short dropouts.

    EKF velocity can contain short large spikes when a stationary pad's PnP
    position jumps. Motion must survive a low-pass filter and a timed,
    direction-consistent confirmation before it can take yaw away from IBVS.
    """

    def __init__(
        self,
        enter_speed: float = 0.35,
        exit_speed: float = 0.18,
        enter_confirm_duration_s: float = 0.75,
        exit_confirm_duration_s: float = 0.50,
        velocity_filter_time_constant_s: float = 0.25,
        course_consistency_deg: float = 40.0,
    ):
        self.enter_speed = max(0.0, float(enter_speed))
        self.exit_speed = max(0.0, float(exit_speed))
        self.enter_confirm_duration = max(0.0, float(enter_confirm_duration_s))
        self.exit_confirm_duration = max(0.0, float(exit_confirm_duration_s))
        self.velocity_filter_time_constant = max(
            1e-3, float(velocity_filter_time_constant_s),
        )
        self.course_consistency = math.radians(max(0.0, float(course_consistency_deg)))
        self.moving = False
        self.target_course_yaw = 0.0
        self.filtered_velocity = np.zeros(2, dtype=float)
        self.last_sample_time: float | None = None
        self.candidate_course_yaw: float | None = None
        self.entry_elapsed = 0.0
        self.exit_elapsed = 0.0

    @property
    def following_target_course(self) -> bool:
        return self.moving

    def reset(self, initial_yaw: float) -> None:
        self.moving = False
        self.target_course_yaw = float(initial_yaw)
        self.filtered_velocity[:] = 0.0
        self.last_sample_time = None
        self.candidate_course_yaw = None
        self.entry_elapsed = 0.0
        self.exit_elapsed = 0.0

    def update(
        self,
        *,
        ibvs_yaw: float,
        target_velocity_xy,
        current_yaw: float,
        previous_reference_yaw: float,
        tracking_valid: bool,
        tracking_mode: str,
        target_state_fresh: bool,
        target_sample_time: float,
    ) -> tuple[float, str]:
        velocity = np.asarray(target_velocity_xy, dtype=float)
        velocity_valid = velocity.shape == (2,) and np.all(np.isfinite(velocity))
        may_update_motion = bool(
            tracking_valid
            and tracking_mode == 'TRACKING'
            and target_state_fresh
            and velocity_valid
            and math.isfinite(float(target_sample_time))
        )
        sample_time = float(target_sample_time)
        is_new_sample = (
            self.last_sample_time is None
            or sample_time > self.last_sample_time + 1e-4
        )
        if may_update_motion and is_new_sample:
            if self.last_sample_time is None:
                dt = 0.0
            else:
                dt = min(0.20, max(0.0, sample_time - self.last_sample_time))
            self.last_sample_time = sample_time
            alpha = 1.0 - math.exp(-dt / self.velocity_filter_time_constant)
            self.filtered_velocity += alpha * (velocity - self.filtered_velocity)
            speed = float(np.linalg.norm(self.filtered_velocity))

            if self.moving:
                if speed <= self.exit_speed:
                    self.exit_elapsed += dt
                    if self.exit_elapsed >= self.exit_confirm_duration:
                        self.moving = False
                        self.candidate_course_yaw = None
                        self.entry_elapsed = 0.0
                        self.exit_elapsed = 0.0
                else:
                    self.exit_elapsed = 0.0
                    self.target_course_yaw = math.atan2(
                        float(self.filtered_velocity[1]),
                        float(self.filtered_velocity[0]),
                    )
            elif speed >= self.enter_speed:
                candidate_yaw = math.atan2(
                    float(self.filtered_velocity[1]),
                    float(self.filtered_velocity[0]),
                )
                if (
                    self.candidate_course_yaw is None
                    or abs(_wrap_angle(candidate_yaw - self.candidate_course_yaw))
                    > self.course_consistency
                ):
                    self.candidate_course_yaw = candidate_yaw
                    self.entry_elapsed = 0.0
                else:
                    self.candidate_course_yaw = _wrap_angle(
                        self.candidate_course_yaw
                        + 0.25 * _wrap_angle(candidate_yaw - self.candidate_course_yaw)
                    )
                    self.entry_elapsed += dt
                if self.entry_elapsed >= self.enter_confirm_duration:
                    self.moving = True
                    self.exit_elapsed = 0.0
                    self.target_course_yaw = self.candidate_course_yaw
            else:
                self.candidate_course_yaw = None
                self.entry_elapsed = 0.0

        if not tracking_valid:
            return float(current_yaw), 'HOLD_LOSS'
        if not target_state_fresh or tracking_mode != 'TRACKING':
            if self.moving:
                return self.target_course_yaw, 'HOLD_PREDICTION'
            return float(previous_reference_yaw), 'HOLD_PREDICTION'
        if self.moving:
            return self.target_course_yaw, 'TARGET_COURSE'
        return float(ibvs_yaw), 'IBVS'


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
    min_bearing_distance: float = 1.0,
) -> float:
    """Update the yaw setpoint from a live pixel or a fresh EKF bearing.

    Horizontal pixel error is an angular error in the camera image and is
    independent of gimbal pitch. When images briefly drop, a fresh predicted
    EKF target bearing keeps the vehicle steering toward the same target.
    Target motion is added as angular-rate feedforward, so stationary-target
    estimator noise cannot leave a persistent yaw bias.
    When the vehicle is close to the target (horizontal range < min_bearing_distance),
    line-of-sight azimuth has an ill-conditioned singularity and must not steer yaw.
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
        if math.hypot(predicted_x, predicted_y) >= float(min_bearing_distance):
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
    """Aim the camera at the landing target based on line-of-sight geometry."""
    dz = max(0.01, float(drone_altitude) - float(target_altitude))
    horiz = max(0.0, float(horizontal_range))
    return -math.atan2(dz, horiz)


def landing_pitch_target(
    *,
    drone_altitude: float,
    target_surface_altitude: float,
    horizontal_range: float,
    target_state_fresh: bool,
    pixel_v: float | None = None,
    image_center_v: float = 240.0,
    focal_y: float = 466.0,
    pixel_gain: float = 0.92,
    pixel_trim_gain: float = 0.30,
) -> float | None:
    """Return a stable landing-camera aim, or ``None`` to hold its last aim.

    The pad is a known ground-plane target, so its measured EKF z is not used
    to steer camera pitch. Tracking dropouts hold the current camera direction;
    the caller must reacquire the target before changing the camera aim again.
    """
    if target_state_fresh:
        target = landing_pitch_from_geometry(
            drone_altitude, target_surface_altitude, horizontal_range,
        )
        if pixel_v is not None and math.isfinite(float(pixel_v)):
            fy = max(1.0, float(focal_y))
            error = float(pixel_v) - float(image_center_v)
            deadband = 6.0
            if abs(error) > deadband:
                effective_error = error - math.copysign(deadband, error)
                target -= (
                    float(pixel_gain)
                    * (effective_error / fy)
                    * float(pixel_trim_gain)
                )
        return float(np.clip(target, math.radians(-90.0), math.radians(-10.0)))

    return None


def choose_landing_yaw(
    *,
    tracking_valid: bool,
    ibvs_yaw: float,
    drone_yaw: float,
    held_yaw: float,
    hold_initialized: bool,
    target_velocity_xy: tuple[float, float] | np.ndarray | None = None,
    align_with_target_motion: bool = False,
    min_target_speed: float = 0.25,
) -> tuple[float, float, bool]:
    """Latch one heading for approach and hold it through landing.

    When ``align_with_target_motion`` is True and target speed exceeds ``min_target_speed``,
    the vehicle aligns its yaw with the target's course angle atan2(vy, vx).
    Otherwise, the heading is latched on the first call (using ibvs_yaw if tracking is valid,
    or drone_yaw) and held steady throughout landing to prevent nadir spinning.
    """
    if align_with_target_motion and target_velocity_xy is not None:
        try:
            vx, vy = float(target_velocity_xy[0]), float(target_velocity_xy[1])
            speed = math.hypot(vx, vy)
            if speed >= max(0.0, float(min_target_speed)):
                course = _wrap_angle(math.atan2(vy, vx))
                return course, course, True
        except (IndexError, TypeError, ValueError):
            pass

    if hold_initialized:
        yaw = _wrap_angle(float(held_yaw))
        return yaw, yaw, True
    if tracking_valid:
        yaw = _wrap_angle(float(ibvs_yaw))
        return yaw, yaw, True
    yaw = _wrap_angle(float(drone_yaw))
    return yaw, yaw, True

