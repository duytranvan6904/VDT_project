#!/usr/bin/env python3
"""Sliding Mode Control (SMC) Guidance for Precision Drone Landing.

Ported from Tuân's Simulink guidance framework (References/Tuan_Simulink/) to ROS 2.
Features:
  1. 3D Sliding Mode Control with power reaching law (S1: range, S2: 45° glide slope, S3: azimuth).
  2. Covariance-Adaptive weighting (ws) and phase transition radius (Rswitch).
  3. Two landing sub-phases:
     - GLIDE_SLOPE (Rxy > Rswitch): 45° synchronized descent and lateral pursuit.
     - FINAL_DESCENT (Rxy <= Rswitch): Closed-loop horizontal centering and gentle vertical touchdown.

Outputs:
  /landing/velocity_cmd     geometry_msgs/Twist (vx, vy, vz in ENU world frame, yaw_rate)
  /landing/sub_phase        std_msgs/String (GLIDE_SLOPE | FINAL_DESCENT)
"""

from __future__ import annotations

import math
import os
import sys
import time
from dataclasses import dataclass
from typing import Optional, Tuple

import numpy as np

try:
    import rclpy
    from diagnostic_msgs.msg import DiagnosticStatus, KeyValue
    from geometry_msgs.msg import Twist
    from nav_msgs.msg import Odometry
    from rclpy.node import Node
    from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
    from std_msgs.msg import Bool, Float64, String
    HAVE_ROS2 = True
except ImportError:
    HAVE_ROS2 = False
    Node = object

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
if REPO_ROOT not in sys.path:
    sys.path.insert(0, REPO_ROOT)


@dataclass
class SMCParams:
    """Parameters for the SMC Precision Landing Guidance Law."""
    ka: float = 0.20            # Range sliding surface gain
    kb: float = 0.60            # Elevation sliding surface gain
    kc: float = 0.40            # Azimuth sliding surface gain
    k1: float = 0.1395          # Reaching law gain 1
    k2: float = 0.1784          # Reaching law gain 2
    k3: float = 0.0442          # Reaching law gain 3
    n: float = 3.0              # Odd numerator power
    m: float = 5.0              # Odd denominator power (power = 3/5 = 0.6)
    theta_des_rad: float = math.pi / 4.0  # 45 deg glide slope (theta_des = 45°)
    zeta_des_rad: float = 0.0   # Desired relative azimuth
    v_max: float = 1.50         # Max velocity magnitude in approach (m/s)
    v_final_xy_max: float = 0.35  # Max horizontal centering velocity in terminal phase (m/s)
    v_descend_fast: float = 0.35  # Descent velocity when altitude > 0.5m (m/s)
    v_descend_touch: float = 0.15 # Touchdown descent velocity when altitude <= 0.3m (m/s)
    rswitch_default: float = 0.80 # Default switching radius (m)
    final_descent_rxy_m: float = 1.00
    final_descent_alt_m: float = 1.50
    final_centering_kp: float = 0.70  # Proportional centering gain in final descent
    glide_centering_kp: float = 0.70  # World-frame target pursuit gain during glide
    target_velocity_ff_gain: float = 0.80  # EKF feed-forward for a moving pad
    rmin: float = 0.05          # Singularity avoidance radius (m)


@dataclass
class SMCResult:
    """Computation output of one SMC Guidance step."""
    velocity_cmd: np.ndarray    # [vx, vy, vz] in ENU world frame (m/s)
    yaw_rate_cmd: float         # Yaw angular rate (rad/s)
    sub_phase: str              # 'GLIDE_SLOPE' or 'FINAL_DESCENT'
    sliding_surface: np.ndarray # [S1, S2, S3]
    r_xy: float                 # Horizontal range (m)
    r_z: float                  # Vertical distance (m)
    valid: bool                 # True if guidance calculation succeeded


class SMCGuidance:
    """Pure mathematical implementation of SMC Landing Guidance."""

    def __init__(self, params: Optional[SMCParams] = None):
        self.p = params or SMCParams()
        # Internal integrated guidance states (speed, course, flight-path angle)
        self.Vp = 0.5
        self.alpha_p = 0.0
        self.gamma = -0.1
        self.initialized = False

    def reset(self, initial_vel: np.ndarray):
        """Reset guidance state from current drone velocity vector in ENU."""
        vx, vy, vz = initial_vel
        v_xy = math.hypot(vx, vy)
        self.Vp = max(0.1, float(np.linalg.norm(initial_vel)))
        self.alpha_p = math.atan2(vy, vx)
        # In ENU: vz positive up -> gamma = atan2(vz, v_xy)
        self.gamma = math.atan2(vz, max(v_xy, 1e-4))
        self.initialized = True

    def step(
        self,
        drone_pos: np.ndarray,      # [x, y, z] in ENU (m)
        drone_vel: np.ndarray,      # [vx, vy, vz] in ENU (m/s)
        target_pos: np.ndarray,     # [x, y, z] in ENU (m)
        target_vel: Optional[np.ndarray] = None, # [vx, vy, vz] in ENU (m/s)
        dt: float = 0.05,
        rswitch_override: Optional[float] = None,
        sliding_weight: float = 1.0,
    ) -> SMCResult:
        """
        Execute one guidance control step.
        """
        if not self.initialized:
            self.reset(drone_vel)

        if target_vel is None:
            target_vel = np.zeros(3)

        # 1. Relative geometry in ENU
        dx = target_pos[0] - drone_pos[0]
        dy = target_pos[1] - drone_pos[1]
        dz = drone_pos[2] - target_pos[2]  # Altitude above target (positive when above)

        r_xy = math.hypot(dx, dy)
        r_z = max(0.0, dz)

        rswitch = rswitch_override if rswitch_override is not None else self.p.rswitch_default
        rswitch = max(self.p.rmin, rswitch)

        # Final descent requires both horizontal and vertical proximity.
        # The adaptive handover radius may tighten this cylinder, but must
        # never trigger a vertical descent while the drone is far above/beside the pad.
        final_radius = min(rswitch, self.p.final_descent_rxy_m)
        if r_xy <= final_radius and r_z <= self.p.final_descent_alt_m:
            return self._final_descent_step(dx, dy, r_z, dt, target_vel)

        # 3. GLIDE_SLOPE Sub-Phase (SMC Guidance)
        return self._glide_slope_step(
            drone_pos, drone_vel, target_pos, target_vel, dx, dy, r_xy, r_z,
            rswitch, dt, sliding_weight
        )

    def _final_descent_step(
        self,
        dx: float,
        dy: float,
        r_z: float,
        dt: float,
        target_vel: Optional[np.ndarray] = None,
    ) -> SMCResult:
        """Terminal vertical descent directly over the landing pad."""
        # Position feedback is expressed in the same ENU world frame as the
        # target estimate. Pass target_vel feedforward to track moving landing pads!
        target_vel_xy = (
            np.asarray(target_vel, dtype=np.float64)[:2]
            if target_vel is not None
            else np.zeros(2)
        )
        xy_cmd = self._position_velocity(
            dx, dy, target_vel_xy, self.p.final_centering_kp,
            self.p.v_final_xy_max,
        )
        vx_cmd, vy_cmd = float(xy_cmd[0]), float(xy_cmd[1])

        # Vertical descent speed tapers near the ground
        if r_z > 0.40:
            vz_cmd = -self.p.v_descend_fast
        else:
            vz_cmd = -self.p.v_descend_touch

        vel_cmd = np.array([vx_cmd, vy_cmd, vz_cmd], dtype=np.float64)

        # Update internal states to track command
        self.reset(vel_cmd)

        return SMCResult(
            velocity_cmd=vel_cmd,
            yaw_rate_cmd=0.0,
            sub_phase="FINAL_DESCENT",
            sliding_surface=np.zeros(3),
            r_xy=math.hypot(dx, dy),
            r_z=r_z,
            valid=True,
        )

    @staticmethod
    def _position_velocity(
        dx: float,
        dy: float,
        target_vel_xy: np.ndarray,
        gain: float,
        speed_limit: float,
    ) -> np.ndarray:
        """Bounded ENU pursuit velocity toward the estimated target position."""
        command = gain * np.array([dx, dy], dtype=np.float64)
        target_velocity = np.asarray(target_vel_xy, dtype=np.float64)[:2]
        if target_velocity.shape == (2,) and np.all(np.isfinite(target_velocity)):
            command += target_velocity
        speed = float(np.linalg.norm(command))
        limit = max(0.0, float(speed_limit))
        if speed > limit and speed > 1e-9:
            command *= limit / speed
        return command

    def _glide_slope_step(
        self,
        drone_pos: np.ndarray,
        drone_vel: np.ndarray,
        target_pos: np.ndarray,
        target_vel: np.ndarray,
        dx: float,
        dy: float,
        r_xy: float,
        r_z: float,
        rswitch: float,
        dt: float,
        sliding_weight: float,
    ) -> SMCResult:
        """Glide Slope phase executing Tuân's Sliding Mode Control Equations."""
        # Target state in horizontal plane
        v_tx, v_ty = target_vel[0], target_vel[1]
        Vt = math.hypot(v_tx, v_ty)
        alpha_t = math.atan2(v_ty, v_tx) if Vt > 1e-4 else 0.0
        dVt = 0.0
        dalpha_t = 0.0
        ddalpha_t = 0.0

        # Drone state from internal integrated guidance states
        Vp = max(0.1, self.Vp)
        alpha_p = self.alpha_p
        gamma = np.clip(self.gamma, -math.pi / 2.5, math.pi / 2.5)

        cg = math.cos(gamma)
        sg = math.sin(gamma)

        # LOS azimuth
        psi = math.atan2(dy, dx)

        # Range rates
        dRxy = Vt * math.cos(alpha_t - psi) - Vp * cg * math.cos(alpha_p - psi)
        dpsi = (Vt * math.sin(alpha_t - psi) - Vp * cg * math.sin(alpha_p - psi)) / max(r_xy, 1e-3)

        # Vertical range rate in ENU (r_z = z_drone - z_target, so dr_z = vz_drone - vz_target)
        dRz = Vp * sg - target_vel[2]

        # Sliding Surfaces (Equations 17 from paper)
        td = math.tan(self.p.theta_des_rad)
        S1 = dRxy + self.p.ka * r_xy
        S2 = -dRz + td * dRxy + self.p.kb * (-r_z + td * r_xy)

        e_raw = psi - alpha_t - self.p.zeta_des_rad
        e_psi = math.atan2(math.sin(e_raw), math.cos(e_raw))
        S3 = (dpsi - dalpha_t) + self.p.kc * e_psi

        # Apply covariance adaptive weighting on horizontal surface
        S = np.array([sliding_weight * S1, S2, S3], dtype=np.float64)

        # Power reaching law
        power = self.p.n / self.p.m
        Sp = np.sign(S) * (np.abs(S) ** power)

        # Formulation of matrix A and vector B
        dp = alpha_p - psi
        dt_ang = alpha_t - psi

        Ap = np.array([
            [-math.cos(dp) * cg,  Vp * math.sin(dp) * cg,  Vp * math.cos(dp) * sg],
            [ sg,                 0.0,                    Vp * cg],
            [-math.sin(dp) * cg, -Vp * math.cos(dp) * cg,  Vp * math.sin(dp) * sg],
        ], dtype=np.float64)

        M = np.array([
            [1.0, 0.0, 0.0],
            [ td, 1.0, 0.0],
            [0.0, 0.0, 1.0],
        ], dtype=np.float64)

        A = M @ Ap

        dRxy_model = Vt * math.cos(dt_ang) - Vp * cg * math.cos(dp)
        Fxy = Vp * math.sin(dp) * cg * dpsi - dVt * math.cos(dt_ang) + Vt * math.sin(dt_ang) * (dalpha_t - dpsi)

        B1 = -self.p.k1 * Sp[0] + Fxy - self.p.ka * dRxy_model
        B2 = td * (Fxy - self.p.kb * dRxy_model) - self.p.k2 * Sp[1] - self.p.kb * Vp * sg
        B3 = (
            -r_xy * self.p.k3 * Sp[2]
            - self.p.kc * r_xy * (dpsi - dalpha_t)
            + ddalpha_t * r_xy
            + dRxy * dpsi
            - Vp * math.cos(dp) * cg * dpsi
            - Vt * math.cos(dt_ang) * (dalpha_t - dpsi)
            - dVt * math.sin(dt_ang)
        )
        B = np.array([B1, B2, B3], dtype=np.float64)

        # Solve acceleration commands A \ B
        try:
            cond = np.linalg.cond(A)
            if not np.isfinite(cond) or cond > 1e12:
                raise np.linalg.LinAlgError("Ill-conditioned A matrix")
            Uraw = np.linalg.solve(A, B)
        except np.linalg.LinAlgError:
            # Fallback towards LOS direction
            Uraw = np.array([0.0, 0.5 * (psi - alpha_p), -0.2])

        dVp = float(np.clip(Uraw[0], -10.0, 10.0))
        dalpha_p = float(np.clip(Uraw[1], -math.pi / 2.0, math.pi / 2.0))
        dgamma = float(np.clip(Uraw[2], -math.pi / 2.0, math.pi / 2.0))

        # Integrate guidance states
        dt_safe = min(max(dt, 0.001), 0.1)
        self.Vp = float(np.clip(self.Vp + dVp * dt_safe, 0.20, self.p.v_max))
        self.alpha_p = float(self.alpha_p + dalpha_p * dt_safe)
        self.gamma = float(np.clip(self.gamma + dgamma * dt_safe, -math.pi / 3.0, 0.0))

        xy_cmd = self._position_velocity(
            dx,
            dy,
            np.asarray(target_vel, dtype=np.float64)[:2]
            * self.p.target_velocity_ff_gain,
            self.p.glide_centering_kp,
            self.p.v_max,
        )
        vx_cmd, vy_cmd = float(xy_cmd[0]), float(xy_cmd[1])
        horizontal_speed = math.hypot(vx_cmd, vy_cmd)
        if r_xy <= rswitch and r_z > self.p.final_descent_alt_m:
            slope_speed = max(0.20, horizontal_speed * math.tan(abs(self.p.theta_des_rad)))
            alt_speed = max(self.p.v_descend_touch, 0.20 * r_z)
            descent_speed = min(
                self.p.v_descend_fast,
                max(slope_speed, alt_speed),
            )
            vz_cmd = -descent_speed
            # Keep the resultant 3D command inside the configured speed cap.
            horizontal_limit = math.sqrt(max(
                0.0, self.p.v_max * self.p.v_max - vz_cmd * vz_cmd
            ))
            if horizontal_speed > horizontal_limit and horizontal_speed > 1e-9:
                xy_scale = horizontal_limit / horizontal_speed
                vx_cmd *= xy_scale
                vy_cmd *= xy_scale
                horizontal_speed = horizontal_limit
            self.gamma = math.atan2(vz_cmd, max(horizontal_speed, 1e-4))
        elif r_xy <= rswitch and r_z <= self.p.final_descent_alt_m:
            vz_cmd = -self.p.v_descend_touch
            self.gamma = math.atan2(vz_cmd, max(horizontal_speed, 1e-4))
        else:
            # Hold height until the vehicle is inside the descent corridor.
            vz_cmd = 0.0
            self.gamma = 0.0

        vel_cmd = np.array([vx_cmd, vy_cmd, vz_cmd], dtype=np.float64)
        yaw_rate = float(np.clip(dalpha_p, -0.6, 0.6))

        return SMCResult(
            velocity_cmd=vel_cmd,
            yaw_rate_cmd=yaw_rate,
            sub_phase="GLIDE_SLOPE",
            sliding_surface=S,
            r_xy=r_xy,
            r_z=r_z,
            valid=True,
        )


class SMCGuidanceNode(Node):
    """ROS 2 Node executing SMC Landing Guidance."""

    def __init__(self):
        super().__init__('smc_guidance_node')

        self.declare_parameter('v_max', 1.50)
        self.declare_parameter('v_final_xy_max', 0.35)
        self.declare_parameter('v_descend_fast', 0.35)
        self.declare_parameter('v_descend_touch', 0.15)
        self.declare_parameter('rswitch_default', 0.80)
        self.declare_parameter('final_descent_rxy_m', 1.00)
        self.declare_parameter('final_descent_alt_m', 0.70)
        self.declare_parameter('final_uncertainty_max_2sigma_m', 0.35)
        self.declare_parameter('max_state_age_s', 0.30)
        self.declare_parameter('max_covariance_age_s', 0.30)
        self.declare_parameter('control_rate_hz', 20.0)

        params = SMCParams(
            v_max=float(self.get_parameter('v_max').value),
            v_final_xy_max=float(self.get_parameter('v_final_xy_max').value),
            v_descend_fast=float(self.get_parameter('v_descend_fast').value),
            v_descend_touch=float(self.get_parameter('v_descend_touch').value),
            rswitch_default=float(self.get_parameter('rswitch_default').value),
            final_descent_rxy_m=float(self.get_parameter('final_descent_rxy_m').value),
            final_descent_alt_m=float(self.get_parameter('final_descent_alt_m').value),
        )
        self.final_uncertainty_max = float(
            self.get_parameter('final_uncertainty_max_2sigma_m').value
        )
        self.max_state_age = float(self.get_parameter('max_state_age_s').value)
        self.max_covariance_age = float(
            self.get_parameter('max_covariance_age_s').value
        )

        self.guidance = SMCGuidance(params)
        rate_hz = float(self.get_parameter('control_rate_hz').value)

        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )

        self.drone_odom: Optional[Odometry] = None
        self.target_odom: Optional[Odometry] = None
        self.mission_phase = 'IDLE'
        self.tracking_mode = 'EXPIRED'
        self.safe_to_land: bool = False
        self.uncertainty_radius = float('inf')
        self.rswitch_adaptive: float = params.rswitch_default
        self.sliding_weight: float = 1.0
        self.last_odom_rx_time = 0.0
        self.last_target_rx_time = 0.0
        self.last_safe_rx_time = 0.0
        self.last_uncertainty_rx_time = 0.0
        self.last_step_time = time.monotonic()
        self.last_log_time = time.monotonic()

        # Subscriptions
        self.create_subscription(Odometry, '/odom', self.drone_odom_cb, sensor_qos)
        self.create_subscription(Odometry, '/ekf/target_state', self.target_odom_cb, 10)
        self.create_subscription(String, '/mission/phase', self.phase_cb, 10)
        self.create_subscription(String, '/ekf/tracking_mode', self.tracking_mode_cb, 10)
        self.create_subscription(Bool, '/landing/safe_to_land', self.safe_cb, 10)
        self.create_subscription(Float64, '/landing/uncertainty_radius', self.uncertainty_cb, 10)
        self.create_subscription(Float64, '/landing/rswitch_adaptive', self.rswitch_cb, 10)
        self.create_subscription(Float64, '/landing/sliding_weight', self.weight_cb, 10)

        # Publishers
        self.vel_pub = self.create_publisher(Twist, '/landing/velocity_cmd', 10)
        self.sub_phase_pub = self.create_publisher(String, '/landing/sub_phase', 10)
        self.diag_pub = self.create_publisher(DiagnosticStatus, '/landing/smc_status', 10)

        dt = 1.0 / max(rate_hz, 1.0)
        self.control_timer = self.create_timer(dt, self.control_loop)

        self.get_logger().info(
            f"SMCGuidanceNode initialized: rate={rate_hz:.1f}Hz, Rswitch_default={params.rswitch_default:.2f}m"
        )

    def drone_odom_cb(self, msg: Odometry):
        self.drone_odom = msg
        self.last_odom_rx_time = time.monotonic()

    def target_odom_cb(self, msg: Odometry):
        self.target_odom = msg
        self.last_target_rx_time = time.monotonic()

    def phase_cb(self, msg: String):
        previous = self.mission_phase
        self.mission_phase = msg.data
        if self.mission_phase == 'APPROACH' and previous not in ('APPROACH', 'LAND'):
            # Start the landing controller from the current motion, not from
            # integrator state accumulated while following/searching.
            self.guidance.initialized = False

    def tracking_mode_cb(self, msg: String):
        self.tracking_mode = msg.data

    def safe_cb(self, msg: Bool):
        self.safe_to_land = msg.data
        self.last_safe_rx_time = time.monotonic()

    def uncertainty_cb(self, msg: Float64):
        self.uncertainty_radius = float(msg.data)
        self.last_uncertainty_rx_time = time.monotonic()

    def rswitch_cb(self, msg: Float64):
        self.rswitch_adaptive = float(msg.data)

    def weight_cb(self, msg: Float64):
        self.sliding_weight = float(msg.data)

    def _publish_hold(self, state: str = 'HOLD'):
        # The vehicle may brake/hover while this command is held. Discard the
        # old integrated speed, course, and flight-path state so reacquisition
        # restarts guidance from measured odometry instead of a stale trajectory.
        self.guidance.initialized = False
        self.vel_pub.publish(Twist())
        self.sub_phase_pub.publish(String(data=state))

    def control_loop(self):
        now = time.monotonic()
        dt = now - self.last_step_time
        self.last_step_time = now

        if self.mission_phase not in ('APPROACH', 'LAND'):
            self._publish_hold('INACTIVE')
            return

        if self.drone_odom is None or self.target_odom is None:
            self._publish_hold()
            return

        state_fresh = (
            now - self.last_odom_rx_time <= self.max_state_age
            and now - self.last_target_rx_time <= self.max_state_age
        )
        covariance_fresh = (
            self.last_safe_rx_time > 0.0
            and self.last_uncertainty_rx_time > 0.0
            and now - self.last_safe_rx_time <= self.max_covariance_age
            and now - self.last_uncertainty_rx_time <= self.max_covariance_age
        )
        if (
            not state_fresh
            or not covariance_fresh
            or self.tracking_mode not in ('TRACKING', 'PREDICTING')
            or not self.safe_to_land
        ):
            # Never let a predicted or stale target state drive a landing
            # command. The FSM holds and waits for reacquisition/abort.
            self._publish_hold()
            return

        dp = self.drone_odom.pose.pose.position
        dv = self.drone_odom.twist.twist.linear
        tp = self.target_odom.pose.pose.position
        tv = self.target_odom.twist.twist.linear

        drone_pos = np.array([dp.x, dp.y, dp.z], dtype=np.float64)
        drone_vel = np.array([dv.x, dv.y, dv.z], dtype=np.float64)
        target_pos = np.array([tp.x, tp.y, tp.z], dtype=np.float64)
        target_vel = np.array([tv.x, tv.y, tv.z], dtype=np.float64)

        res = self.guidance.step(
            drone_pos=drone_pos,
            drone_vel=drone_vel,
            target_pos=target_pos,
            target_vel=target_vel,
            dt=dt,
            rswitch_override=self.rswitch_adaptive,
            sliding_weight=self.sliding_weight,
        )

        if (
            res.sub_phase == 'FINAL_DESCENT'
            and self.uncertainty_radius > self.final_uncertainty_max
        ):
            # If uncertainty is elevated above 0.60m, hold altitude while maintaining
            # active horizontal centering so the target stays locked in the camera frame!
            if res.r_z > 0.40:
                res.velocity_cmd[2] = 0.0
                res.sub_phase = 'FINAL_GATE_WAIT'
                self.sub_phase_pub.publish(String(data=res.sub_phase))
                twist = Twist()
                twist.linear.x = float(res.velocity_cmd[0])
                twist.linear.y = float(res.velocity_cmd[1])
                twist.linear.z = 0.0
                twist.angular.z = float(res.yaw_rate_cmd)
                self.vel_pub.publish(twist)
                return

        # Publish Twist command
        twist = Twist()
        twist.linear.x = float(res.velocity_cmd[0])
        twist.linear.y = float(res.velocity_cmd[1])
        twist.linear.z = float(res.velocity_cmd[2])
        twist.angular.z = float(res.yaw_rate_cmd)
        self.vel_pub.publish(twist)

        self.sub_phase_pub.publish(String(data=res.sub_phase))

        # Diagnostic message
        diag = DiagnosticStatus()
        diag.level = DiagnosticStatus.OK
        diag.name = "PrecisionLanding:SMCGuidance"
        diag.message = f"Phase: {res.sub_phase}"
        diag.hardware_id = "SLIDING_MODE_GUIDANCE"
        diag.values = [
            KeyValue(key="sub_phase", value=res.sub_phase),
            KeyValue(key="vx_cmd_mps", value=f"{res.velocity_cmd[0]:.3f}"),
            KeyValue(key="vy_cmd_mps", value=f"{res.velocity_cmd[1]:.3f}"),
            KeyValue(key="vz_cmd_mps", value=f"{res.velocity_cmd[2]:.3f}"),
            KeyValue(key="r_xy_m", value=f"{res.r_xy:.3f}"),
            KeyValue(key="r_z_m", value=f"{res.r_z:.3f}"),
            KeyValue(key="s1_range", value=f"{res.sliding_surface[0]:.4f}"),
            KeyValue(key="s2_slope", value=f"{res.sliding_surface[1]:.4f}"),
            KeyValue(key="s3_azimuth", value=f"{res.sliding_surface[2]:.4f}"),
            KeyValue(key="rswitch_m", value=f"{self.rswitch_adaptive:.3f}"),
            KeyValue(key="sliding_weight", value=f"{self.sliding_weight:.4f}"),
        ]
        self.diag_pub.publish(diag)

        if now - self.last_log_time > 2.0:
            self.get_logger().info(
                f"[{res.sub_phase}] Rxy={res.r_xy:.2f}m, Rz={res.r_z:.2f}m | "
                f"Cmd=({res.velocity_cmd[0]:.2f}, {res.velocity_cmd[1]:.2f}, {res.velocity_cmd[2]:.2f})m/s"
            )
            self.last_log_time = now


def main():
    rclpy.init()
    node = SMCGuidanceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
