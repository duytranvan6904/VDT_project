#!/usr/bin/env python3
"""PX4 Offboard Commander — giao tiếp PX4 SITL qua micro-XRCE-DDS.

Chức năng:
  1. Kích hoạt Offboard mode và Arm drone
  2. Nhận velocity setpoint [vx, vy, vz] + yaw từ Mission FSM
  3. Chuyển đổi ENU → NED cho PX4
  4. Publish TrajectorySetpoint + OffboardControlMode
  5. Safety heartbeat: OffboardControlMode @ 10 Hz

Yêu cầu: px4_msgs ROS 2 package phải được build.
Nếu px4_msgs chưa cài, node chạy ở SIMULATION-ONLY mode (log setpoints,
không gửi PX4).

ROS 2 interface:
  Subscribe: /mission/velocity_setpoint, /mission/yaw_setpoint, /mission/phase
  Publish:   /fmu/in/offboard_control_mode, /fmu/in/trajectory_setpoint,
             /fmu/in/vehicle_command
"""

from __future__ import annotations

import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64, String

# ---------------------------------------------------------------------------
# Try importing px4_msgs — fallback to pymavlink from PX4-Autopilot tree
# ---------------------------------------------------------------------------
try:
    from px4_msgs.msg import (
        OffboardControlMode,
        TrajectorySetpoint,
        VehicleCommand,
    )
    HAS_PX4_MSGS = True
except ImportError:
    HAS_PX4_MSGS = False

HAS_PYMAVLINK = False
mavutil = None
if not HAS_PX4_MSGS:
    try:
        import sys, os
        os.environ['MAVLINK_DIALECT'] = 'common'
        px4_mav_path = '/home/duy/VDT_project/PX4-Autopilot/src/modules/mavlink/mavlink'
        if px4_mav_path not in sys.path:
            sys.path.insert(0, px4_mav_path)
        from pymavlink import mavutil
        HAS_PYMAVLINK = True
    except Exception:
        HAS_PYMAVLINK = False


def _enu_to_ned_velocity(vx_world, vy_world, vz_world):
    """Convert velocity from Gazebo world (East=+X, North=+Y, Up=+Z)
    to PX4 Local NED (North=+X, East=+Y, Down=+Z).

    In PX4 SITL (with Gazebo ENU world):
      North = Gazebo +Y (North)  -> vn = vy_world
      East  = Gazebo +X (East)   -> ve = vx_world
      Down  = Gazebo -Z (Down)   -> vd = -vz_world
    """
    return float(vy_world), float(vx_world), float(-vz_world)


def _enu_yaw_to_ned(yaw_world):
    """Convert yaw from Gazebo world to PX4 Local NED.

    Gazebo ENU: 0 = +X (East), +CCW towards +Y (North)
    PX4 NED:    0 = +X (North), +CW towards +Y (East)
    -> yaw_ned = pi/2 - yaw_world
    """
    yaw_ned = math.pi / 2.0 - yaw_world
    # Normalize to [-π, π]
    while yaw_ned > math.pi:
        yaw_ned -= 2.0 * math.pi
    while yaw_ned < -math.pi:
        yaw_ned += 2.0 * math.pi
    return yaw_ned


class OffboardCommander(Node):
    """PX4 Offboard mode commander.

    When ``px4_msgs`` is available, the node publishes real PX4 commands.
    Otherwise it runs in simulation-only mode, logging setpoints for debug.
    """

    # PX4 vehicle command codes
    VEHICLE_CMD_DO_SET_MODE = 176
    VEHICLE_CMD_COMPONENT_ARM_DISARM = 400

    def __init__(self):
        super().__init__('offboard_commander')

        self.declare_parameter('auto_arm', True)
        self.declare_parameter('auto_offboard', True)
        self.declare_parameter('min_offboard_alt', 2.0)
        self.declare_parameter('max_accel', 2.0)
        self.auto_arm = self.get_parameter('auto_arm').value
        self.auto_offboard = self.get_parameter('auto_offboard').value
        self.min_offboard_alt = float(self.get_parameter('min_offboard_alt').value)
        self.max_accel = float(self.get_parameter('max_accel').value)

        # ── State ────────────────────────────────────────────────────────
        self.velocity_enu = [0.0, 0.0, 0.0]
        self.yaw_enu = 0.0
        self.phase = 'IDLE'
        self.current_alt = 0.0
        self.has_odom = False
        self.armed = False
        self.offboard_active = False
        self.offboard_engaged = False
        self.current_cmd_ned = [0.0, 0.0, 0.0]
        self.last_setpoint_time = 0.0
        self.arm_sent = False
        self.offboard_sent = False
        self.last_log_time = 0.0
        self.last_heartbeat_time = 0.0
        self.last_mode_req_time = 0.0
        self.px4_current_main_mode = None

        # ── QoS for PX4 topics ───────────────────────────────────────────
        px4_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )

        # ── Subscribers ──────────────────────────────────────────────────
        self.create_subscription(
            Twist, '/mission/velocity_setpoint',
            self.velocity_cb, 10,
        )
        self.create_subscription(
            Float64, '/mission/yaw_setpoint',
            self.yaw_cb, 10,
        )
        self.create_subscription(
            String, '/mission/phase',
            self.phase_cb, 10,
        )
        self.create_subscription(
            Odometry, '/odom',
            self.odom_cb, sensor_qos,
        )

        # ── PX4 Publishers (hoặc MAVLink UDP connection) ────────────────
        self.mav_conn = None
        if HAS_PX4_MSGS:
            self.offboard_pub = self.create_publisher(
                OffboardControlMode,
                '/fmu/in/offboard_control_mode', px4_qos,
            )
            self.setpoint_pub = self.create_publisher(
                TrajectorySetpoint,
                '/fmu/in/trajectory_setpoint', px4_qos,
            )
            self.command_pub = self.create_publisher(
                VehicleCommand,
                '/fmu/in/vehicle_command', px4_qos,
            )
            self.get_logger().info('Offboard Commander: px4_msgs AVAILABLE (micro-XRCE-DDS mode).')
        elif HAS_PYMAVLINK:
            self.offboard_pub = None
            self.setpoint_pub = None
            self.command_pub = None
            try:
                # Cổng 14540 là cổng chuẩn PX4 SITL MAVLink cho companion computer / offboard
                self.mav_conn = mavutil.mavlink_connection('udp:127.0.0.1:14540')
                self.get_logger().info(
                    'Offboard Commander: Kết nối MAVLink UDP (udp:127.0.0.1:14540) thành công!\n'
                    '  Drone sẽ nhận lệnh vận tốc APF thực tế trong chế độ OFFBOARD.'
                )
            except Exception as e:
                self.get_logger().warning(f'Không thể mở socket MAVLink UDP: {e}')
        else:
            self.offboard_pub = None
            self.setpoint_pub = None
            self.command_pub = None
            self.get_logger().warning('Chạy ở chế độ SIMULATION-ONLY (chỉ log setpoint).')

        # ── Heartbeat timer (10 Hz for OffboardControlMode) ──────────────
        self.create_timer(0.1, self.heartbeat_cb)

        # ── Setpoint timer (sync with FSM at 20 Hz) ─────────────────────
        self.create_timer(0.05, self.publish_setpoint)

    # ── Callbacks ────────────────────────────────────────────────────────

    def odom_cb(self, msg: Odometry):
        self.current_alt = float(msg.pose.pose.position.z)
        self.has_odom = True

    def velocity_cb(self, msg: Twist):
        self.velocity_enu = [msg.linear.x, msg.linear.y, msg.linear.z]

    def yaw_cb(self, msg: Float64):
        self.yaw_enu = msg.data

    def phase_cb(self, msg: String):
        prev_phase = self.phase
        self.phase = msg.data

        # Reset latch on touchdown / disarm
        if self.phase in ('IDLE', 'LAND') and self.current_alt < 0.5:
            self.offboard_engaged = False

        # Tự động kích hoạt OFFBOARD mode khi bước vào pha FOLLOW để APF lái drone (nếu đã đủ độ cao)
        if self.phase in ('FOLLOW', 'APPROACH') and prev_phase in ('IDLE', 'SEARCH'):
            if self.current_alt >= self.min_offboard_alt:
                self.offboard_engaged = True
                self._send_offboard_mode()

    # ── PX4 Communication ────────────────────────────────────────────────

    def heartbeat_cb(self):
        """Publish OffboardControlMode at 10 Hz (DDS) or MAVLink companion heartbeat + mode enforce."""
        now = time.monotonic()

        # Kiểm tra điều kiện chốt kích hoạt OFFBOARD mode
        if not self.offboard_engaged:
            if self.phase in ('FOLLOW', 'APPROACH') and self.current_alt >= self.min_offboard_alt:
                self.offboard_engaged = True
                self.get_logger().info(
                    f'✅ Drone đạt độ cao an toàn ({self.current_alt:.2f}m >= {self.min_offboard_alt:.1f}m)! '
                    f'Kích hoạt chốt OFFBOARD mode.'
                )

        if HAS_PX4_MSGS and self.offboard_pub is not None:
            if not self.offboard_engaged:
                return
            msg = OffboardControlMode()
            msg.position = False
            msg.velocity = True
            msg.acceleration = False
            msg.attitude = False
            msg.body_rate = False
            msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
            self.offboard_pub.publish(msg)

        elif self.mav_conn is not None:
            # 1. Phát Heartbeat định kỳ 1 Hz để PX4 nhận biết Companion Computer kết nối
            if now - self.last_heartbeat_time >= 1.0:
                try:
                    self.mav_conn.mav.heartbeat_send(
                        mavutil.mavlink.MAV_TYPE_ONBOARD_CONTROLLER,
                        mavutil.mavlink.MAV_AUTOPILOT_INVALID,
                        0, 0, 0
                    )
                except Exception:
                    pass
                self.last_heartbeat_time = now

            # 2. Đọc trạng thái flight mode hiện tại của PX4 từ HEARTBEAT
            try:
                while True:
                    m = self.mav_conn.recv_match(type='HEARTBEAT', blocking=False)
                    if not m:
                        break
                    if m.get_srcSystem() == 1:
                        self.px4_current_main_mode = (m.custom_mode >> 16) & 0xFF
            except Exception:
                pass

            # 3. Khi đã chốt OFFBOARD mode, liên tục duy trì yêu cầu OFFBOARD nếu bị rớt mode
            if self.offboard_engaged:
                if self.px4_current_main_mode != 6 and (now - self.last_mode_req_time >= 1.0):
                    self._send_offboard_mode()
                    self.last_mode_req_time = now

    def publish_setpoint(self):
        """Publish TrajectorySetpoint with velocity + yaw in NED with smooth slew-rate limiting."""
        # SAFETY GUARD: Chỉ phát setpoint sau khi OFFBOARD đã được chốt kích hoạt an toàn!
        if not self.offboard_engaged:
            return

        now = time.monotonic()
        dt = 0.05
        if self.last_setpoint_time > 0.0:
            dt = max(0.01, min(0.2, now - self.last_setpoint_time))
        self.last_setpoint_time = now

        # Convert ENU → NED
        raw_vn, raw_ve, raw_vd = _enu_to_ned_velocity(*self.velocity_enu)
        yaw_ned = _enu_yaw_to_ned(self.yaw_enu)

        # Slew-rate limiter (acceleration clamping)
        max_dv_xy = self.max_accel * dt
        max_dv_z = 0.8 * dt  # Gentler vertical acceleration limit (0.8 m/s^2) to prevent altitude wobbling
        def slew(curr: float, target: float, limit: float) -> float:
            diff = target - curr
            if abs(diff) > limit:
                return curr + math.copysign(limit, diff)
            return target

        vn = slew(self.current_cmd_ned[0], raw_vn, max_dv_xy)
        ve = slew(self.current_cmd_ned[1], raw_ve, max_dv_xy)
        vd = slew(self.current_cmd_ned[2], raw_vd, max_dv_z)
        self.current_cmd_ned = [vn, ve, vd]

        if HAS_PX4_MSGS and self.setpoint_pub is not None:
            msg = TrajectorySetpoint()
            msg.position = [float('nan')] * 3   # velocity mode
            msg.velocity = [float(vn), float(ve), float(vd)]
            msg.yaw = float(yaw_ned)
            msg.yawspeed = float('nan')
            msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
            self.setpoint_pub.publish(msg)

        elif self.mav_conn is not None:
            # Gửi setpoint NED thực tế qua MAVLink UDP tới PX4 SITL
            time_boot_ms = int(time.monotonic() * 1000) & 0xFFFFFFFF
            # 2503 (0x9c7): Chỉ kích hoạt vx, vy, vz và yaw; bỏ qua position, accel, force, yaw_rate
            type_mask = 2503
            try:
                self.mav_conn.mav.set_position_target_local_ned_send(
                    time_boot_ms,
                    1, 1, # target system, component
                    mavutil.mavlink.MAV_FRAME_LOCAL_NED,
                    type_mask,
                    0.0, 0.0, 0.0, # x, y, z (ignored)
                    float(vn), float(ve), float(vd), # vx, vy, vz
                    0.0, 0.0, 0.0, # ax, ay, az (ignored)
                    float(yaw_ned), 0.0 # yaw, yaw_rate
                )
            except Exception:
                pass

            if now - self.last_log_time > 2.0:
                mode_str = "OFFBOARD" if self.px4_current_main_mode == 6 else f"PX4_Mode_{self.px4_current_main_mode}"
                self.get_logger().info(
                    f'[MAVLink Offboard] State={mode_str} | Phase={self.phase:<7} | '
                    f'vel_NED=[{vn:.2f}, {ve:.2f}, {vd:.2f}] m/s | '
                    f'yaw_NED={math.degrees(yaw_ned):.1f}°'
                )
                self.last_log_time = now

        else:
            # Simulation-only: log định kỳ
            now = time.monotonic()
            if now - self.last_log_time > 2.0:
                self.get_logger().info(
                    f'[SIM-ONLY] Phase={self.phase} '
                    f'vel_NED=[{vn:.2f}, {ve:.2f}, {vd:.2f}] '
                    f'yaw_NED={math.degrees(yaw_ned):.1f}°'
                )
                self.last_log_time = now

    def _send_arm_command(self):
        """Send ARM command to PX4."""
        if HAS_PX4_MSGS and self.command_pub is not None:
            msg = VehicleCommand()
            msg.command = self.VEHICLE_CMD_COMPONENT_ARM_DISARM
            msg.param1 = 1.0   # 1 = arm
            msg.target_system = 1
            msg.target_component = 1
            msg.source_system = 1
            msg.source_component = 1
            msg.from_external = True
            msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
            self.command_pub.publish(msg)
            self.get_logger().info('ARM command sent to PX4 (DDS).')
        elif self.mav_conn is not None:
            try:
                self.mav_conn.mav.command_long_send(
                    1, 1,
                    mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                    0, 1.0, 0, 0, 0, 0, 0, 0
                )
                self.get_logger().info('ARM command sent to PX4 (MAVLink).')
            except Exception as e:
                self.get_logger().warning(f'Failed to send ARM command: {e}')

    def _send_offboard_mode(self):
        """Send mode switch to OFFBOARD."""
        if HAS_PX4_MSGS and self.command_pub is not None:
            msg = VehicleCommand()
            msg.command = self.VEHICLE_CMD_DO_SET_MODE
            msg.param1 = 1.0   # custom mode
            msg.param2 = 6.0   # PX4_CUSTOM_MAIN_MODE_OFFBOARD
            msg.target_system = 1
            msg.target_component = 1
            msg.source_system = 1
            msg.source_component = 1
            msg.from_external = True
            msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
            self.command_pub.publish(msg)
            self.get_logger().info('OFFBOARD mode command sent to PX4 (DDS).')
        elif self.mav_conn is not None:
            try:
                # Chuyển mode sang OFFBOARD: base_mode=1 (CUSTOM), custom_main_mode=6 (OFFBOARD)
                self.mav_conn.mav.command_long_send(
                    1, 1,
                    mavutil.mavlink.MAV_CMD_DO_SET_MODE,
                    0,
                    mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                    6.0, # PX4_CUSTOM_MAIN_MODE_OFFBOARD
                    0.0, 0.0, 0.0, 0.0, 0.0
                )
                self.get_logger().info('✅ Đã gửi lệnh chuyển sang chế độ OFFBOARD tới PX4 SITL (MAVLink)!')
            except Exception as e:
                self.get_logger().warning(f'Failed to set OFFBOARD mode: {e}')

    def _send_disarm_command(self):
        """Send DISARM command to PX4."""
        if HAS_PX4_MSGS and self.command_pub is not None:
            msg = VehicleCommand()
            msg.command = self.VEHICLE_CMD_COMPONENT_ARM_DISARM
            msg.param1 = 0.0   # 0 = disarm
            msg.target_system = 1
            msg.target_component = 1
            msg.source_system = 1
            msg.source_component = 1
            msg.from_external = True
            msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
            self.command_pub.publish(msg)
            self.get_logger().info('DISARM command sent to PX4 (DDS).')
        elif self.mav_conn is not None:
            try:
                self.mav_conn.mav.command_long_send(
                    1, 1,
                    mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                    0, 0.0, 0, 0, 0, 0, 0, 0
                )
                self.get_logger().info('DISARM command sent to PX4 (MAVLink).')
            except Exception:
                pass


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main():
    rclpy.init()
    node = OffboardCommander()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print('\n[Offboard] Shutting down...')
        node._send_disarm_command()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
