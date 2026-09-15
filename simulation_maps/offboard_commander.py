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
from std_msgs.msg import Float64, String

# ---------------------------------------------------------------------------
# Try importing px4_msgs — fallback gracefully
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


def _enu_to_ned_velocity(vx_enu, vy_enu, vz_enu):
    """Convert velocity from ENU to NED frame for PX4.

    ENU: x-East, y-North, z-Up
    NED: x-North, y-East, z-Down
    """
    return vy_enu, vx_enu, -vz_enu


def _enu_yaw_to_ned(yaw_enu):
    """Convert yaw from ENU to NED convention.

    ENU yaw: 0 = East, +CCW
    NED yaw: 0 = North, +CW
    → yaw_ned = π/2 - yaw_enu
    """
    yaw_ned = math.pi / 2.0 - yaw_enu
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
        self.auto_arm = self.get_parameter('auto_arm').value
        self.auto_offboard = self.get_parameter('auto_offboard').value

        # ── State ────────────────────────────────────────────────────────
        self.velocity_enu = [0.0, 0.0, 0.0]
        self.yaw_enu = 0.0
        self.phase = 'IDLE'
        self.armed = False
        self.offboard_active = False
        self.arm_sent = False
        self.offboard_sent = False
        self.last_log_time = 0.0

        # ── QoS for PX4 topics ───────────────────────────────────────────
        px4_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
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

        # ── PX4 Publishers (or simulation stubs) ─────────────────────────
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
            self.get_logger().info('Offboard Commander ready (px4_msgs AVAILABLE).')
        else:
            self.offboard_pub = None
            self.setpoint_pub = None
            self.command_pub = None
            self.get_logger().warning(
                'px4_msgs NOT FOUND — running in SIMULATION-ONLY mode.\n'
                '  Cài px4_msgs: cd ~/px4_msgs_ws && colcon build\n'
                '  Hoặc clone: git clone https://github.com/PX4/px4_msgs.git'
            )

        # ── Heartbeat timer (10 Hz for OffboardControlMode) ──────────────
        self.create_timer(0.1, self.heartbeat_cb)

        # ── Setpoint timer (sync with FSM at 20 Hz) ─────────────────────
        self.create_timer(0.05, self.publish_setpoint)

    # ── Callbacks ────────────────────────────────────────────────────────

    def velocity_cb(self, msg: Twist):
        self.velocity_enu = [msg.linear.x, msg.linear.y, msg.linear.z]

    def yaw_cb(self, msg: Float64):
        self.yaw_enu = msg.data

    def phase_cb(self, msg: String):
        self.phase = msg.data

        # Auto-arm and switch to offboard when entering SEARCH
        if self.phase == 'SEARCH' and self.auto_offboard and not self.offboard_sent:
            self._send_offboard_mode()
            self.offboard_sent = True
        if self.phase == 'SEARCH' and self.auto_arm and not self.arm_sent:
            self._send_arm_command()
            self.arm_sent = True

    # ── PX4 Communication ────────────────────────────────────────────────

    def heartbeat_cb(self):
        """Publish OffboardControlMode at 10 Hz to keep PX4 in offboard."""
        if not HAS_PX4_MSGS or self.offboard_pub is None:
            return
        if self.phase == 'IDLE':
            return

        msg = OffboardControlMode()
        msg.position = False
        msg.velocity = True
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.offboard_pub.publish(msg)

    def publish_setpoint(self):
        """Publish TrajectorySetpoint with velocity + yaw in NED."""
        if self.phase == 'IDLE':
            return

        # Convert ENU → NED
        vn, ve, vd = _enu_to_ned_velocity(*self.velocity_enu)
        yaw_ned = _enu_yaw_to_ned(self.yaw_enu)

        if HAS_PX4_MSGS and self.setpoint_pub is not None:
            msg = TrajectorySetpoint()
            msg.position = [float('nan')] * 3   # velocity mode
            msg.velocity = [float(vn), float(ve), float(vd)]
            msg.yaw = float(yaw_ned)
            msg.yawspeed = float('nan')
            msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
            self.setpoint_pub.publish(msg)
        else:
            # Simulation-only: log periodically
            now = time.monotonic()
            if now - self.last_log_time > 1.0:
                self.get_logger().info(
                    f'[SIM-ONLY] phase={self.phase} '
                    f'vel_NED=[{vn:.2f}, {ve:.2f}, {vd:.2f}] '
                    f'yaw_NED={math.degrees(yaw_ned):.1f}°'
                )
                self.last_log_time = now

    def _send_arm_command(self):
        """Send ARM command to PX4."""
        if not HAS_PX4_MSGS or self.command_pub is None:
            self.get_logger().info('[SIM-ONLY] Would send ARM command')
            return

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
        self.get_logger().info('ARM command sent to PX4.')

    def _send_offboard_mode(self):
        """Send mode switch to OFFBOARD."""
        if not HAS_PX4_MSGS or self.command_pub is None:
            self.get_logger().info('[SIM-ONLY] Would switch to OFFBOARD mode')
            return

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
        self.get_logger().info('OFFBOARD mode command sent to PX4.')

    def _send_disarm_command(self):
        """Send DISARM command to PX4."""
        if not HAS_PX4_MSGS or self.command_pub is None:
            return

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
        self.get_logger().info('DISARM command sent to PX4.')


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
