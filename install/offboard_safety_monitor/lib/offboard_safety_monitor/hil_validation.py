#!/usr/bin/env python3
"""Runtime validation harness for PX4 SITL/HIL or a connected vehicle.

Run only with props removed and a controlled test environment.
"""
import argparse
import time

import rclpy
from offboard_manager.msg import OffboardStatus
from px4_msgs.msg import BatteryStatus, VehicleCommandAck, VehicleLocalPosition, VehicleStatus
from std_msgs.msg import Bool

VEHICLE_CMD_DO_SET_MODE = 176
VEHICLE_CMD_COMPONENT_ARM_DISARM = 400
VEHICLE_CMD_RESULT_ACCEPTED = 0
NAV_STATE_OFFBOARD = 14


class HilValidation:
    def __init__(self, timeout, require_command_ack):
        self.node = rclpy.create_node('offboard_safety_hil_validation')
        self.timeout = timeout
        self.require_command_ack = require_command_ack
        self.status_time = None
        self.last_nav_state = None
        self.position_time = None
        self.battery_time = None
        self.force_land = None
        self.inhibit = None
        self.offboard_status = None
        self.command_acks = {}
        self.node.create_subscription(VehicleStatus, '/fmu/out/vehicle_status', self.status_cb, 10)
        self.node.create_subscription(VehicleLocalPosition, '/fmu/out/vehicle_local_position', self.position_cb, 10)
        self.node.create_subscription(BatteryStatus, '/fmu/out/battery_status', self.battery_cb, 10)
        self.node.create_subscription(Bool, 'safety/force_land', self.force_land_cb, 10)
        self.node.create_subscription(Bool, 'safety/inhibit_offboard', self.inhibit_cb, 10)
        self.node.create_subscription(OffboardStatus, 'offboard/status', self.offboard_status_cb, 10)
        self.node.create_subscription(
            VehicleCommandAck, '/fmu/out/vehicle_command_ack', self.command_ack_cb, 10)

    def status_cb(self, msg):
        self.status_time = time.monotonic()
        self.last_nav_state = msg.nav_state

    def position_cb(self, _):
        self.position_time = time.monotonic()

    def battery_cb(self, _):
        self.battery_time = time.monotonic()

    def force_land_cb(self, msg):
        self.force_land = msg.data

    def inhibit_cb(self, msg):
        self.inhibit = msg.data

    def offboard_status_cb(self, msg):
        self.offboard_status = msg

    def command_ack_cb(self, msg):
        if msg.result == VEHICLE_CMD_RESULT_ACCEPTED:
            self.command_acks[msg.command] = True

    def run(self):
        deadline = time.monotonic() + self.timeout
        while time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.1)
        now = time.monotonic()
        checks = {
            'vehicle_status_fresh': self.status_time is not None and now - self.status_time < 1.0,
            'local_position_fresh': self.position_time is not None and now - self.position_time < 1.0,
            'battery_fresh': self.battery_time is not None and now - self.battery_time < 1.0,
            'force_land_topic_present': self.force_land is not None,
            'inhibit_topic_present': self.inhibit is not None,
            'offboard_status_present': self.offboard_status is not None,
        }
        if self.require_command_ack:
            checks.update({
                'offboard_command_ack_accepted':
                    self.command_acks.get(VEHICLE_CMD_DO_SET_MODE, False),
                'arm_command_ack_accepted':
                    self.command_acks.get(VEHICLE_CMD_COMPONENT_ARM_DISARM, False),
                'px4_offboard_confirmed':
                    self.last_nav_state == NAV_STATE_OFFBOARD,
            })
        for name, passed in checks.items():
            print(f'{name}: {"PASS" if passed else "FAIL"}')
        return all(checks.values())


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--duration', type=float, default=5.0)
    parser.add_argument(
        '--skip-command-ack', action='store_true',
        help='Only check topic freshness; do not require Offboard/arm ACKs.')
    args = parser.parse_args()
    rclpy.init()
    harness = HilValidation(args.duration, not args.skip_command_ack)
    try:
        raise SystemExit(0 if harness.run() else 1)
    finally:
        harness.node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
