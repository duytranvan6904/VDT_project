#!/usr/bin/env python3
"""Runtime validation harness for PX4 SITL/HIL or a connected vehicle.

Run only with props removed and a controlled test environment.
"""
import argparse
import time

import rclpy
from px4_msgs.msg import BatteryStatus, VehicleLocalPosition, VehicleStatus
from std_msgs.msg import Bool


class HilValidation:
    def __init__(self, timeout):
        self.node = rclpy.create_node('offboard_safety_hil_validation')
        self.timeout = timeout
        self.status_time = None
        self.position_time = None
        self.battery_time = None
        self.force_land = None
        self.inhibit = None
        self.node.create_subscription(VehicleStatus, '/fmu/out/vehicle_status', self.status_cb, 10)
        self.node.create_subscription(VehicleLocalPosition, '/fmu/out/vehicle_local_position', self.position_cb, 10)
        self.node.create_subscription(BatteryStatus, '/fmu/out/battery_status', self.battery_cb, 10)
        self.node.create_subscription(Bool, 'safety/force_land', self.force_land_cb, 10)
        self.node.create_subscription(Bool, 'safety/inhibit_offboard', self.inhibit_cb, 10)

    def status_cb(self, _):
        self.status_time = time.monotonic()

    def position_cb(self, _):
        self.position_time = time.monotonic()

    def battery_cb(self, _):
        self.battery_time = time.monotonic()

    def force_land_cb(self, msg):
        self.force_land = msg.data

    def inhibit_cb(self, msg):
        self.inhibit = msg.data

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
        }
        for name, passed in checks.items():
            print(f'{name}: {"PASS" if passed else "FAIL"}')
        return all(checks.values())


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--duration', type=float, default=5.0)
    args = parser.parse_args()
    rclpy.init()
    harness = HilValidation(args.duration)
    try:
        raise SystemExit(0 if harness.run() else 1)
    finally:
        harness.node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
