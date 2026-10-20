#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from px4_msgs.msg import VehicleStatus, VehicleLocalPosition


class FakePx4(Node):
    def __init__(self):
        super().__init__('fake_px4')
        self.ps = self.create_publisher(
            VehicleStatus, '/sim/out/vehicle_status_v1', qos_profile_sensor_data)
        self.pl = self.create_publisher(
            VehicleLocalPosition, '/sim/out/vehicle_local_position', qos_profile_sensor_data)
        self.create_timer(0.1, self.tick)

    def tick(self):
        s = VehicleStatus()
        s.arming_state = VehicleStatus.ARMING_STATE_ARMED
        s.nav_state = 14
        s.failsafe = False
        s.pre_flight_checks_pass = True
        self.ps.publish(s)
        p = VehicleLocalPosition()
        p.xy_valid = True
        p.z_valid = True
        p.z = -3.0
        self.pl.publish(p)


def main():
    rclpy.init()
    rclpy.spin(FakePx4())


if __name__ == '__main__':
    main()
