import argparse
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                       ReliabilityPolicy, qos_profile_sensor_data)
from rclpy.utilities import remove_ros_args
from std_msgs.msg import Bool, String
from px4_msgs.msg import VehicleCommand, VehicleLocalPosition, VehicleStatus

PUB_QOS = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                     durability=DurabilityPolicy.TRANSIENT_LOCAL,
                     history=HistoryPolicy.KEEP_LAST, depth=1)
LATCH_QOS = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                       durability=DurabilityPolicy.TRANSIENT_LOCAL,
                       history=HistoryPolicy.KEEP_LAST, depth=1)
CMD_QOS = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                     history=HistoryPolicy.KEEP_LAST, depth=10)
NAN = float('nan')

IDLE, ARMING, CLIMBING, AIRBORNE, LANDING = (
    'idle', 'arming', 'climbing', 'airborne', 'landing')


class FlightNode(Node):
    def __init__(self, alt, arm_timeout, climb_timeout):
        super().__init__('takeoff_cmd')
        self.alt = alt
        self.arm_timeout = arm_timeout
        self.climb_timeout = climb_timeout
        self.status = None
        self.local = None
        self.killed = False
        self.h0 = NAN
        self.state = IDLE
        self.t_state = time.monotonic()
        self.t_send = 0.0
        self.pub = self.create_publisher(
            VehicleCommand, '/fmu/in/vehicle_command', PUB_QOS)
        self.state_pub = self.create_publisher(
            String, 'takeoff_cmd/state', LATCH_QOS)
        self.create_subscription(
            VehicleStatus, '/fmu/out/vehicle_status_v1',
            self.on_status, qos_profile_sensor_data)
        self.create_subscription(
            VehicleLocalPosition, '/fmu/out/vehicle_local_position',
            self.on_local, qos_profile_sensor_data)
        self.create_subscription(Bool, 'system/killed', self.on_killed, LATCH_QOS)
        self.create_subscription(
            String, 'takeoff_cmd/command', self.on_command, CMD_QOS)
        self.create_timer(0.05, self.tick)
        self.set_state(IDLE)

    def on_status(self, m):
        self.status = m

    def on_local(self, m):
        self.local = m

    def on_killed(self, m):
        self.killed = m.data

    def on_command(self, m):
        c = m.data.strip()
        if c == 'emergency_land':
            self.begin_landing()
        elif c == 'land':
            if self.status and self.armed():
                self.begin_landing()
        elif c == 'takeoff':
            self.begin_takeoff()

    def set_state(self, s):
        self.state = s
        self.t_state = time.monotonic()
        self.state_pub.publish(String(data=s))

    def send(self, cmd, params):
        m = VehicleCommand()
        m.timestamp = self.get_clock().now().nanoseconds // 1000
        m.command = cmd
        for i, v in enumerate(params, 1):
            setattr(m, f'param{i}', float(v))
        m.target_system = 1
        m.target_component = 1
        m.source_system = 1
        m.source_component = 1
        m.from_external = True
        self.pub.publish(m)
        self.t_send = time.monotonic()

    def send_arm(self):
        self.send(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM,
                  [1, 0, 0, 0, 0, 0, 0])

    def send_land(self):
        self.send(VehicleCommand.VEHICLE_CMD_NAV_LAND,
                  [0, 0, 0, NAN, NAN, NAN, NAN])

    def armed(self):
        return self.status.arming_state == VehicleStatus.ARMING_STATE_ARMED

    def landing(self):
        return self.status.nav_state == VehicleStatus.NAVIGATION_STATE_AUTO_LAND

    def height(self):
        return -self.local.z if self.local.z_valid else NAN

    def ready(self):
        if self.status.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD:
            return True
        return self.height() - self.h0 >= 0.9 * self.alt

    def gps_ok(self):
        l = self.local
        return l.xy_global and l.z_global and not l.dead_reckoning

    def begin_takeoff(self):
        if self.state != IDLE or self.killed or not (self.status and self.local):
            return
        if self.armed() or not self.gps_ok():
            return
        self.send_arm()
        self.set_state(ARMING)

    def begin_landing(self):
        if self.state == LANDING:
            return
        self.send_land()
        self.set_state(LANDING)

    def tick_landing(self, now):
        if self.status and not self.armed():
            self.set_state(IDLE)
            return
        in_land = self.status and self.landing()
        if not in_land and now - self.t_send > 0.5:
            self.send_land()

    def tick(self):
        now = time.monotonic()
        if self.killed:
            if self.state != IDLE:
                self.set_state(IDLE)
            return
        if self.state == LANDING:
            self.tick_landing(now)
            return
        if not (self.status and self.local):
            return
        dt = now - self.t_state
        if self.state == ARMING:
            if self.armed():
                self.send(VehicleCommand.VEHICLE_CMD_NAV_TAKEOFF,
                    [-1, 0, 0, NAN, NAN, NAN, NAN])
                self.h0 = self.height()
                self.set_state(CLIMBING)
            elif dt > self.arm_timeout:
                self.set_state(IDLE)
            elif now - self.t_send > 1.0:
                self.send_arm()
        elif self.state == CLIMBING:
            if not self.armed():
                self.set_state(IDLE)
            elif self.ready():
                self.set_state(AIRBORNE)
            elif dt > self.climb_timeout:
                self.begin_landing()
        elif self.state == AIRBORNE:
            if not self.armed():
                self.set_state(IDLE)


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--alt', type=float, default=3.0)
    p.add_argument('--arm-timeout', type=float, default=5.0)
    p.add_argument('--climb-timeout', type=float, default=20.0)
    a = p.parse_args(remove_ros_args(sys.argv)[1:])
    rclpy.init()
    node = FlightNode(a.alt, a.arm_timeout, a.climb_timeout)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == '__main__':
    main()
