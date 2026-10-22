#!/usr/bin/env python3
import argparse
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, qos_profile_sensor_data
from std_msgs.msg import UInt8, Bool
from px4_msgs.msg import VehicleStatus, VehicleLocalPosition, VehicleCommand, TrajectorySetpoint
from vdt_msgs.msg import PlannerOutput, TimeoutFlags, RcChannelsRaw, RcFsmInput, OffboardStatus


class EngageSim(Node):
    def __init__(self, a):
        super().__init__('engage_sim')
        self.a = a
        self.armed = False
        self.nav_state = 4
        self.t0 = self.get_clock().now().nanoseconds / 1e9
        self.sp_count = 0
        self.last_active = None
        self.log = []

        self.pub_status = self.create_publisher(VehicleStatus, '/fmu/out/vehicle_status_v1', qos_profile_sensor_data)
        self.pub_pos = self.create_publisher(VehicleLocalPosition, '/fmu/out/vehicle_local_position', qos_profile_sensor_data)
        self.pub_fsm = self.create_publisher(UInt8, 'fsm/state', 10)
        self.pub_planner = self.create_publisher(PlannerOutput, 'planner/velocity_setpoint', 10)
        self.pub_flags = self.create_publisher(TimeoutFlags, 'input_cache/timeout_flags', 10)
        self.pub_rc = self.create_publisher(RcChannelsRaw, 'rc/channels_raw', 10)
        self.pub_rcfsm = self.create_publisher(RcFsmInput, 'rc/fsm_input', 10)
        killed_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)
        self.pub_killed = self.create_publisher(Bool, 'system/killed', killed_qos)
        self.pub_inhibit = self.create_publisher(Bool, 'safety/inhibit_offboard', 10)

        self.create_subscription(VehicleCommand, '/fmu/in/vehicle_command', self.on_cmd, qos_profile_sensor_data)
        self.create_subscription(TrajectorySetpoint, '/fmu/in/trajectory_setpoint', self.on_sp, qos_profile_sensor_data)
        self.create_subscription(OffboardStatus, 'offboard/status', self.on_status, 10)

        self.pub_killed.publish(Bool(data=False))
        self.create_timer(0.05, self.tick_fast)
        self.create_timer(1.0, self.tick_slow)

    def elapsed(self):
        return self.get_clock().now().nanoseconds / 1e9 - self.t0

    def on_cmd(self, m):
        if m.command == 176:
            if int(m.param2) == 6:
                self.nav_state = 14
                self.get_logger().info('PX4 <- OFFBOARD mode request: nav_state=14')
            elif int(m.param2) == 4:
                self.nav_state = 4
                self.get_logger().warn('PX4 <- LOITER (failsafe/reset)')
        elif m.command == 400:
            if m.param1 >= 0.5:
                if not self.armed:
                    self.get_logger().info('PX4 <- ARM from offboard_node')
                self.armed = True
            else:
                self.armed = False
                self.nav_state = 4
                self.get_logger().warn('PX4 <- DISARM')

    def on_sp(self, m):
        self.sp_count += 1

    def on_status(self, m):
        if m.offboard_active != self.last_active:
            self.last_active = m.offboard_active
            self.get_logger().info(f'offboard_active={m.offboard_active} @ {self.elapsed():.1f}s')

    def tick_fast(self):
        t = self.elapsed()
        if not self.armed and t >= self.a.arm_after and not self.a.no_arm:
            self.armed = True
            self.get_logger().info(f'manual ARM simulated @ {t:.1f}s')

        s = VehicleStatus()
        s.arming_state = VehicleStatus.ARMING_STATE_ARMED if self.armed else VehicleStatus.ARMING_STATE_DISARMED
        s.nav_state = self.nav_state
        s.failsafe = self.a.failsafe
        s.pre_flight_checks_pass = True
        self.pub_status.publish(s)

        p = VehicleLocalPosition()
        gps = not self.a.no_gps
        p.xy_valid = gps
        p.z_valid = gps
        p.z = -self.a.alt
        self.pub_pos.publish(p)

        self.pub_fsm.publish(UInt8(data=self.a.fsm_state))
        self.pub_planner.publish(PlannerOutput(vx=0.0, vy=0.0, vz=0.0, yaw=float('nan')))
        self.pub_flags.publish(TimeoutFlags())
        self.pub_inhibit.publish(Bool(data=self.a.inhibit))

        rc = RcChannelsRaw()
        ch = [1500] * 16
        ch[3] = 1000
        ch[4] = 1000
        rc.ch = ch
        rc.valid = True
        rc.failsafe = False
        self.pub_rc.publish(rc)
        self.pub_rcfsm.publish(RcFsmInput(land_switch=False, kill_switch=False))

    def tick_slow(self):
        self.get_logger().info(
            f't={self.elapsed():.0f}s armed={self.armed} nav_state={self.nav_state} '
            f'gps_ok={not self.a.no_gps} alt={self.a.alt} setpoints/s={self.sp_count}')
        self.sp_count = 0


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--arm-after', type=float, default=3.0)
    ap.add_argument('--no-arm', action='store_true')
    ap.add_argument('--no-gps', action='store_true')
    ap.add_argument('--alt', type=float, default=0.0)
    ap.add_argument('--fsm-state', type=int, default=0)
    ap.add_argument('--failsafe', action='store_true')
    ap.add_argument('--inhibit', action='store_true')
    a, ros_args = ap.parse_known_args()
    rclpy.init(args=ros_args)
    node = EngageSim(a)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


main()