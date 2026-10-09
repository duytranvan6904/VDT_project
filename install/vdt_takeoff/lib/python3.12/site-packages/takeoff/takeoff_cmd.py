import argparse
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                       ReliabilityPolicy, qos_profile_sensor_data)
from rclpy.utilities import remove_ros_args
from std_msgs.msg import Bool
from px4_msgs.msg import VehicleCommand, VehicleLocalPosition, VehicleStatus

PUB_QOS = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                     durability=DurabilityPolicy.TRANSIENT_LOCAL,
                     history=HistoryPolicy.KEEP_LAST, depth=1)
LATCH_QOS = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                       durability=DurabilityPolicy.TRANSIENT_LOCAL,
                       history=HistoryPolicy.KEEP_LAST, depth=1)


class Takeoff(Node):
    def __init__(self, alt, climb_timeout):
        super().__init__('takeoff_cmd')
        self.alt = alt
        self.climb_timeout = climb_timeout
        self.status = None
        self.local = None
        self.killed = False
        self.pub = self.create_publisher(
            VehicleCommand, '/fmu/in/vehicle_command', PUB_QOS)
        self.create_subscription(
            VehicleStatus, '/fmu/out/vehicle_status',
            self.on_status, qos_profile_sensor_data)
        self.create_subscription(
            VehicleLocalPosition, '/fmu/out/vehicle_local_position',
            self.on_local, qos_profile_sensor_data)
        self.create_subscription(Bool, 'system/killed', self.on_killed, LATCH_QOS)

    def on_status(self, m):
        self.status = m

    def on_local(self, m):
        self.local = m

    def on_killed(self, m):
        self.killed = m.data

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

    def wait(self, cond, timeout):
        t0 = time.monotonic()
        while time.monotonic() - t0 < timeout:
            rclpy.spin_once(self, timeout_sec=0.05)
            if cond():
                return True
        return False

    def armed(self):
        return self.status.arming_state == VehicleStatus.ARMING_STATE_ARMED

    def height(self):
        return -self.local.z if self.local.z_valid else float('nan')

    def ready(self):
        if self.status.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD:
            return True
        return self.height() >= 0.9 * self.alt

    def fail(self, msg):
        self.get_logger().error(msg)
        return False

    def run(self):
        if not self.wait(lambda: self.status and self.local, 10.0):
            return self.fail('Không nhận được vehicle_status/local_position')
        if self.killed:
            return self.fail('Hệ thống đang ở trạng thái killed')
        if self.armed():
            return self.fail('UAV đã armed, bỏ qua lệnh')

        self.send(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM,
                  [1, 0, 0, 0, 0, 0, 0])
        if not self.wait(self.armed, 5.0):
            return self.fail('Arm không thành công, kiểm tra preflight trên QGC')

        nan = float('nan')
        self.send(VehicleCommand.VEHICLE_CMD_NAV_TAKEOFF,
                  [-1, 0, 0, nan, nan, nan, nan])
        if not self.wait(lambda: self.ready() or self.killed, self.climb_timeout):
            self.send(VehicleCommand.VEHICLE_CMD_NAV_LAND,
                      [0, 0, 0, nan, nan, nan, nan])
            return self.fail('Không đạt độ cao, đã gửi lệnh hạ cánh')
        if self.killed:
            return self.fail('Nhận system/killed trong lúc cất cánh')

        self.get_logger().info(
            f'Đã đạt {self.height():.1f} m, nav_state={self.status.nav_state}')
        return True


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--alt', type=float, default=3.0)
    p.add_argument('--climb-timeout', type=float, default=20.0)
    a = p.parse_args(remove_ros_args(sys.argv)[1:])
    rclpy.init()
    node = Takeoff(a.alt, a.climb_timeout)
    ok = node.run()
    node.destroy_node()
    rclpy.shutdown()
    sys.exit(0 if ok else 1)


if __name__ == '__main__':
    main()
