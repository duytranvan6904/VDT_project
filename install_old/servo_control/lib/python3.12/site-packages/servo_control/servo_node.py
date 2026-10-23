import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from std_msgs.msg import Float32
from px4_msgs.msg import VehicleCommand

from servo_control.servo_logic import (
    angle_to_pwm_us,
    control_angle_to_servo_angle,
    servo_angle_to_value,
)


class ServoNode(Node):
    def __init__(self):
        super().__init__('servo_node')

        self.servo_function = float(self.declare_parameter('servo_function', 33.0).value)
        self.command_timeout_sec = float(self.declare_parameter('command_timeout_sec', 2.5).value)
        self.min_send_interval_sec = float(self.declare_parameter('min_send_interval_sec', 0.1).value)
        self.min_send_delta = float(self.declare_parameter('min_send_delta', 0.01).value)
        self.keepalive_sec = float(self.declare_parameter('keepalive_sec', 1.5).value)
        self.pwm_min_us = float(self.declare_parameter('pwm_min_us', 1000).value)
        self.pwm_max_us = float(self.declare_parameter('pwm_max_us', 2000).value)
        self.angle_min_deg = float(self.declare_parameter('angle_min_deg', 0.0).value)
        self.angle_max_deg = float(self.declare_parameter('angle_max_deg', 180.0).value)
        self.home_angle_deg = float(self.declare_parameter('home_angle_deg', 90.0).value)
        self.input_timeout_sec = float(self.declare_parameter('input_timeout_sec', 1.0).value)
        self.debug_enabled = self.declare_parameter('debug_enabled', False).value

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.cmd_pub = self.create_publisher(VehicleCommand, '/fmu/in/vehicle_command', qos)

        self.current_angle_deg = self.home_angle_deg
        self.current_value = 0.0
        self.current_pwm_us = 0.0
        self.last_sent_value = None
        self.last_sent_time = 0.0
        self.last_command_time = None
        self.timed_out = False
        self.write_angle(self.home_angle_deg, force=True)

        self.sub = self.create_subscription(
            Float32, 'gimbal/target_angle_deg', self.angle_cb, 10
        )
        self.create_timer(0.1, self.watchdog_cb)

    def send_value(self, value):
        m = VehicleCommand()
        m.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        m.command = 310
        m.param1 = float(value)
        m.param2 = self.command_timeout_sec
        m.param5 = self.servo_function
        m.target_system = 1
        m.target_component = 1
        m.source_system = 1
        m.source_component = 1
        m.from_external = True
        self.cmd_pub.publish(m)

    def write_angle(self, servo_angle_deg, force=False):
        value = servo_angle_to_value(servo_angle_deg, self.angle_min_deg, self.angle_max_deg)
        self.current_angle_deg = servo_angle_deg
        self.current_value = value
        self.current_pwm_us = angle_to_pwm_us(
            servo_angle_deg, self.angle_min_deg, self.angle_max_deg,
            self.pwm_min_us, self.pwm_max_us
        )
        now = time.monotonic()
        elapsed = now - self.last_sent_time
        changed = self.last_sent_value is None or abs(value - self.last_sent_value) >= self.min_send_delta
        due = elapsed >= self.min_send_interval_sec
        alive = self.keepalive_sec > 0.0 and elapsed >= self.keepalive_sec
        if force or (changed and due) or alive:
            self.send_value(value)
            self.last_sent_value = value
            self.last_sent_time = now
        self.log_debug()

    def angle_cb(self, msg):
        if not math.isfinite(msg.data):
            return
        servo_angle = control_angle_to_servo_angle(
            msg.data, self.home_angle_deg, self.angle_min_deg, self.angle_max_deg
        )
        self.write_angle(servo_angle)
        self.last_command_time = time.monotonic()
        self.timed_out = False

    def watchdog_cb(self):
        if self.last_command_time is None:
            return
        if time.monotonic() - self.last_command_time > self.input_timeout_sec:
            if not self.timed_out:
                self.write_angle(self.home_angle_deg, force=True)
                self.timed_out = True

    def release(self):
        for _ in range(5):
            self.send_value(float('nan'))
            time.sleep(0.05)

    def log_debug(self):
        if not self.debug_enabled:
            return
        self.get_logger().info(
            f'angle={self.current_angle_deg:.2f} value={self.current_value:.3f} pwm_us={self.current_pwm_us:.1f}'
        )


def main():
    rclpy.init()
    node = ServoNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.release()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()