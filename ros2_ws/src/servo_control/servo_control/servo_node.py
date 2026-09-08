import math
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from gpiozero import AngularServo
from gpiozero.pins.lgpio import LGPIOFactory

from servo_control.servo_logic import angle_to_pwm_us, control_angle_to_servo_angle


class ServoNode(Node):
    def __init__(self):
        super().__init__('servo_node')

        self.gpio_pin = self.declare_parameter('gpio_pin', 18).value
        self.pwm_min_us = self.declare_parameter('pwm_min_us', 600).value
        self.pwm_max_us = self.declare_parameter('pwm_max_us', 2400).value
        self.angle_min_deg = self.declare_parameter('angle_min_deg', 0.0).value
        self.angle_max_deg = self.declare_parameter('angle_max_deg', 180.0).value
        self.home_angle_deg = self.declare_parameter('home_angle_deg', 90.0).value
        self.input_timeout_sec = self.declare_parameter('input_timeout_sec', 1.0).value
        self.debug_enabled = self.declare_parameter('debug_enabled', False).value

        factory = LGPIOFactory()
        self.servo = AngularServo(
            self.gpio_pin,
            min_angle=self.angle_min_deg,
            max_angle=self.angle_max_deg,
            min_pulse_width=self.pwm_min_us / 1e6,
            max_pulse_width=self.pwm_max_us / 1e6,
            pin_factory=factory
        )

        self.current_angle_deg = self.home_angle_deg
        self.current_pwm_us = 0.0
        self.last_command_time = None
        self.timed_out = False
        self.write_angle(self.home_angle_deg)

        self.sub = self.create_subscription(
            Float32, 'gimbal/target_angle_deg', self.angle_cb, 10
        )
        self.create_timer(0.1, self.watchdog_cb)

    def write_angle(self, servo_angle_deg):
        self.servo.angle = servo_angle_deg
        self.current_angle_deg = servo_angle_deg
        self.current_pwm_us = angle_to_pwm_us(
            servo_angle_deg, self.angle_min_deg, self.angle_max_deg,
            self.pwm_min_us, self.pwm_max_us
        )
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
                self.write_angle(self.home_angle_deg)
                self.timed_out = True

    def log_debug(self):
        if not self.debug_enabled:
            return
        self.get_logger().info(
            f'angle={self.current_angle_deg:.2f} pwm_us={self.current_pwm_us:.1f}'
        )


def main():
    rclpy.init()
    node = ServoNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.servo.detach()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()