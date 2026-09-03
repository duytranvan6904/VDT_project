import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from gpiozero import AngularServo
from gpiozero.pins.lgpio import LGPIOFactory

class ServoNode(Node):
    def __init__(self):
        super().__init__('servo_node')
        self.declare_parameter('gpio_pin', 18)
        pin = self.get_parameter('gpio_pin').value

        factory = LGPIOFactory()
        self.servo = AngularServo(
            pin,
            min_angle=0,
            max_angle=180,
            min_pulse_width=0.0006,
            max_pulse_width=0.0024,
            pin_factory=factory
        )

        self.sub = self.create_subscription(
            Float32, 'servo_angle', self.angle_cb, 10
        )

    def angle_cb(self, msg):
        angle = max(0.0, min(180.0, msg.data))
        self.servo.angle = angle

def main():
    rclpy.init()
    node = ServoNode()
    rclpy.spin(node)
    node.servo.detach()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()