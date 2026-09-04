import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import VehicleStatus

from xrce_bridge_manager.xrce_logic import agent_is_running, spawn_agent


class XrceBridgeNode(Node):
    def __init__(self):
        super().__init__('xrce_bridge_node')

        self.serial_port = self.declare_parameter('serial_port', '/dev/ttyAMA0').value
        self.baudrate = self.declare_parameter('baudrate', 921600).value
        self.connection_timeout_sec = self.declare_parameter('connection_timeout_sec', 2.0).value
        self.debug_enabled = self.declare_parameter('debug_enabled', False).value

        self.agent_process = None
        self.last_status_time = 0.0
        self.retry_count = 0
        self.connected = False

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=5
        )
        self.create_subscription(
            VehicleStatus, '/fmu/out/vehicle_status', self.status_cb, qos_profile
        )

        self.start_agent()
        self.create_timer(1.0, self.monitor_step)

        self.get_logger().info(f'XRCE bridge manager started on {self.serial_port}')

    def status_cb(self, msg):
        self.last_status_time = time.monotonic()

    def start_agent(self):
        if agent_is_running(self.serial_port):
            return
        self.agent_process = spawn_agent(self.serial_port, self.baudrate)

    def check_connection(self):
        elapsed = time.monotonic() - self.last_status_time
        return elapsed < self.connection_timeout_sec

    def reconnect(self):
        self.retry_count += 1
        self.start_agent()

    def monitor_step(self):
        self.connected = self.check_connection()
        if not self.connected:
            self.reconnect()
        self.log_debug()

    def log_debug(self):
        if not self.debug_enabled:
            return
        self.get_logger().info(
            f'connected={self.connected} retry_count={self.retry_count}'
        )

    def destroy_node(self):
        if self.agent_process is not None and self.agent_process.poll() is None:
            self.agent_process.terminate()
        super().destroy_node()


def main():
    rclpy.init()
    node = XrceBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()