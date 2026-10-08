#!/usr/bin/env python3
"""
Host GCS Operator Terminal for Precision Landing
Runs on Host Computer (Trạm mặt đất) connected to Mobile Hotspot Wi-Fi.
Allows operator to send Land commands, trigger emergency holds, and monitor mission status.
"""

import sys
import threading
import time

try:
    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import Bool, Float64, String
except ImportError:
    print("[ERROR] ROS 2 (rclpy) is not installed on this environment.")
    print("Please install ROS 2 Humble/Iron or source your ROS environment:")
    print("  source /opt/ros/humble/setup.bash")
    sys.exit(1)


class GCSOperatorTerminal(Node):
    def __init__(self):
        super().__init__('gcs_operator_terminal')

        # Status variables
        self.phase = "DISCONNECTED / UNKNOWN"
        self.tracking_mode = "UNKNOWN"
        self.safe_to_land = False
        self.uncertainty_2sigma = 0.0
        self.sub_phase = "N/A"
        self.touchdown = False

        # Publishers
        self.land_cmd_pub = self.create_publisher(Bool, '/operator/land_command', 10)

        # Subscribers
        self.create_subscription(String, '/mission/phase', self._phase_cb, 10)
        self.create_subscription(String, '/ekf/tracking_mode', self._tracking_cb, 10)
        self.create_subscription(Bool, '/landing/safe_to_land', self._safe_cb, 10)
        self.create_subscription(Float64, '/landing/uncertainty_radius', self._unc_cb, 10)
        self.create_subscription(String, '/landing/sub_phase', self._subphase_cb, 10)
        self.create_subscription(Bool, '/landing/touchdown', self._touchdown_cb, 10)

        self.get_logger().info("GCS Operator Terminal initialized successfully.")

    def _phase_cb(self, msg: String):
        self.phase = msg.data

    def _tracking_cb(self, msg: String):
        self.tracking_mode = msg.data

    def _safe_cb(self, msg: Bool):
        self.safe_to_land = msg.data

    def _unc_cb(self, msg: Float64):
        self.uncertainty_2sigma = msg.data

    def _subphase_cb(self, msg: String):
        self.sub_phase = msg.data

    def _touchdown_cb(self, msg: Bool):
        self.touchdown = msg.data

    def send_land_command(self):
        msg = Bool(data=True)
        self.land_cmd_pub.publish(msg)
        self.get_logger().info(">>> SENT LAND COMMAND (True) to /operator/land_command <<<")

    def send_abort_command(self):
        msg = Bool(data=False)
        self.land_cmd_pub.publish(msg)
        self.get_logger().warn(">>> SENT ABORT / HOLD COMMAND (False) to /operator/land_command <<<")

    def print_dashboard(self):
        safe_str = "SAFE (OK)" if self.safe_to_land else "UNSAFE / GATED"
        touch_str = "TOUCHDOWN LATCHED!" if self.touchdown else "IN AIR"
        sys.stdout.write(
            f"\r[STATUS] Phase: {self.phase:<12} | EKF: {self.tracking_mode:<10} | "
            f"Gate: {safe_str:<12} | 2σ: {self.uncertainty_2sigma:.3f}m | "
            f"SMC: {self.sub_phase:<12} | Contact: {touch_str:<18}"
        )
        sys.stdout.flush()


def terminal_input_loop(node: GCSOperatorTerminal):
    print("\n" + "=" * 70)
    print("🚁 VDT PRECISION LANDING — OPERATOR CONTROL CONSOLE (GCS)")
    print("=" * 70)
    print("Commands:")
    print("  [L] + Enter : Send LAND command (/operator/land_command = True)")
    print("  [A] + Enter : Send ABORT/HOLD command (/operator/land_command = False)")
    print("  [Q] + Enter : Quit GCS Terminal")
    print("=" * 70 + "\n")

    while rclpy.ok():
        try:
            cmd = input().strip().lower()
            if cmd in ('l', 'land'):
                node.send_land_command()
            elif cmd in ('a', 'abort', 'hold'):
                node.send_abort_command()
            elif cmd in ('q', 'quit', 'exit'):
                print("Exiting Operator Console...")
                break
        except (EOFError, KeyboardInterrupt):
            break


def main():
    rclpy.init()
    node = GCSOperatorTerminal()

    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    input_thread = threading.Thread(target=terminal_input_loop, args=(node,), daemon=True)
    input_thread.start()

    try:
        while rclpy.ok() and input_thread.is_alive():
            node.print_dashboard()
            time.sleep(0.2)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
