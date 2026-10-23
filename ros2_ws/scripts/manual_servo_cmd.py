#!/usr/bin/env python3
"""
Manual Servo Command CLI
Gửi lệnh thủ công để điều khiển góc chúi của Gimbal / Servo camera.
Góc 0°: ngang, góc dương (ví dụ 30°, 45°, 90°): chúi xuống đất.
"""

import sys
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32


def main():
    if len(sys.argv) < 2:
        print("\nCách dùng:")
        print("  python3 manual_servo_cmd.py <góc_độ>")
        print("Ví dụ:")
        print("  python3 manual_servo_cmd.py 0     # Nhìn ngang")
        print("  python3 manual_servo_cmd.py 30    # Chúi xuống 30 độ (tìm marker)")
        print("  python3 manual_servo_cmd.py 45    # Chúi xuống 45 độ")
        print("  python3 manual_servo_cmd.py 90    # Chúi thẳng xuống đất 90 độ\n")
        sys.exit(1)

    try:
        target_angle = float(sys.argv[1])
    except ValueError:
        print(f"Lỗi: '{sys.argv[1]}' không phải là số hợp lệ.")
        sys.exit(1)

    rclpy.init()
    node = Node('manual_servo_cmd')
    pub = node.create_publisher(Float32, 'gimbal/target_angle_deg', 10)

    # Gửi lệnh liên tục trong 1 giây để đảm bảo servo_node bắt được và không bị watchdog
    msg = Float32()
    msg.data = target_angle
    print(f">> Đang gửi góc gimbal: {target_angle:.1f}° tới topic 'gimbal/target_angle_deg'...")

    for _ in range(10):
        pub.publish(msg)
        rclpy.spin_once(node, timeout_sec=0.1)

    print(">> Đã gửi xong lệnh!")
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
