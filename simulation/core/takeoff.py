#!/usr/bin/env python3
"""Lệnh cất cánh tự động (Auto-Takeoff) tới độ cao mong muốn (mặc định 3.0m) qua MAVLink.

Bỏ qua giới hạn cứng 10m của thanh trượt QGroundControl.
Sử dụng:
    python3 simulation_maps/takeoff.py
    python3 simulation_maps/takeoff.py --alt 3.0
"""

import argparse
import os
import sys
import time

os.environ['MAVLINK_DIALECT'] = 'common'
px4_mav_path = '/home/duy/VDT_project/PX4-Autopilot/src/modules/mavlink/mavlink'
if px4_mav_path not in sys.path:
    sys.path.insert(0, px4_mav_path)

from pymavlink import mavutil


def takeoff(target_alt=3.0):
    print(f"[Takeoff] Đang kết nối tới PX4 SITL qua MAVLink UDP (127.0.0.1:14540)...")
    try:
        master = mavutil.mavlink_connection('udp:127.0.0.1:14540')
        master.wait_heartbeat(timeout=5)
        print(f"[Takeoff] Đã kết nối với Drone (System ID: {master.target_system})")
    except Exception as e:
        print(f"[ERROR] Không thể kết nối tới PX4 SITL: {e}")
        print("Hãy đảm bảo PX4 SITL (Terminal 1) đang chạy.")
        return False

    # 1. Đặt tham số MIS_TAKEOFF_ALT về đúng target_alt
    print(f"[Takeoff] Cài đặt tham số PX4: MIS_TAKEOFF_ALT = {target_alt:.1f}m...")
    master.param_set_send('MIS_TAKEOFF_ALT', float(target_alt))
    time.sleep(0.5)

    # 2. Gửi lệnh Arm
    print("[Takeoff] Gửi lệnh ARM động cơ...")
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0,
        1.0, 0, 0, 0, 0, 0, 0
    )
    time.sleep(0.5)

    # 3. Gửi lệnh Takeoff
    # Trong PX4, param7 là độ cao tuyệt đối AMSL (Mean Sea Level). Nếu truyền 3.0m,
    # PX4 coi là 3m trên mực nước biển (trong khi mặt đất ở Zurich SITL là ~488m),
    # dẫn đến việc lệnh bị clamp về độ cao tối thiểu 1.5m.
    # Để cất cánh tương đối (relative altitude) chuẩn xác theo MIS_TAKEOFF_ALT,
    # param7 PHẢI là float('nan') để Navigator tự động tính: ground_alt + MIS_TAKEOFF_ALT!
    print(f"[Takeoff] Ra lệnh cất cánh lên độ cao tương đối {target_alt:.1f}m so với mặt đất...")
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
        0,
        -1.0, 0.0, 0.0, float('nan'), float('nan'), float('nan'),
        float('nan')  # Param 7: NaN để PX4 dùng độ cao tương đối từ MIS_TAKEOFF_ALT
    )
    print(f"✅ Lệnh cất cánh {target_alt:.1f}m đã được gửi thành công!")
    print("Drone đang cất cánh trong Gazebo. Bạn có thể quan sát trên QGroundControl.")
    return True


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description="Auto Takeoff script for PX4 SITL")
    parser.add_argument('--alt', type=float, default=3.0, help="Target altitude in meters (default: 3.0)")
    args = parser.parse_args()
    takeoff(args.alt)
