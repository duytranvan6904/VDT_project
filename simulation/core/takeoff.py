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


def _wait_command_ack(master, command_id: int, timeout_s: float):
    deadline = time.monotonic() + max(0.0, timeout_s)
    while time.monotonic() < deadline:
        msg = master.recv_match(
            type='COMMAND_ACK', blocking=True,
            timeout=min(0.5, max(0.05, deadline - time.monotonic())),
        )
        if msg is not None and int(msg.command) == int(command_id):
            return msg
    return None


def _wait_armed(master, timeout_s: float) -> bool:
    deadline = time.monotonic() + max(0.0, timeout_s)
    while time.monotonic() < deadline:
        heartbeat = master.recv_match(
            type='HEARTBEAT', blocking=True,
            timeout=min(0.5, max(0.05, deadline - time.monotonic())),
        )
        if heartbeat is None or heartbeat.get_srcSystem() != master.target_system:
            continue
        if heartbeat.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED:
            return True
    return False


def _wait_for_climb(master, start_relative_alt_m: float | None, target_alt: float, timeout_s: float) -> bool:
    deadline = time.monotonic() + max(0.0, timeout_s)
    start_alt = start_relative_alt_m
    while time.monotonic() < deadline:
        msg = master.recv_match(
            type=['GLOBAL_POSITION_INT', 'LOCAL_POSITION_NED', 'HEARTBEAT'],
            blocking=True,
            timeout=min(0.5, max(0.05, deadline - time.monotonic())),
        )
        if msg is None:
            continue
        if msg.get_type() == 'GLOBAL_POSITION_INT':
            current_alt = float(msg.relative_alt) * 1e-3
        elif msg.get_type() == 'LOCAL_POSITION_NED':
            current_alt = -float(msg.z)
        else:
            continue

        if start_alt is None:
            start_alt = current_alt
        if current_alt - start_alt >= max(0.5, 0.35 * float(target_alt)):
            print(f"[Takeoff] PX4 confirms climb: relative altitude {current_alt:.2f}m.")
            return True
    return False


def takeoff(target_alt=3.0):
    os.environ['MAVLINK_DIALECT'] = 'common'
    print(f"[Takeoff] Đang kết nối tới PX4 SITL qua MAVLink UDP (127.0.0.1:14540)...")
    try:
        master = mavutil.mavlink_connection('udp:127.0.0.1:14540')
        heartbeat = master.wait_heartbeat(timeout=10)
        if heartbeat is None:
            print('[ERROR] PX4 heartbeat timeout; refusing to report takeoff success.')
            master.close()
            return False
        print(f"[Takeoff] Đã kết nối với Drone (System ID: {master.target_system})")
    except Exception as e:
        print(f"[ERROR] Không thể kết nối tới PX4 SITL: {e}")
        print("Hãy đảm bảo PX4 SITL đang chạy.")
        return False

    # 1. Đặt tham số MIS_TAKEOFF_ALT về đúng target_alt
    print(f"[Takeoff] Cài đặt tham số PX4: MIS_TAKEOFF_ALT = {target_alt:.1f}m...")
    master.param_set_send('MIS_TAKEOFF_ALT', float(target_alt))
    time.sleep(0.5)

    # PX4 may reject arming briefly while SITL sensors/preflight checks settle.
    # Retry only after observing the command acknowledgement or armed heartbeat.
    print('[Takeoff] Chờ PX4 xác nhận ARM...')
    arm_command = mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM
    arm_deadline = time.monotonic() + 25.0
    armed = False
    while time.monotonic() < arm_deadline:
        master.mav.command_long_send(
            master.target_system, master.target_component,
            arm_command, 0, 1.0, 0, 0, 0, 0, 0, 0,
        )
        ack = _wait_command_ack(master, arm_command, 2.0)
        if ack is not None and ack.result != mavutil.mavlink.MAV_RESULT_ACCEPTED:
            print(f'[Takeoff] ARM bị từ chối (COMMAND_ACK result={ack.result}); sẽ thử lại.')
        armed = _wait_armed(master, 1.0)
        if armed:
            break
        time.sleep(0.5)

    if not armed:
        print('[ERROR] PX4 không xác nhận trạng thái armed sau 25s.')
        master.close()
        return False

    # 3. Gửi lệnh Takeoff
    print(f"[Takeoff] Ra lệnh cất cánh lên độ cao tương đối {target_alt:.1f}m so với mặt đất...")
    start_position = master.recv_match(
        type='GLOBAL_POSITION_INT', blocking=True, timeout=2.0,
    )
    start_alt = (
        float(start_position.relative_alt) * 1e-3
        if start_position is not None else None
    )
    takeoff_command = mavutil.mavlink.MAV_CMD_NAV_TAKEOFF
    master.mav.command_long_send(
        master.target_system, master.target_component,
        takeoff_command, 0, -1.0, 0.0, 0.0,
        float('nan'), float('nan'), float('nan'), float('nan'),
    )
    ack = _wait_command_ack(master, takeoff_command, 5.0)
    if ack is None or ack.result != mavutil.mavlink.MAV_RESULT_ACCEPTED:
        result = 'no COMMAND_ACK' if ack is None else f'COMMAND_ACK result={ack.result}'
        print(f'[ERROR] PX4 did not accept NAV_TAKEOFF ({result}).')
        master.close()
        return False

    climbed = _wait_for_climb(master, start_alt, target_alt, 25.0)
    master.close()
    if not climbed:
        print('[ERROR] Takeoff command was accepted, but PX4 did not confirm a climb within 25s.')
        return False

    print(f"✅ PX4 confirmed takeoff toward {target_alt:.1f}m.")
    print("Drone đang cất cánh trong Gazebo. Bạn có thể quan sát trên QGroundControl.")
    return True


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description="Auto Takeoff script for PX4 SITL")
    parser.add_argument('--alt', type=float, default=3.0, help="Target altitude in meters (default: 3.0)")
    args = parser.parse_args()
    takeoff(args.alt)
