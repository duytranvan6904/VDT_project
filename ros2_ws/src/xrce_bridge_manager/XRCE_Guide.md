# XRCE Bridge Manager — Guide

## 1. Package cần cài đặt ngoài

Có. Cần binary `MicroXRCEAgent` — không có qua apt hay ROS2, phải build riêng từ repo `Micro-XRCE-DDS-Agent` và cài vào PATH hệ thống trước khi chạy node này.

## 2. Nguyên lý hoạt động

```
xrce_bridge_node (timer 1Hz)
        |
        v
  spawn/kiểm tra tiến trình MicroXRCEAgent (serial bridge)
        |
        v
PX4 (uxrce_dds_client) tự kết nối qua Agent
        |
        v
/fmu/out/vehicle_status --> xrce_bridge_node (theo dõi để suy ra còn kết nối hay không)
```

Lúc khởi động node:
1. Kiểm tra Agent đã chạy chưa (`pgrep`), nếu chưa thì spawn tiến trình `MicroXRCEAgent serial --dev <serial_port> -b <baudrate>`.

Mỗi chu kỳ 1s (`monitor_step`):
1. Kiểm tra `MicroXRCEAgent` do node quản lý còn sống (`poll()`) hoặc có tiến trình Agent bên ngoài.
2. Tính thời gian kể từ lần nhận `VehicleStatus` gần nhất.
3. Chỉ báo connected khi Agent còn sống và `VehicleStatus` còn fresh.
4. Nếu vượt `connection_timeout_sec` hoặc Agent đã chết → coi là mất kết nối, tăng `retry_count`, gọi lại `start_agent()` để khởi động lại tiến trình Agent.
3. Log debug nếu bật.

Lưu ý: participant/topic phía PX4 (`offboard_control_mode`, `trajectory_setpoint`, `vehicle_odometry`...) là do firmware PX4 tự tạo khi Agent sống — node này không tạo topic nào cả, chỉ giữ cho Agent luôn chạy.

## 3. Cách chạy

```bash
ros2 run xrce_bridge_manager xrce_bridge_node
```

Chạy kèm tham số tùy chỉnh:

```bash
ros2 run xrce_bridge_manager xrce_bridge_node --ros-args \
  -p serial_port:=/dev/ttyAMA0 \
  -p baudrate:=921600 \
  -p connection_timeout_sec:=2.0
```

Danh sách tham số:

| Tên | Mặc định | Ý nghĩa |
|---|---|---|
| `serial_port` | /dev/ttyAMA0 | Cổng serial nối tới PX4 |
| `baudrate` | 921600 | Tốc độ baud của Agent |
| `connection_timeout_sec` | 2.0 | Thời gian tối đa không nhận `VehicleStatus` trước khi coi là mất kết nối |
| `debug_enabled` | false | Bật log mỗi chu kỳ |

## 4. Cách debug

Bật log runtime:

```bash
ros2 run xrce_bridge_manager xrce_bridge_node --ros-args -p debug_enabled:=true
```

Mỗi chu kỳ in ra dạng:

```
[INFO] [xrce_bridge_node]: connected=True retry_count=0
```

Kiểm tra tiến trình Agent có đang chạy thật không (ngoài ROS2):

```bash
pgrep -af MicroXRCEAgent
```

Kiểm tra có đang nhận dữ liệu từ PX4 không (nếu topic này có message tức Agent + client đã bắt tay thành công):

```bash
ros2 topic echo /fmu/out/vehicle_status
```

Nếu `connected` cứ nhảy `False` liên tục dù Agent vẫn sống: khả năng cao sai `serial_port`, `baudrate` không khớp cấu hình `uxrce_dds_client`, hoặc `VehicleStatus` publish thưa hơn timeout. Kiểm tra bằng `param show UXRCE_DDS_*` qua QGroundControl/MAVLink console và tăng timeout phù hợp.

Nếu Agent liên tục bị spawn lại (retry_count tăng đều dù dây cắm ổn định): kiểm tra `connection_timeout_sec` có đang đặt quá nhỏ so với tần số publish thật của `vehicle_status` (PX4 mặc định publish khá chậm, thường dưới 1Hz) — nên tăng lên ít nhất 3-5s để tránh false positive.

## Unit test

Test mock `pgrep` và `Popen`, không khởi chạy MicroXRCEAgent thật:

```bash
python3 -m pip install pytest
python3 -m pytest ros2_ws/src/xrce_bridge_manager/test/test_xrce_logic.py
```