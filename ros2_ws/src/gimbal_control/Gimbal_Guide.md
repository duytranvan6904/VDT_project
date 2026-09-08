# Gimbal Control — Guide

## 1. Package cần cài đặt ngoài

Không có. Chỉ dùng các package ROS2 chuẩn (rclcpp, std_msgs, nav_msgs) và package tự viết `fsm_state_machine` (để lấy lại msg `AltEstimate`, `TimeoutFlags`).

## 2. Nguyên lý hoạt động

```
FSM --> gimbal/state_request (phase hiện tại)
EKF --> hpad/state_filtered (x, y, z)
Alt estimator --> alt_estimator/state (altitude, touchdown)
Timeout flags --> input_cache/timeout_flags
        |
        v
   gimbal_node
        |
        v
gimbal/target_angle_deg (Float32, độ)
        |
        v
  code PWM servo của bạn
```

Mỗi chu kỳ:
1. Tính `delta_h = ekf.z - alt.altitude`, `d_horiz = hypot(ekf.x, ekf.y)`.
2. Nếu `ekf_timeout` hoặc `alt_timeout` bật → giữ nguyên góc cũ, bỏ qua chu kỳ.
3. Tính góc mục tiêu theo phase:

| Phase | Công thức |
|---|---|
| SEARCH | 0 độ |
| FOLLOW | `-atan2(delta_h, d_horiz)` |
| APPROACH | giống FOLLOW |
| LAND | nội suy -60 đến -90 độ theo `delta_h / land_entry_height` |
| COMPLETE | 0 độ (mặc định) |

4. PID làm mượt từ góc hiện tại tới góc mục tiêu (không phải closed-loop thật vì servo không feedback). Integral có anti-windup: không tích phân thêm khi output đã bão hòa theo hướng lỗi.
5. Slew rate limiter giới hạn tốc độ đổi góc tối đa mỗi giây.
6. Publish góc cuối ra `gimbal/target_angle_deg`.

Đổi phase thì PID tự reset integral và prev_error về 0.

Telemetry, `dt`, hệ số PID và kết quả trung gian phải là finite. Dữ liệu NaN/vô hạn hoặc `dt <= 0` sẽ bỏ qua chu kỳ và reset PID khi cần. `land_entry_height` phải lớn hơn 0; giá trị không hợp lệ dùng góc bắt đầu LAND (`-60` độ) thay vì chia cho 0.

## 3. Cách chạy

```bash
ros2 run gimbal_control gimbal_node
```

Chạy kèm tham số tùy chỉnh:

```bash
ros2 run gimbal_control gimbal_node --ros-args \
  -p kp:=1.2 -p ki:=0.0 -p kd:=0.15 \
  -p max_slew_rate_deg_s:=45.0 \
  -p land_entry_height:=0.6
```

Danh sách tham số:

| Tên | Mặc định | Ý nghĩa |
|---|---|---|
| `kp`, `ki`, `kd` | 1.0, 0.0, 0.1 | Hệ số PID |
| `out_min_deg`, `out_max_deg` | -90, 90 | Giới hạn output PID |
| `max_slew_rate_deg_s` | 60.0 | Tốc độ xoay tối đa (độ/giây) |
| `land_entry_height` | 0.5 | Ngưỡng delta_h dùng nội suy góc LAND |
| `debug_enabled` | false | Bật log mỗi chu kỳ |

## 4. Cách debug

Bật log runtime:

```bash
ros2 run gimbal_control gimbal_node --ros-args -p debug_enabled:=true
```

Mỗi chu kỳ in ra dạng:

```
[INFO] [gimbal_node]: phase=1 target=-12.30 smoothed=-8.10 limited=-7.00
```

Xem góc output thực tế đang publish:

```bash
ros2 topic echo /gimbal/target_angle_deg
```

Giả lập phase để test riêng từng nhánh công thức góc:

```bash
ros2 topic pub /gimbal/state_request std_msgs/msg/UInt8 "{data: 3}" -r 10
```
(0=SEARCH, 1=FOLLOW, 2=APPROACH, 3=LAND, 4=COMPLETE)

Nếu góc không đổi dù phase đổi: kiểm tra `input_cache/timeout_flags` có đang báo `ekf_timeout` hoặc `alt_timeout` = true hay không.

```bash
ros2 topic echo /input_cache/timeout_flags
```