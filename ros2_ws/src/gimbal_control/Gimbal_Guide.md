# Gimbal Control — Guide

## 1. Package cần cài đặt ngoài

- Package ROS 2 chuẩn: `rclcpp`, `std_msgs`.
- Package nội bộ:
  - `input_state_cache`: Để lấy định nghĩa message `InputSnapshot`.

## 2. Nguyên lý hoạt động

```text
[fsm_node]          ──► gimbal/state_request (phase hiện tại) ──\
                                                                ├─► gimbal_node (timer 20Hz)
[input_state_cache] ──► input_cache/snapshot (delta_h, d_horiz) ──/        │
                                                                           v
                                                            gimbal/target_angle_deg (Float32, độ)
                                                                           │
                                                                           v
                                                                     servo_control (MG90S)
```

Mỗi chu kỳ 50 ms (20 Hz):
1. **Tiếp nhận Snapshot:** Đọc trực tiếp $\Delta h$ (`delta_h`), khoảng cách ngang $d_{horiz}$ (`d_horiz`) và cờ hợp lệ `valid` từ bản tin `InputSnapshot`.
2. **Kiểm tra tính hợp lệ & Watchdog:** Nếu `valid == false` hoặc mất snapshot quá 200 ms $\rightarrow$ giữ nguyên góc điều khiển cũ, bỏ qua chu kỳ để tránh giật gimbal.
3. **Tính góc mục tiêu theo phase bay:**

| Phase | Trạng thái FSM | Công thức góc mục tiêu |
|---|---|---|
| SEARCH | 0 | 0 độ (hướng thẳng ngang tìm kiếm) |
| FOLLOW | 1 | $-\operatorname{atan2}(\Delta h, d_{horiz})$ đổi sang độ |
| APPROACH | 2 | $-\operatorname{atan2}(\Delta h, d_{horiz})$ đổi sang độ |
| LAND | 3 | Nội suy tuyến tính từ -60° đến -90° theo tỷ lệ $\Delta h / \text{land\_entry\_height}$ |
| COMPLETE | 4 | 0 độ (mặc định) |

4. **Làm mượt PID (Open-loop Smoothing):** PID đưa góc lệnh hiện tại tiệm cận góc mục tiêu một cách êm ái. Khâu tích phân (I) tích hợp anti-windup (dừng tích lũy khi đầu ra đã chạm ngưỡng bão hòa).
5. **Giới hạn gia tốc góc (Slew Rate Limiter):** Giới hạn tốc độ đổi góc tối đa mỗi giây theo `max_slew_rate_deg_s`.
6. **Publish góc điều khiển:** Xuất góc ra topic `gimbal/target_angle_deg`.

Khi đổi phase, bộ PID tự động reset `integral` và `prev_error` về 0. Toàn bộ tham số và dữ liệu đầu vào phải là số hữu hạn (`std::isfinite`), dữ liệu NaN hoặc Inf sẽ bị loại bỏ ngay lập tức.

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
| `kp`, `ki`, `kd` | 1.0, 0.0, 0.1 | Hệ số PID làm mượt góc |
| `out_min_deg`, `out_max_deg` | -90, 90 | Giới hạn góc xuất ra (độ) |
| `max_slew_rate_deg_s` | 60.0 | Tốc độ xoay tối đa (độ/giây) |
| `land_entry_height` | 0.5 | Ngưỡng delta_h dùng nội suy góc chúi khi LAND |
| `debug_enabled` | false | Bật log chi tiết mỗi chu kỳ |

## 4. Cách debug

Bật log runtime:

```bash
ros2 run gimbal_control gimbal_node --ros-args -p debug_enabled:=true
```

Mỗi chu kỳ in ra dạng:

```text
[INFO] [gimbal_node]: phase=1 target=-12.30 smoothed=-8.10 limited=-7.00
```

Xem góc output thực tế đang publish:

```bash
ros2 topic echo /gimbal/target_angle_deg
```

### Giả lập kiểm thử độc lập các góc chúi

1. **Giả lập dữ liệu Snapshot đầu vào (đang ở độ cao 2m, cách H-Pad 2m):**
   ```bash
   ros2 topic pub /input_cache/snapshot input_state_cache/msg/InputSnapshot \
     "{valid: true, marker_detected: true, altitude: 2.0, delta_h: 2.0, d_horiz: 2.0}" -r 20
   ```

2. **Giả lập lệnh chuyển Phase từ FSM:**
   * **Phase SEARCH (0):**
     ```bash
     ros2 topic pub /gimbal/state_request std_msgs/msg/UInt8 "{data: 0}" -r 10
     ```
     *(Góc mục tiêu = 0°)*
   * **Phase FOLLOW / APPROACH (1 hoặc 2):**
     ```bash
     ros2 topic pub /gimbal/state_request std_msgs/msg/UInt8 "{data: 1}" -r 10
     ```
     *($\Delta h = 2\text{m}, d_{horiz} = 2\text{m} \rightarrow$ Góc mục tiêu $\approx -45^\circ$)*
   * **Phase LAND (3):**
     ```bash
     ros2 topic pub /gimbal/state_request std_msgs/msg/UInt8 "{data: 3}" -r 10
     ```
     *(Camera chúi xuống từ -60° đến -90° khi hạ độ cao)*

3. **Kiểm tra trạng thái Snapshot đầu vào:**
   Nếu góc không thay đổi dù phase đã đổi, kiểm tra cờ `valid` của snapshot:
   ```bash
   ros2 topic echo /input_cache/snapshot
   ```

### Chạy unit test logic tự động:

```bash
cd ros2_ws
colcon test --packages-select gimbal_control --ctest-args -R gimbal_logic_test
```