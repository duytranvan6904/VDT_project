# Input State Cache — Guide

## 1. Package cần cài đặt ngoài

- Package chuẩn ROS 2: `rclcpp`, `nav_msgs`.
- Tạm thời phụ thuộc `fsm_state_machine` để lấy định nghĩa `VisionMarker` và `AltEstimate` (trước khi 2 message này được chuyển về đúng package sản sinh).
- Package tự sở hữu và biên dịch 2 message:
  - `input_state_cache/msg/InputSnapshot`: Bản tin snapshot trạng thái cảm biến đồng bộ.
  - `input_state_cache/msg/TimeoutFlags`: Bản tin cờ timeout (hỗ trợ tương thích ngược).

## 2. Nguyên lý hoạt động

```text
hpad/state_filtered   --\
hpad/pose             --\
alt_estimator/state   --> input_cache_node (timer 20Hz)
planner/velocity_sp   --/       │
                                ├─► input_cache/snapshot
                                │   (valid, marker_detected, align_error,
                                │    altitude, delta_h, d_horiz, touchdown,
                                │    planner_timeout)
                                │
                                └─► input_cache/timeout_flags
                                    (ekf_timeout, vision_timeout,
                                     alt_timeout, planner_timeout)
```

Node hoạt động như một **Bộ tập hợp trạng thái (State Aggregator)** trung tâm:

1. **Lưu trữ bộ đệm (Buffering):** Ghi nhận payload và thời điểm nhận gần nhất của từng nguồn cảm biến (`EKF`, `Vision`, `Altimeter`, `Planner`).
2. **Kiểm tra độ tươi (Watchdog):** So sánh `now - last_time` với ngưỡng timeout tương ứng cho từng cảm biến.
3. **Kiểm chuẩn dữ liệu (Sanitization):** Kiểm tra `isnan()` và `isinf()` trên từng kênh dữ liệu thô. Nếu cảm biến bị timeout hoặc có giá trị NaN, cờ `valid` của cảm biến đó bị hủy.
4. **Tính toán hình học tương đối (Computed Geometry):**
   - Chênh cao: $\Delta h = z_{ekf} - altitude$
   - Khoảng cách mặt phẳng ngang: $d_{horiz} = \sqrt{x_{ekf}^2 + y_{ekf}^2}$
5. **Phát hành nguyên tử (Atomic Publish):** Ở chu kỳ 50 ms (20 Hz), phát hành bản tin `InputSnapshot` duy nhất cho `fsm_node` và `gimbal_node` sử dụng đồng bộ.

## 3. Cách chạy

```bash
ros2 run input_state_cache input_cache_node
```

Chạy kèm tham số tùy chỉnh:

```bash
ros2 run input_state_cache input_cache_node --ros-args \
  -p ekf_timeout_sec:=0.5 \
  -p vision_timeout_sec:=0.5 \
  -p alt_timeout_sec:=0.5 \
  -p planner_timeout_sec:=1.0
```

Danh sách tham số:

| Tên | Mặc định | Ý nghĩa |
|---|---|---|
| `ekf_timeout_sec` | 0.5 | Thời gian tối đa không nhận `hpad/state_filtered` |
| `vision_timeout_sec` | 0.5 | Thời gian tối đa không nhận `hpad/pose` |
| `alt_timeout_sec` | 0.5 | Thời gian tối đa không nhận `alt_estimator/state` |
| `planner_timeout_sec` | 1.0 | Thời gian tối đa không nhận `planner/velocity_setpoint` |
| `debug_enabled` | false | Bật log chi tiết mỗi chu kỳ |

## 4. Cách debug

Bật log runtime:

```bash
ros2 run input_state_cache input_cache_node --ros-args -p debug_enabled:=true
```

Mỗi chu kỳ in ra dạng:

```text
[INFO] [input_cache_node]: valid=1 marker=1 alt=1.20 delta_h=0.50 d_horiz=0.80 ekf_to=0 vis_to=0 alt_to=0
```

Xem nội dung snapshot phát ra:

```bash
ros2 topic echo /input_cache/snapshot
```

Xem cờ timeout tương thích ngược:

```bash
ros2 topic echo /input_cache/timeout_flags
```

Kiểm tra tần số xuất bản:

```bash
ros2 topic hz /input_cache/snapshot
```

## 5. Xử lý sự cố thường gặp

1. **Trường `valid = false` liên tục dù publisher vẫn gửi dữ liệu:**
   - Kiểm tra tên topic nguồn có khớp chính xác: `hpad/state_filtered`, `hpad/pose`, `alt_estimator/state`.
   - Kiểm tra dữ liệu cảm biến đầu vào có chứa giá trị `NaN` hoặc `Inf` hay không.
2. **`fsm_node` hoặc `gimbal_node` rơi vào Fail-safe:**
   - Kiểm tra tần số xuất bản của `input_cache/snapshot`. Nếu chu kỳ gửi vượt quá 200 ms, cơ chế Watchdog tại `fsm_node` sẽ tự động kích hoạt chế độ an toàn do nghi ngờ node cache bị treo/sập.