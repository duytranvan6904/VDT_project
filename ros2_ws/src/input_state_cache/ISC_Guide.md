# Input State Cache — Guide

## 1. Package cần cài đặt ngoài

- Package chuẩn ROS 2: `rclcpp`, `nav_msgs`, `std_msgs`.
- Phụ thuộc `vdt_msgs` để lấy định nghĩa `VisionMarker`, `AltEstimate`, `InputSnapshot` và `TimeoutFlags`.

## 2. Nguyên lý hoạt động

```text
hpad/state_filtered   --\   (Odometry, frame world ENU, vị trí H-Pad)
ekf/tracking_mode     --\   (String)
odom                  --\   (Odometry, frame world ENU, vị trí UAV)
vision/marker         --> input_cache_node (timer 20Hz)
alt_estimator/state   --/       │
planner/velocity_sp   --/       ├─► input_cache/snapshot
                                │   (valid, marker_detected, align_error,
                                │    altitude, delta_h, d_horiz, touchdown,
                                │    planner_timeout)
                                │
                                └─► input_cache/timeout_flags
                                    (ekf_timeout, vision_timeout,
                                     alt_timeout, planner_timeout)
```

Node hoạt động như một **Bộ tập hợp trạng thái (State Aggregator)** trung tâm:

1. **Lưu trữ bộ đệm (Buffering):** Ghi nhận payload và thời điểm nhận gần nhất của từng nguồn (`EKF`, `Tracking mode`, `Odom`, `Vision`, `Altimeter`, `Planner`).
2. **Kiểm tra frame (Frame check):** Bản tin `hpad/state_filtered` và `odom` có `header.frame_id` khác tham số `world_frame` bị loại bỏ và không cập nhật thời điểm nhận. Đặt `world_frame` rỗng để tắt kiểm tra.
3. **Kiểm tra độ tươi (Watchdog):** So sánh `now - last_time` với ngưỡng timeout tương ứng cho từng nguồn. Nguồn chưa nhận lần nào được coi là timeout.
4. **Kiểm chuẩn dữ liệu (Sanitization):** Kiểm tra `isnan()` và `isinf()` trên từng kênh dữ liệu thô.
5. **Xác định tính hợp lệ từng nguồn:**
   - `ekf_valid`: `hpad/state_filtered` còn tươi, `ekf/tracking_mode` còn tươi và thuộc `TRACKING`, `PREDICTING` hoặc `PREDICTING_DEGRADED`, vị trí hữu hạn. Mode `EXPIRED` hoặc chuỗi không nhận dạng được bị coi là không hợp lệ.
   - `odom_valid`: `odom` còn tươi và vị trí hữu hạn.
   - `alt_valid`: `alt_estimator/state` còn tươi và `altitude` hữu hạn.
   - `marker_detected`: `vision/marker` còn tươi, `marker_visible = true` và `pixel_align_error` hữu hạn.
   - `touchdown`: `alt_valid` và `touchdown_flag = true`.
6. **Tính toán hình học tương đối (Computed Geometry):** Chỉ thực hiện khi `ekf_valid` và `odom_valid` cùng đúng, cả hai trong frame `world`:
   - $dx = x_{ekf} - x_{odom}$, $dy = y_{ekf} - y_{odom}$
   - Khoảng cách mặt phẳng ngang: $d_{horiz} = \sqrt{dx^2 + dy^2}$
   - Chênh cao (UAV cao hơn H-Pad là dương): $\Delta h = z_{odom} - z_{ekf}$
   - Sai số căn chỉnh (mét): $align\_error = d_{horiz}$
   - Khi không đủ điều kiện, `delta_h`, `d_horiz`, `align_error` là `NaN`.
7. **Phát hành nguyên tử (Atomic Publish):** Ở chu kỳ 50 ms (20 Hz), phát hành bản tin `InputSnapshot` duy nhất cho `fsm_node` và `gimbal_node` sử dụng đồng bộ.

### Ý nghĩa các trường của `InputSnapshot`

| Trường | Ý nghĩa |
|---|---|
| `valid` | `odom_valid && alt_valid`. Chỉ mô tả trạng thái UAV, không phụ thuộc việc có mục tiêu hay không |
| `marker_detected` | Marker đang được nhìn thấy và dữ liệu vision còn tươi |
| `align_error` | Khoảng cách ngang UAV tới H-Pad (m), `NaN` nếu không có mục tiêu hợp lệ |
| `altitude` | Độ cao từ `alt_estimator/state`, `NaN` nếu không hợp lệ |
| `delta_h` | Chênh cao UAV so với H-Pad (m), `NaN` nếu không có mục tiêu hợp lệ |
| `d_horiz` | Khoảng cách ngang UAV tới H-Pad (m), `NaN` nếu không có mục tiêu hợp lệ |
| `touchdown` | Đã chạm đất và `alt_estimator` còn tươi |
| `planner_timeout` | Chưa từng nhận hoặc quá `planner_timeout_sec` kể từ lần nhận cuối `planner/velocity_setpoint` |

### Ý nghĩa các trường của `TimeoutFlags`

| Trường | Điều kiện |
|---|---|
| `ekf_timeout` | `hpad/state_filtered` hoặc `ekf/tracking_mode` quá hạn, hoặc mode là `EXPIRED` |
| `vision_timeout` | `vision/marker` quá hạn |
| `alt_timeout` | `alt_estimator/state` quá hạn |
| `planner_timeout` | Giống `InputSnapshot.planner_timeout` |

## 3. Cách chạy

```bash
ros2 run input_state_cache input_cache_node --ros-args \
  -r hpad/state_filtered:=/ekf/target_state
```

Chạy kèm tham số tùy chỉnh:

```bash
ros2 run input_state_cache input_cache_node --ros-args \
  -r hpad/state_filtered:=/ekf/target_state \
  -p ekf_timeout_sec:=0.5 \
  -p odom_timeout_sec:=0.5 \
  -p vision_timeout_sec:=0.5 \
  -p alt_timeout_sec:=0.5 \
  -p planner_timeout_sec:=1.0 \
  -p world_frame:=world
```

Danh sách tham số:

| Tên | Mặc định | Ý nghĩa |
|---|---|---|
| `ekf_timeout_sec` | 0.5 | Thời gian tối đa không nhận `hpad/state_filtered` và `ekf/tracking_mode` |
| `odom_timeout_sec` | 0.5 | Thời gian tối đa không nhận `odom` |
| `vision_timeout_sec` | 0.5 | Thời gian tối đa không nhận `vision/marker` |
| `alt_timeout_sec` | 0.5 | Thời gian tối đa không nhận `alt_estimator/state` |
| `planner_timeout_sec` | 1.0 | Thời gian tối đa không nhận `planner/velocity_setpoint` |
| `world_frame` | `world` | `frame_id` bắt buộc của `hpad/state_filtered` và `odom`; rỗng để tắt kiểm tra |
| `debug_enabled` | false | Bật log chi tiết mỗi chu kỳ |

Danh sách topic đầu vào:

| Topic | Kiểu | QoS | Nguồn |
|---|---|---|---|
| `hpad/state_filtered` | `nav_msgs/Odometry` | reliable | `/ekf/target_state` của vision (remap) |
| `ekf/tracking_mode` | `std_msgs/String` | reliable | EKF của vision |
| `odom` | `nav_msgs/Odometry` | best effort | `px4_state_bridge` |
| `vision/marker` | `fsm_state_machine/VisionMarker` | reliable | `vision_interface_bridge` |
| `alt_estimator/state` | `fsm_state_machine/AltEstimate` | reliable | `px4_state_bridge` |
| `planner/velocity_setpoint` | `offboard_manager/PlannerOutput` | reliable | planner |

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

Kiểm tra dữ liệu đầu vào:

```bash
ros2 topic echo /ekf/tracking_mode
ros2 topic echo /odom --field pose.pose.position
ros2 topic echo /vision/marker
ros2 topic echo /alt_estimator/state
```

### Chạy unit test logic tự động:

```bash
cd ros2_ws
colcon test --packages-select input_state_cache --ctest-args -R input_cache_logic_test
```

## 5. Xử lý sự cố thường gặp

1. **`valid = false` liên tục dù publisher vẫn gửi dữ liệu:**
   - `valid` chỉ phụ thuộc `odom` và `alt_estimator/state`. Kiểm tra hai topic này có nhận được, còn tươi và không chứa `NaN` hoặc `Inf`.
   - Kiểm tra tên topic nguồn khớp chính xác: `odom`, `alt_estimator/state`.
2. **`delta_h`, `d_horiz`, `align_error` là `NaN`:**
   - Chưa có mục tiêu hợp lệ. Kiểm tra `ekf/tracking_mode` có phải `EXPIRED` hoặc không nhận được hay không.
   - Kiểm tra remap `hpad/state_filtered:=/ekf/target_state` và `odom` có đang được publish.
   - Kiểm tra log cảnh báo `frame_id ... dropped`: `frame_id` của `hpad/state_filtered` và `odom` phải trùng `world_frame`.
3. **`marker_detected = false` dù marker đang trong khung hình:**
   - Kiểm tra `vision/marker` có được publish (node `vision_interface_bridge`) và `pixel_align_error` hữu hạn.
   - Kiểm tra `vision_timeout_sec`.
4. **`touchdown` không bao giờ lên `true`:**
   - Kiểm tra `alt_estimator/state` còn tươi và `touchdown_flag` có được bridge đặt khi PX4 báo đã hạ cánh.
5. **`planner_timeout = true` liên tục:**
   - Planner chưa publish hoặc publish chậm hơn `planner_timeout_sec`. Planner cần publish liên tục kể cả khi chưa có mục tiêu.
6. **`fsm_node` hoặc `gimbal_node` rơi vào Fail-safe:**
   - Kiểm tra tần số xuất bản của `input_cache/snapshot`. Nếu chu kỳ gửi vượt quá 200 ms, cơ chế Watchdog tại `fsm_node` sẽ tự động kích hoạt chế độ an toàn do nghi ngờ node cache bị treo/sập.