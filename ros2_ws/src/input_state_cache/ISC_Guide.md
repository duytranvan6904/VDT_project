# Input State Cache — Guide

## 1. Package cần cài đặt ngoài

Không có. Chỉ dùng rclcpp, nav_msgs, và hai package tự viết `fsm_state_machine` (msg `VisionMarker`, `AltEstimate`, `TimeoutFlags`) và `offboard_manager` (msg `PlannerOutput`).

## 2. Nguyên lý hoạt động

```
hpad/state_filtered   --\
hpad/pose               --\
alt_estimator/state       --> input_cache_node (timer 20Hz)
planner/velocity_setpoint --/
                                    |
                                    v
                        input_cache/timeout_flags
                    (ekf_timeout, vision_timeout,
                     alt_timeout, planner_timeout)
```

Node chỉ ghi lại **thời điểm nhận message gần nhất** cho từng topic (không lưu nội dung — các node khác đã tự cache nội dung của chúng rồi). Mỗi chu kỳ 50ms:

1. So sánh thời gian hiện tại với thời điểm nhận gần nhất của từng topic.
2. Nếu chênh lệch vượt ngưỡng tương ứng → gắn cờ timeout đó là `true`.
3. Publish `TimeoutFlags` ra `input_cache/timeout_flags` cho `fsm_node` (đọc ekf + vision), `gimbal_node` (đọc ekf + alt), và tương lai là `offboard_node` (đọc planner) dùng chung.

Không có bước "cache dữ liệu ekf/vision/alt/planner" như pseudocode gốc mô tả — vì các node tiêu thụ đã tự giữ bản riêng của chúng, gom lại đây chỉ dư thừa.

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
| `debug_enabled` | false | Bật log mỗi chu kỳ |

## 4. Cách debug

Bật log runtime:

```bash
ros2 run input_state_cache input_cache_node --ros-args -p debug_enabled:=true
```

Mỗi chu kỳ in ra dạng:

```
[INFO] [input_cache_node]: ekf=0 vision=0 alt=0 planner=1
```

Xem cờ timeout đang publish ra:

```bash
ros2 topic echo /input_cache/timeout_flags
```

Giả lập một nguồn bị mất tín hiệu (ví dụ tắt thử node vision, hoặc không publish gì lên `hpad/pose`) rồi quan sát `vision_timeout` có chuyển `true` đúng sau khoảng `vision_timeout_sec` không:

```bash
ros2 topic hz /hpad/pose
```

Nếu một flag cứ đứng yên `true` dù publisher vẫn chạy: kiểm tra đúng tên topic node đó đang publish có khớp với tên node này đang subscribe không (`hpad/state_filtered`, `hpad/pose`, `alt_estimator/state`, `planner/velocity_setpoint`) — sai tên topic là nguyên nhân phổ biến nhất.

Nếu `fsm_node`/`gimbal_node` vẫn báo hành vi như timeout dù `input_cache/timeout_flags` đang toàn `false`: kiểm tra `ekf_timeout_sec`/`alt_timeout_sec` ở đây có đang đặt nhỏ hơn `frame_timeout` tương ứng bên `fsm_node`/`gimbal_node`, gây timeout giả do lệch cấu hình giữa hai bên.

## Test

Package hiện chưa có test executable riêng. Có thể kiểm tra behavior bằng cách tắt từng publisher và quan sát:

```bash
ros2 topic echo /input_cache/timeout_flags
ros2 topic hz /hpad/state_filtered
ros2 topic hz /hpad/pose
ros2 topic hz /alt_estimator/state
ros2 topic hz /planner/velocity_setpoint
```