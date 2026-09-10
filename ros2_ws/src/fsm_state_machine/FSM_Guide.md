# FSM 5-State — Guide

## 1. Package cần cài đặt ngoài

Không có. Chỉ dùng các package ROS2 chuẩn (rclcpp, std_msgs, nav_msgs, rosidl_default_generators).

## 2. Nguyên lý hoạt động

```
[EKF]        --> hpad/state_filtered   --\
[Vision]     --> hpad/pose             --> FsmNode (timer 100ms)
[Alt Est.]   --> alt_estimator/state   --/       |
[RC]         --> rc/fsm_input          --/       |
[Input Cache]--> input_cache/timeout_flags        v
                                            FsmActuators (publish)
                                                  |
                          cmd/yaw_rate, gimbal/state_request,
                          planner/mode, planner/apf_gain,
                          gimbal/align_error_cmd,
                          cmd/vertical_descent_rate,
                          cmd/disarm_request, fsm/state
```

Mỗi chu kỳ 100ms:
1. Gộp dữ liệu cache thành `SensorInput` (marker_detected, align_error, altitude, delta_h, d_horiz, touchdown).
2. `sensor_validate()` ép `valid = false` và `marker_detected = false` nếu timeout hoặc dữ liệu NaN.
3. Cập nhật `marker_stable_count` và `marker_lost_time`.
4. Kiểm tra điều kiện chuyển state theo bảng dưới.
5. Nếu state đổi, reset counters, ghi `last_transition_time`.
6. Gọi `action_*` tương ứng state hiện tại để publish lệnh điều khiển.
7. Publish `fsm/state` cho các module khác đọc read-only.

`safety/force_land = true` có ưu tiên cao nhất và chuyển trực tiếp SEARCH/FOLLOW/APPROACH sang LAND. Khi `planner_timeout = true`, FSM đưa FOLLOW/APPROACH về SEARCH để không tiếp tục tiến theo planner stale.

| State hiện tại | Điều kiện | State kế tiếp |
|---|---|---|
| SEARCH | `marker_stable_count >= 10` | FOLLOW |
| FOLLOW | `marker_lost_time > 5.0` | SEARCH |
| FOLLOW | `land_switch && marker_detected` | APPROACH |
| APPROACH | `!land_switch` | FOLLOW |
| APPROACH | `marker_lost_time > 3.0` | FOLLOW |
| APPROACH | `align_error < 0.3 && delta_h < land_entry_height` | LAND |
| SEARCH/FOLLOW/APPROACH | `safety/force_land == true` | LAND |
| FOLLOW/APPROACH | `planner_timeout == true` | SEARCH |
| LAND | `touchdown == true` | COMPLETE |

## 3. Cách chạy

```bash
ros2 run fsm_state_machine fsm_node
```

Chạy kèm tham số tùy chỉnh:

```bash
ros2 run fsm_state_machine fsm_node --ros-args \
  -p land_entry_height:=0.6 \
  -p yaw_search_rate:=0.3 \
  -p land_descent_rate:=0.4
```

Danh sách tham số:

| Tên | Mặc định | Ý nghĩa |
|---|---|---|
| `land_entry_height` | 0.5 | Ngưỡng delta_h để cho phép chuyển APPROACH -> LAND |
| `yaw_search_rate` | 0.3 | Tốc độ yaw khi ở SEARCH |
| `land_descent_rate` | 0.4 | Tốc độ hạ độ cao khi ở LAND |
| `debug_enabled` | false | Bật log chi tiết mỗi chu kỳ |

## 4. Cách debug

Bật log runtime:

```bash
ros2 run fsm_state_machine fsm_node --ros-args -p debug_enabled:=true
```

Mỗi chu kỳ in ra dạng:

```
[INFO] [fsm_node]: state=1 stable=12 lost_t=0.00 align_err=0.045 alt=3.20 delta_h=1.10 land_sw=0
```

Xem state đang chạy:

```bash
ros2 topic echo /fsm/state
```

Xem các lệnh FSM đang publish ra:

```bash
ros2 topic echo /gimbal/state_request
ros2 topic echo /planner/mode
ros2 topic echo /cmd/yaw_rate
ros2 topic echo /cmd/vertical_descent_rate
ros2 topic echo /cmd/disarm_request
```

Giả lập input để test từng nhánh chuyển trạng thái mà chưa cần phần cứng thật:

```bash
# ép SEARCH -> FOLLOW
ros2 topic pub /hpad/pose fsm_state_machine/msg/VisionMarker \
  "{marker_visible: true, pixel_align_error: 0.1}" -r 10

# ép FOLLOW -> APPROACH
ros2 topic pub /rc/fsm_input fsm_state_machine/msg/RcFsmInput \
  "{land_switch: true, kill_switch: false}" -r 10

# ép LAND -> COMPLETE
ros2 topic pub /alt_estimator/state fsm_state_machine/msg/AltEstimate \
  "{altitude: 0.05, touchdown_flag: true}" -r 10
```

Nếu FSM đứng yên ở SEARCH dù marker luôn visible, kiểm tra timeout flags có đang ép sai không:

```bash
ros2 topic echo /input_cache/timeout_flags
```

Test logic tự động:

```bash
cd ros2_ws
colcon test --packages-select fsm_state_machine --ctest-args -R fsm_logic_test
```