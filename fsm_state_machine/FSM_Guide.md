# FSM 4-State — Hướng dẫn cài đặt, hoạt động và debug

## 1. Package cần cài

### 1.1. Hệ thống (ROS2 + build tool)

```bash
sudo apt update
sudo apt install -y \
  ros-${ROS_DISTRO}-rclcpp \
  ros-${ROS_DISTRO}-std-msgs \
  ros-${ROS_DISTRO}-nav-msgs \
  ros-${ROS_DISTRO}-rosidl-default-generators \
  ros-${ROS_DISTRO}-rosidl-default-runtime \
  python3-colcon-common-extensions
```

`ROS_DISTRO` là bản ROS2 đang dùng (humble, iron, jazzy...). Kiểm tra bằng:

```bash
echo $ROS_DISTRO
```

### 1.2. Công cụ debug/quan sát (không bắt buộc nhưng nên có)

```bash
sudo apt install -y \
  ros-${ROS_DISTRO}-rqt \
  ros-${ROS_DISTRO}-rqt-graph \
  ros-${ROS_DISTRO}-rqt-topic \
  gdb
```

### 1.3. Build package

```bash
cd ~/ros2_ws
colcon build --packages-select fsm_state_machine
source install/setup.bash
```

---

## 2. Cách thức hoạt động

### 2.1. Kiến trúc tổng quan

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

### 2.2. Vòng đời một chu kỳ `update()`

1. Đọc dữ liệu mới nhất đã cache từ các subscriber (`ekf_state_`, `vision_state_`, `alt_state_`, `rc_input_`, `timeout_flags_`).
2. `build_sensor_input()` gộp dữ liệu thành `SensorInput` (marker_detected, align_error, altitude, delta_h, d_horiz, touchdown).
3. `sensor_validate()` ép `valid = false` và `marker_detected = false` nếu có timeout hoặc dữ liệu NaN.
4. `counters_update_marker_stable()` / `counters_update_marker_lost()` cập nhật số khung marker ổn định liên tiếp và thời gian mất marker.
5. `evaluate_transition()` gọi đúng hàm `check_*_transition` theo state hiện tại để quyết định state kế tiếp.
6. Nếu state đổi, reset counters và ghi `last_transition_time`.
7. Gọi đúng `action_*` tương ứng state hiện tại để publish lệnh điều khiển.
8. Publish `fsm/state` cho các module khác đọc read-only.
9. Ghi log debug nếu `debug_enabled = true`.

### 2.3. Bảng chuyển trạng thái

| State hiện tại | Điều kiện | State kế tiếp |
|---|---|---|
| SEARCH | `marker_stable_count >= 10` | FOLLOW |
| FOLLOW | `marker_lost_time > 5.0` | SEARCH |
| FOLLOW | `land_switch && marker_detected` | APPROACH |
| APPROACH | `!land_switch` | FOLLOW |
| APPROACH | `marker_lost_time > 3.0` | FOLLOW |
| APPROACH | `align_error < 0.3 && delta_h < land_entry_height` | LAND |
| LAND | `touchdown == true` | COMPLETE |

### 2.4. Tham số runtime (ROS2 parameter)

| Tên tham số | Mặc định | Ý nghĩa |
|---|---|---|
| `land_entry_height` | 0.5 | Ngưỡng delta_h để cho phép chuyển APPROACH -> LAND |
| `yaw_search_rate` | 0.3 | Tốc độ yaw khi ở SEARCH |
| `land_descent_rate` | 0.4 | Tốc độ hạ độ cao khi ở LAND |
| `debug_enabled` | false | Bật log chi tiết mỗi chu kỳ |

---

## 3. Cách debug FSM 4-state

### 3.1. Build ở chế độ Debug (symbol đầy đủ cho gdb, bật macro FSM_DEBUG_BUILD)

```bash
colcon build --packages-select fsm_state_machine \
  --cmake-args -DCMAKE_BUILD_TYPE=Debug
source install/setup.bash
```

### 3.2. Chạy node với log debug runtime bật sẵn

```bash
ros2 run fsm_state_machine fsm_node --ros-args -p debug_enabled:=true
```

Mỗi 100ms sẽ in ra một dòng dạng:

```
[INFO] [fsm_node]: state=1 stable=12 lost_t=0.00 align_err=0.045 alt=3.20 delta_h=1.10 land_sw=0
```

### 3.3. Chỉnh log level của ROS2 (không cần đổi tham số node)

```bash
ros2 run fsm_state_machine fsm_node --ros-args --log-level fsm_node:=debug
```

### 3.4. Kiểm tra state đang chạy

```bash
ros2 topic echo /fsm/state
```

### 3.5. Kiểm tra các lệnh FSM đang publish ra

```bash
ros2 topic echo /gimbal/state_request
ros2 topic echo /planner/mode
ros2 topic echo /planner/apf_gain
ros2 topic echo /cmd/yaw_rate
ros2 topic echo /cmd/vertical_descent_rate
ros2 topic echo /cmd/disarm_request
```

### 3.6. Giả lập input để test từng nhánh chuyển trạng thái mà chưa cần phần cứng thật

Giả lập marker luôn thấy để ép SEARCH -> FOLLOW:

```bash
ros2 topic pub /hpad/pose fsm_state_machine/msg/VisionMarker \
  "{marker_visible: true, pixel_align_error: 0.1}" -r 10
```

Giả lập RC bật land_switch để ép FOLLOW -> APPROACH:

```bash
ros2 topic pub /rc/fsm_input fsm_state_machine/msg/RcFsmInput \
  "{land_switch: true, kill_switch: false}" -r 10
```

Giả lập touchdown để ép LAND -> COMPLETE:

```bash
ros2 topic pub /alt_estimator/state fsm_state_machine/msg/AltEstimate \
  "{altitude: 0.05, touchdown_flag: true}" -r 10
```

### 3.7. Kiểm tra tham số đang chạy, chỉnh trực tiếp không cần restart node

```bash
ros2 param list /fsm_node
ros2 param get /fsm_node land_entry_height
ros2 param set /fsm_node debug_enabled true
```

### 3.8. Xem sơ đồ kết nối topic thực tế (đúng luồng dữ liệu chưa)

```bash
rqt_graph
```

### 3.9. Debug sâu bằng gdb khi nghi ngờ crash hoặc logic sai ở tầng C++

```bash
ros2 run --prefix 'gdb -ex run --args' fsm_state_machine fsm_node
```

Đặt breakpoint tại các hàm logic thuần (không phụ thuộc ROS, dễ trace nhất):

```
break fsm_state_machine::evaluate_transition
break fsm_state_machine::check_approach_transition
```

### 3.10. Kiểm tra timeout flags có đang ép sai state không

Nếu FSM đứng yên ở SEARCH dù marker luôn visible, khả năng cao `input_cache/timeout_flags` đang báo `vision_timeout = true`:

```bash
ros2 topic echo /input_cache/timeout_flags
```