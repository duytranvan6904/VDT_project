# Offboard Manager — Guide

## 1. Package cần cài đặt ngoài

Có. Cần package `px4_msgs` — không có sẵn qua apt, phải clone và build từ nguồn PX4-ROS2 bridge vào trong workspace trước khi build `offboard_manager`. Phiên bản `px4_msgs` phải khớp với firmware PX4 đang chạy.

Phụ thuộc `vdt_msgs` để lấy định nghĩa `TimeoutFlags`.

## 2. Nguyên lý hoạt động

```text
fsm/state (UInt8)                 --\
planner/velocity_setpoint          --\
(PlannerOutput, ENU)                --> OffboardNode (timer 20Hz)
input_cache/timeout_flags          --/          |
/fmu/out/vehicle_status            --/          |
/fmu/out/vehicle_local_position    --/          v
        /fmu/in/offboard_control_mode, /fmu/in/trajectory_setpoint,
        /fmu/in/vehicle_command
```

### Quy ước frame của `PlannerOutput`

`PlannerOutput` dùng frame thế giới ENU. Node tự đổi sang NED của PX4 trước khi gửi:

| Trường | Ý nghĩa (ENU) | Đổi sang PX4 (NED) |
|---|---|---|
| `vx` | vận tốc hướng Đông (m/s) | `vy_ned = vx` |
| `vy` | vận tốc hướng Bắc (m/s) | `vx_ned = vy` |
| `vz` | vận tốc hướng lên (m/s) | `vz_ned = -vz` |
| `yaw` | heading (rad), 0 = hướng Đông, ngược chiều kim đồng hồ là dương | `yaw_ned = wrap(pi/2 - yaw)` về khoảng `[-pi, pi]` |

`yaw = NaN` nghĩa là giữ nguyên yaw hiện tại.

### Kiểm tra và giới hạn planner output

- `vx`, `vy`: giới hạn mặc định `+/-2.0 m/s` (`max_horizontal_velocity`).
- `vz`: giới hạn mặc định `+/-1.0 m/s` (`max_vertical_velocity`).
- `yaw`: không bị giới hạn, được wrap về `[-pi, pi]` sau khi đổi sang NED.
- `vx`, `vy`, `vz` là NaN hoặc vô hạn: dùng vận tốc 0 và giữ yaw hiện tại.
- `yaw` là NaN: vận tốc vẫn được dùng, yaw giữ nguyên.

### Planner stale

Planner được coi là stale khi một trong hai điều kiện đúng:

- `input_cache/timeout_flags.planner_timeout = true` (mặc định là `true` cho đến khi nhận được bản tin đầu tiên).
- Không nhận được `planner/velocity_setpoint` trong `planner_timeout_sec` (mặc định 1.0 s).

Planner stale chỉ ảnh hưởng FOLLOW và APPROACH (UAV hover, giữ yaw). Chuỗi engage, SEARCH và LAND không phụ thuộc planner.

### Cất cánh

Node không cất cánh. Cất cánh bằng PX4 takeoff trên QGroundControl, sau đó node tự engage khi UAV đã đủ độ cao (xem bên dưới).

### Chuỗi engage

Mỗi chu kỳ 50 ms, khi chưa `offboard_active`:

0. Kiểm tra điều kiện đã bay (airborne): `VehicleStatus` fresh và đang `ARMED`, `VehicleLocalPosition` fresh với `z_valid`, và độ cao `-z >= min_engage_altitude_m` (mặc định 2.0 m). Nếu không đạt thì reset chuỗi engage và không gửi gì cho PX4.
1. Gửi heartbeat (`offboard_control_mode`, position=false, velocity=true).
2. Gửi setpoint SEARCH (velocity 0, yaw_rate = `yaw_search_rate`).
3. Tăng `engage_counter`.
4. Đủ `required_engage_cycles` (mặc định 10) thì gửi yêu cầu chuyển OFFBOARD và chờ `VehicleStatus.nav_state == OFFBOARD` từ message còn fresh.
5. Khi mode đã xác nhận, chờ `VehicleLocalPosition` và `VehicleStatus` fresh, EKF có `xy_valid/z_valid` và PX4 không ở failsafe.
6. Gửi `VEHICLE_CMD_COMPONENT_ARM_DISARM` (arm), chờ PX4 báo `ARMED`, rồi đặt `offboard_active = true`.

`VehicleStatus` và `VehicleLocalPosition` được coi là fresh nếu đã nhận ít nhất một lần và thời gian từ lần nhận gần nhất không vượt `data_freshness_timeout_sec` (mặc định 1.0 giây). Nếu mode hoặc health confirmation timeout, chuỗi engage được reset và không arm.

Khi `system/killed` hoặc `safety/inhibit_offboard` là `true`, node reset chuỗi engage và ngừng publish setpoint.

### Khi đã `offboard_active`

1. Gửi heartbeat.
2. Watchdog kiểm tra heartbeat gần nhất có bị trễ quá `watchdog_timeout_sec` (mặc định 0.5 s) không. Nếu trễ, vào failsafe (set mode HOLD/LOITER, `offboard_active = false`) và bỏ qua bước 3.
3. Build setpoint theo `fsm_state` hiện tại rồi publish:

| FSM state | Setpoint (NED) |
|---|---|
| SEARCH | velocity 0, yaw_rate = `yaw_search_rate` |
| FOLLOW | velocity và yaw lấy từ `planner_output` (đã đổi sang NED); planner stale thì velocity 0, giữ yaw |
| APPROACH | giống FOLLOW |
| LAND | velocity ngang 0, `vz = +land_descent_rate` (dương là xuống), giữ yaw |
| COMPLETE hoặc state khác | velocity 0, yaw_rate 0 |

Watchdog ở đây tự giám sát vòng lặp gửi heartbeat của chính node, không phải theo dõi heartbeat từ PX4. Với timer cố định 50 ms thì gần như không bao giờ trigger trừ khi loop bị treo.

## 3. Cách chạy

```bash
ros2 run offboard_manager offboard_node
```

Chạy kèm tham số tùy chỉnh:

```bash
ros2 run offboard_manager offboard_node --ros-args \
  -p required_engage_cycles:=10 \
  -p mode_confirm_timeout_cycles:=20 \
  -p health_confirm_timeout_cycles:=100 \
  -p data_freshness_timeout_sec:=1.0 \
  -p watchdog_timeout_sec:=0.5 \
  -p planner_timeout_sec:=1.0 \
  -p min_engage_altitude_m:=2.0 \
  -p yaw_search_rate:=0.3 \
  -p land_descent_rate:=0.4
```

Danh sách tham số:

| Tên | Mặc định | Ý nghĩa |
|---|---|---|
| `required_engage_cycles` | 10 | Số chu kỳ gửi setpoint SEARCH trước khi chuyển hẳn sang OFFBOARD + arm |
| `mode_confirm_timeout_cycles` | 20 | Số chu kỳ chờ PX4 xác nhận `nav_state=OFFBOARD` |
| `health_confirm_timeout_cycles` | 100 | Số chu kỳ chờ health/readiness trước khi arm |
| `data_freshness_timeout_sec` | 1.0 | Tuổi tối đa của `VehicleStatus` và `VehicleLocalPosition` |
| `max_horizontal_velocity` | 2.0 | Giới hạn trị tuyệt đối vận tốc ngang từ planner |
| `max_vertical_velocity` | 1.0 | Giới hạn trị tuyệt đối vận tốc dọc từ planner |
| `watchdog_timeout_sec` | 0.5 | Thời gian tối đa cho phép giữa 2 lần heartbeat trước khi vào failsafe |
| `planner_timeout_sec` | 1.0 | Thời gian tối đa không nhận `planner/velocity_setpoint` trước khi coi planner là stale |
| `min_engage_altitude_m` | 2.0 | Độ cao tối thiểu (m, tính theo `-z` của local position) để bắt đầu chuỗi engage |
| `yaw_search_rate` | 0.3 | Tốc độ yaw (rad/s) dùng cho setpoint SEARCH |
| `land_descent_rate` | 0.4 | Tốc độ hạ độ cao (m/s) dùng cho setpoint LAND |
| `debug_enabled` | false | Bật log mỗi chu kỳ |

Tham số `max_yaw` đã bị loại bỏ. Nếu file yaml cấu hình còn dòng này thì có thể xóa.

## 4. Cách debug

Bật log runtime:

```bash
ros2 run offboard_manager offboard_node --ros-args -p debug_enabled:=true
```

Mỗi chu kỳ in ra dạng:

```text
[INFO] [offboard_node]: phase=2 active=1 armed=1 nav_state=14 px4_ready=1
```

Xem các lệnh đang gửi ra PX4:

```bash
ros2 topic echo /fmu/in/offboard_control_mode
ros2 topic echo /fmu/in/trajectory_setpoint
ros2 topic echo /fmu/in/vehicle_command
```

Xem độ cao và trạng thái arm mà node dùng để quyết định engage:

```bash
ros2 topic echo /fmu/out/vehicle_local_position --field z
ros2 topic echo /fmu/out/vehicle_status --field arming_state
```

Giả lập fsm_state, cờ timeout và planner output để test từng nhánh setpoint mà chưa cần FSM/planner thật (planner output ở frame ENU):

```bash
ros2 topic pub /fsm/state std_msgs/msg/UInt8 "{data: 1}" -r 10
```

```bash
ros2 topic pub /input_cache/timeout_flags input_state_cache/msg/TimeoutFlags \
  "{ekf_timeout: false, vision_timeout: false, alt_timeout: false, planner_timeout: false}" -r 10
```

```bash
ros2 topic pub /planner/velocity_setpoint offboard_manager/msg/PlannerOutput \
  "{vx: 0.5, vy: 0.0, vz: 0.0, yaw: 0.0}" -r 10
```

Giữ nguyên yaw hiện tại (yaw NaN):

```bash
ros2 topic pub /planner/velocity_setpoint offboard_manager/msg/PlannerOutput \
  "{vx: 0.5, vy: 0.0, vz: 0.0, yaw: .nan}" -r 10
```

Kết quả kỳ vọng với `vx: 0.5` (hướng Đông, ENU): trong `/fmu/in/trajectory_setpoint`, `velocity[1] = 0.5` (hướng Đông, NED) và `yaw = 1.5708`.

Nếu node không bao giờ engage:

- Kiểm tra UAV đã `ARMED` và độ cao `-z` đã đạt `min_engage_altitude_m` (cất cánh bằng PX4 takeoff trên QGroundControl trước).
- Kiểm tra `VehicleStatus` và `VehicleLocalPosition` có đang nhận được (`z_valid`).
- Kiểm tra `system/killed` và `safety/inhibit_offboard` không phải `true`.

Nếu node kẹt mãi ở giai đoạn engage sau khi đã đủ điều kiện: kiểm tra `engage_counter` có tăng đều mỗi chu kỳ không qua log debug. Nếu không tăng, khả năng cao timer bị treo hoặc `required_engage_cycles` bị set quá lớn.

Nếu vừa vào OFFBOARD xong lại rớt về failsafe ngay: kiểm tra `watchdog_timeout_sec` có đang đặt quá nhỏ so với chu kỳ timer (50 ms) không.

Nếu UAV đứng yên (hover) ở FOLLOW hoặc APPROACH: planner đang bị coi là stale. Kiểm tra `planner/velocity_setpoint` có được publish liên tục và `input_cache/timeout_flags.planner_timeout` có phải `false` không.

Nếu UAV bay lệch hướng hoặc quay sai chiều: kiểm tra `PlannerOutput` có đang ở frame ENU không (`vx` Đông, `vy` Bắc, `vz` lên, `yaw` ENU), vì node tự đổi sang NED.

### Chạy unit test logic tự động:

```bash
cd ros2_ws
colcon test --packages-select offboard_manager --ctest-args -R offboard_logic_test
```