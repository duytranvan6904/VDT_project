# Offboard Manager — Guide

## 1. Package cần cài đặt ngoài

Có. Cần package `px4_msgs` — không có sẵn qua apt, phải clone và build từ nguồn PX4-ROS2 bridge vào trong workspace trước khi build `offboard_manager`.

## 2. Nguyên lý hoạt động

```
fsm/state (UInt8)              --\
planner/velocity_setpoint       --> OffboardNode (timer 20Hz)
(PlannerOutput: vx, vy, vz, yaw) --/
                                          |
                                          v
        /fmu/in/offboard_control_mode, /fmu/in/trajectory_setpoint,
        /fmu/in/vehicle_command
```

Mỗi chu kỳ 50ms:

**Nếu chưa offboard_active (đang trong chuỗi engage):**
1. Gửi heartbeat (`offboard_control_mode`, position=true, velocity=true).
2. Gửi setpoint SEARCH (velocity 0, yaw_rate = `yaw_search_rate`).
3. Tăng `engage_counter`.
4. Đủ `required_engage_cycles` (mặc định 10) → gửi `VEHICLE_CMD_DO_SET_MODE` (OFFBOARD) + `VEHICLE_CMD_COMPONENT_ARM_DISARM` (arm), đặt `offboard_active = true`.

**Nếu đã offboard_active:**
1. Gửi heartbeat.
2. Watchdog kiểm tra heartbeat gần nhất có bị trễ quá `watchdog_timeout_sec` (mặc định 0.5s) không — nếu trễ, vào failsafe (set mode HOLD, `offboard_active = false`) và bỏ qua bước 3.
3. Build setpoint theo `fsm_state` hiện tại rồi publish:

| FSM state | Setpoint |
|---|---|
| SEARCH | velocity 0, yaw_rate = `yaw_search_rate` |
| FOLLOW | velocity + yaw lấy từ `planner_output` |
| APPROACH | giống FOLLOW |
| LAND | velocity ngang 0, vz = `land_descent_rate`, yaw từ `planner_output` |

Watchdog ở đây tự giám sát vòng lặp gửi heartbeat của chính node, không phải theo dõi heartbeat từ PX4 — với timer cố định 50ms thì gần như không bao giờ trigger trừ khi loop bị treo.

## 3. Cách chạy

```bash
ros2 run offboard_manager offboard_node
```

Chạy kèm tham số tùy chỉnh:

```bash
ros2 run offboard_manager offboard_node --ros-args \
  -p required_engage_cycles:=10 \
  -p watchdog_timeout_sec:=0.5 \
  -p yaw_search_rate:=0.3 \
  -p land_descent_rate:=0.4
```

Danh sách tham số:

| Tên | Mặc định | Ý nghĩa |
|---|---|---|
| `required_engage_cycles` | 10 | Số chu kỳ gửi setpoint SEARCH trước khi chuyển hẳn sang OFFBOARD + arm |
| `watchdog_timeout_sec` | 0.5 | Thời gian tối đa cho phép giữa 2 lần heartbeat trước khi vào failsafe |
| `yaw_search_rate` | 0.3 | Tốc độ yaw dùng cho setpoint SEARCH |
| `land_descent_rate` | 0.4 | Tốc độ hạ độ cao dùng cho setpoint LAND |
| `debug_enabled` | false | Bật log mỗi chu kỳ |

## 4. Cách debug

Bật log runtime:

```bash
ros2 run offboard_manager offboard_node --ros-args -p debug_enabled:=true
```

Mỗi chu kỳ in ra dạng:

```
[INFO] [offboard_node]: active=1 armed=1 engage_cnt=10 vx=0.50 vy=0.00 vz=-0.20
```

Xem các lệnh đang gửi ra PX4:

```bash
ros2 topic echo /fmu/in/offboard_control_mode
ros2 topic echo /fmu/in/trajectory_setpoint
ros2 topic echo /fmu/in/vehicle_command
```

Giả lập fsm_state và planner output để test từng nhánh setpoint mà chưa cần FSM/planner thật:

```bash
# ép sang FOLLOW
ros2 topic pub /fsm/state std_msgs/msg/UInt8 "{data: 1}" -r 10

ros2 topic pub /planner/velocity_setpoint offboard_manager/msg/PlannerOutput \
  "{vx: 0.5, vy: 0.0, vz: 0.0, yaw: 0.0}" -r 10
```

Nếu node kẹt mãi ở giai đoạn engage (chưa bao giờ arm): kiểm tra `engage_counter` có tăng đều mỗi chu kỳ không qua log debug — nếu không tăng, khả năng cao timer bị treo hoặc `required_engage_cycles` bị set quá lớn.

Nếu vừa vào OFFBOARD xong lại rớt về failsafe ngay: kiểm tra `watchdog_timeout_sec` có đang đặt quá nhỏ so với chu kỳ timer (50ms) không.