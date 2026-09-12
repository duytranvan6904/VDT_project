# FSM 5-State — Guide

## 1. Package cần cài đặt ngoài

- Package ROS 2 chuẩn: `rclcpp`, `std_msgs`.
- Package nội bộ:
  - `input_state_cache`: Để lấy định nghĩa message `InputSnapshot`.
  - `rc_parser`: Để lấy định nghĩa message `RcFsmInput`.

## 2. Nguyên lý hoạt động

```text
[input_state_cache] ──► input_cache/snapshot ──\
[rc_parser]         ──► rc/fsm_input         ──► FsmNode (timer 100ms)
[safety_monitor]    ──► safety/force_land    ──/       │
[kill_switch]       ──► system/killed        ──/       │
                                                       v
                                                 FsmActuators (publish)
                                                       │
                               cmd/yaw_rate, gimbal/state_request,
                               planner/mode, planner/apf_gain,
                               gimbal/align_error_cmd,
                               cmd/vertical_descent_rate,
                               cmd/disarm_request, fsm/state
```

Mỗi chu kỳ 100 ms:
1. **Tiếp nhận Snapshot:** Đọc dữ liệu từ bản tin `InputSnapshot` mới nhất (marker, sai số pixel, độ cao, $\Delta h$, $d_{horiz}$, touchdown, planner_timeout).
2. **Kiểm tra Watchdog Heartbeat:** Nếu quá 200 ms không nhận được snapshot từ `input_state_cache`, FSM tự động đánh dấu `valid = false`, `marker_detected = false` và kích hoạt chế độ an toàn.
3. **Cập nhật bộ đếm:** Cập nhật `marker_stable_count` và `marker_lost_time`.
4. **Kiểm tra chuyển trạng thái:** Đánh giá các điều kiện chuyển trạng thái theo bảng logic bên dưới.
5. **Chuyển trạng thái:** Nếu state thay đổi, reset bộ đếm, ghi nhận `last_transition_time`.
6. **Thực thi hành động:** Gọi hàm `action_*` tương ứng với state hiện tại để gửi lệnh cho Actuator/Planner/Gimbal.
7. **Publish trạng thái:** Xuất trạng thái hiện tại lên topic `fsm/state`.

`safety/force_land = true` có ưu tiên cao nhất và chuyển trực tiếp SEARCH/FOLLOW/APPROACH sang LAND. Khi `planner_timeout = true`, FSM đưa FOLLOW/APPROACH về SEARCH để không tiếp tục bay theo quỹ đạo cũ.

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
| `land_entry_height` | 0.5 | Ngưỡng delta_h cho phép chuyển APPROACH -> LAND |
| `yaw_search_rate` | 0.3 | Tốc độ xoay yaw khi ở SEARCH (rad/s) |
| `land_descent_rate` | 0.4 | Tốc độ hạ cánh khi ở LAND (m/s) |
| `debug_enabled` | false | Bật log chi tiết mỗi chu kỳ |

## 4. Cách debug

Bật log runtime:

```bash
ros2 run fsm_state_machine fsm_node --ros-args -p debug_enabled:=true
```

Mỗi chu kỳ in ra dạng:

```text
[INFO] [fsm_node]: state=1 stable=12 lost_t=0.00 align_err=0.045 alt=3.20 delta_h=1.10 land_sw=0
```

Xem state hiện tại của FSM:

```bash
ros2 topic echo /fsm/state
```

Xem các lệnh điều khiển FSM đang publish ra:

```bash
ros2 topic echo /gimbal/state_request
ros2 topic echo /planner/mode
ros2 topic echo /cmd/yaw_rate
ros2 topic echo /cmd/vertical_descent_rate
ros2 topic echo /cmd/disarm_request
```

### Giả lập dữ liệu để kiểm thử chuyển trạng thái độc lập (Bench test)

1. **Ép trạng thái SEARCH $\rightarrow$ FOLLOW:**
   ```bash
   ros2 topic pub /input_cache/snapshot input_state_cache/msg/InputSnapshot \
     "{valid: true, marker_detected: true, align_error: 0.1, altitude: 2.0, delta_h: 1.0, d_horiz: 0.5, touchdown: false, planner_timeout: false}" -r 10
   ```

2. **Ép trạng thái FOLLOW $\rightarrow$ APPROACH (bật công tắc hạ cánh RC):**
   ```bash
   ros2 topic pub /rc/fsm_input rc_parser/msg/RcFsmInput \
     "{land_switch: true, kill_switch: false}" -r 10
   ```

3. **Ép trạng thái APPROACH $\rightarrow$ LAND (căn chỉnh thẳng hàng và đạt độ cao hạ):**
   ```bash
   ros2 topic pub /input_cache/snapshot input_state_cache/msg/InputSnapshot \
     "{valid: true, marker_detected: true, align_error: 0.15, altitude: 0.4, delta_h: 0.3, d_horiz: 0.1, touchdown: false, planner_timeout: false}" -r 10
   ```

4. **Ép trạng thái LAND $\rightarrow$ COMPLETE (chạm đất):**
   ```bash
   ros2 topic pub /input_cache/snapshot input_state_cache/msg/InputSnapshot \
     "{valid: true, marker_detected: true, align_error: 0.0, altitude: 0.05, delta_h: 0.0, d_horiz: 0.0, touchdown: true, planner_timeout: false}" -r 10
   ```

5. **Kiểm tra trạng thái Snapshot đầu vào:**
   ```bash
   ros2 topic echo /input_cache/snapshot
   ```

### Chạy unit test logic tự động:

```bash
cd ros2_ws
colcon test --packages-select fsm_state_machine --ctest-args -R fsm_logic_test
```