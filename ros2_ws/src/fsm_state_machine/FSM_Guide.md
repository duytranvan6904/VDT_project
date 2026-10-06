# FSM 5-State — Guide

## 1. Package cần cài đặt ngoài

- Package ROS 2 chuẩn: `rclcpp`, `std_msgs`.
- Phụ thuộc `vdt_msgs` để lấy định nghĩa `RcFsmInput` và `InputSnapshot`.

## 2. Nguyên lý hoạt động

```text
[input_state_cache] ──► input_cache/snapshot ──\
[rc_parser]         ──► rc/fsm_input         ──► FsmNode (timer 100ms)
[safety_monitor]    ──► safety/force_land    ──/       │
[kill_switch]       ──► system/killed        ──/       │
                                                       v
                                                 FsmActuators (publish)
                                                       │
                               gimbal/state_request,
                               planner/mode, planner/apf_gain,
                               gimbal/align_error_cmd,
                               cmd/vertical_descent_rate,
                               cmd/disarm_request, fsm/state
```

`system/killed` dùng QoS `transient_local` (depth 1). Khi `killed = true`, `update()` thoát ngay đầu chu kỳ: FSM không cập nhật bộ đếm, không chuyển state, không publish bất kỳ topic nào (kể cả `fsm/state`). `rc/fsm_input.kill_switch` chỉ được lưu lại, FSM không dùng trực tiếp để dừng.

Mỗi chu kỳ 100 ms:
1. **Tiếp nhận Snapshot:** Đọc dữ liệu từ bản tin `InputSnapshot` mới nhất (marker, sai số pixel, độ cao, $\Delta h$, $d_{horiz}$, `yaw_rate`, touchdown, planner_timeout). `dt` tính theo thời gian giữa hai chu kỳ liên tiếp (chu kỳ đầu `dt = 0`).
2. **Kiểm tra Watchdog Heartbeat:** Nếu quá 200 ms không nhận được snapshot từ `input_state_cache` (hoặc chưa từng nhận), FSM tự động đánh dấu `valid = false`, `marker_detected = false` và coi như `planner_timeout = true` (xem bước 5).
3. **Kiểm tra hợp lệ và tính `geometry_valid`:** `sensor_validate` đặt `valid = false` nếu `align_error` hoặc `altitude` là NaN. Sau đó `geometry_valid = valid && isfinite(delta_h) && isfinite(d_horiz)`. `sensor_validate` có hỗ trợ cờ `ekf_timeout`/`vision_timeout`, nhưng FSM node hiện truyền cờ mặc định (đều `false`) nên chưa phân biệt mode EKF (chờ code planner).
4. **Cập nhật bộ đếm:**
   - `marker_stable_count` chỉ tăng khi thấy marker, `geometry_valid` và `abs(yaw_rate) < yaw_settle_rate`; sai một chu kỳ thì về 0.
   - `marker_lost_time` tăng khi không thấy marker và **không** bị reset khi đổi state.
   - `land_ok_count` chỉ đếm ở APPROACH, tăng khi `geometry_valid && align_error < align_threshold && delta_h < land_entry_height`; sai một chu kỳ thì về 0.
5. **Kiểm tra chuyển trạng thái:** Đánh giá theo thứ tự ưu tiên `safety/force_land`, rồi `planner_timeout`, rồi bảng logic bên dưới. Trong đó `planner_timeout = snapshot_timeout || InputSnapshot.planner_timeout`. Các điều kiện RC được đánh giá trên `land_switch` hiệu lực: `land_switch_effective = rc.land_switch || safety/force_land`.
6. **Chuyển trạng thái:** Nếu state thay đổi, reset `marker_stable_count` và `land_ok_count`, ghi nhận `last_transition_time`. Cờ `land_inhibit` được cập nhật mỗi chu kỳ (dùng `land_switch` hiệu lực): đặt khi rời APPROACH (trừ sang LAND) mà công tắc vẫn bật, xóa khi công tắc tắt.
7. **Thực thi hành động:** Gọi hàm `action_*` tương ứng với state hiện tại để gửi lệnh cho Actuator/Planner/Gimbal.
8. **Publish trạng thái:** Xuất trạng thái hiện tại lên topic `fsm/state`.

`safety/force_land = true` có ưu tiên cao nhất và chuyển trực tiếp SEARCH/FOLLOW/APPROACH sang LAND (không áp dụng khi đã ở LAND hoặc COMPLETE). Khi `planner_timeout = true` (kể cả do mất snapshot quá 200 ms), FSM đưa FOLLOW/APPROACH về SEARCH để không tiếp tục bay theo quỹ đạo cũ (bỏ qua nếu `ignore_planner_timeout = true`).

| State hiện tại | Điều kiện | State kế tiếp |
|---|---|---|
| SEARCH | `marker_stable_count >= enter_follow_cycles` | FOLLOW |
| FOLLOW | `marker_lost_time > follow_lost_timeout` | SEARCH |
| FOLLOW | `land_switch && marker_detected && !land_inhibit` | APPROACH |
| APPROACH | `!land_switch` | FOLLOW |
| APPROACH | `marker_lost_time > approach_lost_timeout` | FOLLOW |
| APPROACH | `land_ok_count >= land_entry_cycles` | LAND |
| SEARCH/FOLLOW/APPROACH | `safety/force_land == true` | LAND |
| FOLLOW/APPROACH | `planner_timeout == true` (bỏ qua nếu `ignore_planner_timeout`) | SEARCH |
| LAND | `touchdown == true` | COMPLETE |

`land_inhibit` chỉ có tác dụng khi `land_requires_rearm = true`. COMPLETE là trạng thái cuối, không có chuyển tiếp đi ra; ở COMPLETE, FSM publish `cmd/disarm_request = true` mỗi chu kỳ.

### Phân chia trách nhiệm điều khiển

| Việc | Chủ sở hữu |
|---|---|
| Quét yaw ở SEARCH, phanh yaw khi thấy marker | IBVS (khi `planner/mode = SEARCH`); FSM không publish `cmd/yaw_rate` |
| Kiểm tra yaw đã ổn định trước khi vào FOLLOW | FSM, qua `yaw_settle_rate` |
| Hạ độ cao (`cmd/vertical_descent_rate`) | FSM |
| Di chuyển XY | Planner |

`cmd/vertical_descent_rate` theo state:

| State | Giá trị |
|---|---|
| SEARCH, FOLLOW | 0 |
| APPROACH | `approach_descent_rate` nếu `marker_detected && geometry_valid && delta_h >= land_entry_height`, ngược lại 0 |
| LAND | `land_descent_rate` |
| COMPLETE | Không publish |

Các lệnh khác FSM publish theo state:

| State | `planner/mode` | `gimbal/state_request` | `planner/apf_gain` | `gimbal/align_error_cmd` |
|---|---|---|---|---|
| SEARCH | SEARCH | SEARCH | Không publish | Không publish |
| FOLLOW | FOLLOW | FOLLOW | 1.0 | Không publish |
| APPROACH | APPROACH | APPROACH | 0.5 | `align_error` |
| LAND | LAND | LAND | 0.0 | Không publish |
| COMPLETE | Không publish | Không publish | Không publish | Không publish |

`planner/mode` và `gimbal/state_request` mang giá trị `uint8` của enum `State`.

### Hợp đồng với các node khác

- **IBVS:** tự quét yaw khi `planner/mode = SEARCH`, và phải đặt yaw rate bằng 0 ngay khi thấy marker, giữ cho đến khi mất marker hẳn. FSM chỉ kiểm tra lại bằng `yaw_settle_rate`, không thay thế việc phanh.
- **Planner:** phải publish `planner/velocity_setpoint` liên tục, kể cả khi chưa có mục tiêu, nếu không `planner_timeout` luôn `true` và FSM dao động SEARCH ↔ FOLLOW.
- **input_state_cache:** phải publish `input_cache/snapshot` đều đặn (khoảng cách giữa hai bản tin không quá 200 ms) và điền `yaw_rate` (rad/s, quanh trục yaw) vào `InputSnapshot`. Nếu `yaw_rate` là NaN thì FSM không bao giờ vào FOLLOW; nếu `align_error` hoặc `altitude` là NaN thì `valid = false` và FSM cũng không vào được FOLLOW.
- **safety_monitor / kill_switch:** `safety/force_land` và `system/killed` là hai đường riêng. `force_land` đưa FSM vào LAND; `killed` làm FSM ngừng hoạt động hoàn toàn (không publish gì nữa).

## 3. Cách chạy

```bash
ros2 run fsm_state_machine fsm_node
```

Chạy kèm tham số tùy chỉnh:

```bash
ros2 run fsm_state_machine fsm_node --ros-args \
  -p land_entry_height:=0.6 \
  -p approach_descent_rate:=0.3 \
  -p land_descent_rate:=0.4
```

Danh sách tham số:

| Tên | Mặc định | Ý nghĩa |
|---|---|---|
| `land_entry_height` | 0.5 | Ngưỡng delta_h cho phép chuyển APPROACH -> LAND (cũng là ngưỡng dừng hạ ở APPROACH) |
| `align_threshold` | 0.3 | Ngưỡng align_error cho phép chuyển APPROACH -> LAND |
| `land_entry_cycles` | 5 | Số chu kỳ liên tiếp thỏa điều kiện LAND |
| `enter_follow_cycles` | 10 | Số chu kỳ liên tiếp để vào FOLLOW |
| `yaw_settle_rate` | 0.1 | Ngưỡng abs(yaw_rate) coi là đã ổn định (rad/s) |
| `follow_lost_timeout` | 2.5 | Mất marker bao lâu thì FOLLOW -> SEARCH (s) |
| `approach_lost_timeout` | 1.5 | Mất marker bao lâu thì APPROACH -> FOLLOW (s) |
| `approach_descent_rate` | 0.3 | Tốc độ hạ khi ở APPROACH (m/s) |
| `land_descent_rate` | 0.4 | Tốc độ hạ cánh khi ở LAND (m/s) |
| `land_requires_rearm` | true | Hủy APPROACH khi công tắc vẫn bật thì phải gạt tắt rồi bật lại mới vào lại APPROACH |
| `ignore_planner_timeout` | false | Bỏ qua `planner_timeout`, chỉ dùng khi bench chưa có planner |
| `debug_enabled` | false | Bật log chi tiết mỗi chu kỳ |

## 4. Cách debug

Bật log runtime:

```bash
ros2 run fsm_state_machine fsm_node --ros-args -p debug_enabled:=true
```

Mỗi chu kỳ in ra dạng:

```text
[INFO] [fsm_node]: state=1 stable=12 lost_t=0.00 land_ok=0 land_inh=0 align_err=0.045 alt=3.20 delta_h=1.10 yaw_rate=0.020 geom=1 land_sw=0
```

Ý nghĩa các trường mới: `land_ok` là `land_ok_count`, `land_inh` là cờ `land_inhibit`, `geom` là `geometry_valid`, `land_sw` là `land_switch` hiệu lực (đã gộp `safety/force_land`). Các giá trị `align_err`, `alt`, `delta_h`, `yaw_rate`, `geom` là sau bước kiểm tra hợp lệ và watchdog.

Khi bật `ignore_planner_timeout`, node in cảnh báo `ignore_planner_timeout is enabled` (throttle 5 s).

Xem state hiện tại của FSM:

```bash
ros2 topic echo /fsm/state
```

Xem các lệnh điều khiển FSM đang publish ra:

```bash
ros2 topic echo /gimbal/state_request
ros2 topic echo /planner/mode
ros2 topic echo /planner/apf_gain
ros2 topic echo /gimbal/align_error_cmd
ros2 topic echo /cmd/vertical_descent_rate
ros2 topic echo /cmd/disarm_request
```

### Giả lập dữ liệu để kiểm thử chuyển trạng thái độc lập (Bench test)

Khi chưa có planner, chạy FSM với `ignore_planner_timeout:=true`, nếu không `planner_timeout` luôn `true` và FSM dao động SEARCH ↔ FOLLOW:

```bash
ros2 run fsm_state_machine fsm_node --ros-args -p ignore_planner_timeout:=true -p debug_enabled:=true
```

Lưu ý: snapshot phải được publish liên tục (ví dụ `-r 10`). Nếu ngừng publish quá 200 ms, FSM coi là mất snapshot, `marker_detected = false` và `valid = false`.

1. **Ép trạng thái SEARCH $\rightarrow$ FOLLOW:**
```bash
   ros2 topic pub /input_cache/snapshot input_state_cache/msg/InputSnapshot \
     "{valid: true, marker_detected: true, align_error: 0.1, altitude: 2.0, delta_h: 1.0, d_horiz: 0.5, yaw_rate: 0.0, touchdown: false, planner_timeout: false}" -r 10
```

2. **Ép trạng thái FOLLOW $\rightarrow$ APPROACH (bật công tắc hạ cánh RC):**
```bash
   ros2 topic pub /rc/fsm_input rc_parser/msg/RcFsmInput \
     "{land_switch: true, kill_switch: false}" -r 10
```
   Ở APPROACH với snapshot của ca 1 (`delta_h: 1.0`), `/cmd/vertical_descent_rate` phải bằng `approach_descent_rate`, `/planner/apf_gain` bằng 0.5 và `/gimbal/align_error_cmd` bằng `align_error` của snapshot.

3. **Ép trạng thái APPROACH $\rightarrow$ LAND (căn chỉnh thẳng hàng và đạt độ cao hạ):**
```bash
   ros2 topic pub /input_cache/snapshot input_state_cache/msg/InputSnapshot \
     "{valid: true, marker_detected: true, align_error: 0.15, altitude: 0.4, delta_h: 0.3, d_horiz: 0.1, yaw_rate: 0.0, touchdown: false, planner_timeout: false}" -r 10
```
   Cần giữ liên tục tối thiểu `land_entry_cycles` chu kỳ (0.5 s ở 10 Hz) mới sang LAND.

4. **Ép trạng thái LAND $\rightarrow$ COMPLETE (chạm đất):**
```bash
   ros2 topic pub /input_cache/snapshot input_state_cache/msg/InputSnapshot \
     "{valid: true, marker_detected: true, align_error: 0.0, altitude: 0.05, delta_h: 0.0, d_horiz: 0.0, yaw_rate: 0.0, touchdown: true, planner_timeout: false}" -r 10
```
   Ở COMPLETE, `/cmd/disarm_request` phải có `data: true`.

5. **Kiểm tra trạng thái Snapshot đầu vào:**
```bash
   ros2 topic echo /input_cache/snapshot
```

6. **Yaw chưa ổn định thì không vào FOLLOW:** publish snapshot như ca 1 nhưng `yaw_rate: 0.3`. Kỳ vọng `/fsm/state` vẫn là 0 (SEARCH). Đổi `yaw_rate: 0.02` thì sang 1 (FOLLOW) sau `enter_follow_cycles` chu kỳ.

7. **Dữ liệu hình học NaN thì không vào FOLLOW:** publish snapshot như ca 1 nhưng `delta_h: .nan`. Kỳ vọng `/fsm/state` vẫn là 0.

8. **Công tắc hạ cánh không tự sống lại:** đang ở APPROACH với `land_switch: true`, publish snapshot `marker_detected: false` quá `approach_lost_timeout` giây để về FOLLOW, rồi cho `marker_detected: true` trở lại. Kỳ vọng vẫn ở FOLLOW. Publish `land_switch: false` rồi `true` thì mới sang APPROACH.

9. **Force land:** đang ở SEARCH/FOLLOW/APPROACH, publish `safety/force_land`:
```bash
   ros2 topic pub /safety/force_land std_msgs/msg/Bool "{data: true}" -r 10
```
   Kỳ vọng `/fsm/state` chuyển ngay sang 3 (LAND), `/cmd/vertical_descent_rate` bằng `land_descent_rate`.

10. **Mất snapshot:** đang ở FOLLOW hoặc APPROACH (không bật `ignore_planner_timeout`), ngừng publish snapshot quá 200 ms. Kỳ vọng `/fsm/state` về 0 (SEARCH).

11. **Kill:** publish `system/killed` bằng `true`:
```bash
    ros2 topic pub --once /system/killed std_msgs/msg/Bool "{data: true}"
```
    Kỳ vọng FSM ngừng publish mọi topic, `/fsm/state` không còn cập nhật.

### Chạy unit test logic tự động:

```bash
cd ros2_ws
colcon test --packages-select fsm_state_machine --ctest-args -R fsm_logic_test
```

Các ca cần có trong `fsm_logic_test`:

1. `delta_h` hoặc `d_horiz` là NaN (sau `sensor_validate`): `search_entry_ok` trả `false`.
2. `yaw_rate = 0.3`: `search_entry_ok` trả `false`; `yaw_rate = 0.02`: trả `true`.
3. APPROACH → FOLLOW do mất marker với công tắc vẫn bật: `land_inhibit` thành `true`, FOLLOW không sang APPROACH dù marker quay lại; công tắc tắt thì cờ xóa. Tương tự với APPROACH → SEARCH do `planner_timeout`.
4. `land_ok` đúng một chu kỳ rồi sai: không sang LAND; đúng đủ `land_entry_cycles` chu kỳ liên tiếp: sang LAND.
5. `counters_reset` không xóa `marker_lost_time`.
6. `ignore_planner_timeout = true`: `planner_timeout_transition` luôn trả `nullopt`.
7. `align_error` hoặc `altitude` là NaN (sau `sensor_validate`): `valid = false` và `geometry_valid = false`.
8. `force_land_transition`: trả `LAND` ở SEARCH/FOLLOW/APPROACH, trả `nullopt` ở LAND/COMPLETE hoặc khi `force_land_requested = false`.
9. `effective_rc_input`: `land_switch` hiệu lực bằng `rc.land_switch || force_land_requested`.