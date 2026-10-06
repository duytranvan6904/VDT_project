# FSM 5-State — Guide

## 1. Package cần cài đặt ngoài

- Package ROS 2 chuẩn: `rclcpp`, `std_msgs`.
- Phụ thuộc `vdt_msgs` để lấy định nghĩa `RcFsmInput` và `InputSnapshot` (cả hai đều là `vdt_msgs::msg::...`).

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
3. **Kiểm tra hợp lệ và tính `geometry_valid`:** `sensor_validate` đặt `valid = false` nếu `align_error` hoặc `altitude` là NaN. Sau đó `geometry_valid = valid && isfinite(delta_h) && isfinite(d_horiz)`. `sensor_validate` có hỗ trợ cờ `ekf_timeout`/`vision_timeout` (khi bật thì `valid = false` và `marker_detected = false`), nhưng FSM node hiện truyền cờ mặc định (đều `false`) nên chưa phân biệt mode EKF (chờ code planner).
4. **Cập nhật bộ đếm:**
   - `marker_stable_count` chỉ tăng khi thấy marker, `geometry_valid` và `abs(yaw_rate) < yaw_settle_rate`; sai một chu kỳ thì về 0.
   - `marker_lost_time` cộng thêm `dt` khi không thấy marker, về 0 ngay khi thấy marker lại, và **không** bị reset khi đổi state.
   - `land_ok_count` chỉ đếm ở APPROACH, tăng khi `geometry_valid && align_error < align_threshold && delta_h < land_entry_height` (các so sánh đều chặt, không cần `marker_detected`); sai một chu kỳ thì về 0.
5. **Kiểm tra chuyển trạng thái:** Đánh giá theo thứ tự ưu tiên `safety/force_land`, rồi `planner_timeout`, rồi bảng logic bên dưới. Trong đó `planner_timeout = snapshot_timeout || InputSnapshot.planner_timeout`. Các điều kiện RC được đánh giá trên `land_switch` hiệu lực: `land_switch_effective = rc.land_switch || safety/force_land`.
6. **Chuyển trạng thái:** Nếu state thay đổi, reset `marker_stable_count` và `land_ok_count`, ghi nhận `last_transition_time`. Cờ `land_inhibit` được cập nhật mỗi chu kỳ (dùng `land_switch` hiệu lực): đặt khi rời APPROACH (trừ sang LAND) mà công tắc vẫn bật, xóa khi công tắc tắt.
7. **Thực thi hành động:** Gọi hàm `action_*` tương ứng với state hiện tại để gửi lệnh cho Actuator/Planner/Gimbal.
8. **Publish trạng thái:** Xuất trạng thái hiện tại lên topic `fsm/state`.

`safety/force_land = true` có ưu tiên cao nhất và chuyển trực tiếp SEARCH/FOLLOW/APPROACH sang LAND (không áp dụng khi đã ở LAND hoặc COMPLETE). Khi `planner_timeout = true` (kể cả do mất snapshot quá 200 ms), FSM đưa FOLLOW/APPROACH về SEARCH để không tiếp tục bay theo quỹ đạo cũ (bỏ qua nếu `ignore_planner_timeout = true`).

Các dòng của cùng một state trong bảng được xét từ trên xuống, dòng đầu thỏa điều kiện thì thắng (ví dụ FOLLOW mất marker quá timeout về SEARCH trước khi xét APPROACH).

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

`planner/mode` và `gimbal/state_request` mang giá trị `uint8` của enum `State` (SEARCH=0, FOLLOW=1, APPROACH=2, LAND=3, COMPLETE=4).

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
| `approach_descent_rate` | 0.3 | Tốc độ hạ khi ở APPROACH (m/s), do `FsmActuators` khai báo |
| `land_descent_rate` | 0.4 | Tốc độ hạ cánh khi ở LAND (m/s), do `FsmActuators` khai báo |
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
   ros2 topic pub /input_cache/snapshot vdt_msgs/msg/InputSnapshot \
     "{valid: true, marker_detected: true, align_error: 0.1, altitude: 2.0, delta_h: 1.0, d_horiz: 0.5, yaw_rate: 0.0, touchdown: false, planner_timeout: false}" -r 10
```

2. **Ép trạng thái FOLLOW $\rightarrow$ APPROACH (bật công tắc hạ cánh RC):**
```bash
   ros2 topic pub /rc/fsm_input vdt_msgs/msg/RcFsmInput \
     "{land_switch: true, kill_switch: false}" -r 10
```
   Ở APPROACH với snapshot của ca 1 (`delta_h: 1.0`), `/cmd/vertical_descent_rate` phải bằng `approach_descent_rate`, `/planner/apf_gain` bằng 0.5 và `/gimbal/align_error_cmd` bằng `align_error` của snapshot.

3. **Ép trạng thái APPROACH $\rightarrow$ LAND (căn chỉnh thẳng hàng và đạt độ cao hạ):**
```bash
   ros2 topic pub /input_cache/snapshot vdt_msgs/msg/InputSnapshot \
     "{valid: true, marker_detected: true, align_error: 0.15, altitude: 0.4, delta_h: 0.3, d_horiz: 0.1, yaw_rate: 0.0, touchdown: false, planner_timeout: false}" -r 10
```
   Cần giữ liên tục tối thiểu `land_entry_cycles` chu kỳ (0.5 s ở 10 Hz) mới sang LAND.

4. **Ép trạng thái LAND $\rightarrow$ COMPLETE (chạm đất):**
```bash
   ros2 topic pub /input_cache/snapshot vdt_msgs/msg/InputSnapshot \
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

## 5. Test

Test chia 3 tầng, nằm trong `test/` của package (C++ dùng gtest, riêng tầng rosbag viết bằng Python). Tầng 1 không cần ROS 2 chạy; tầng 2 và 3 cần source ROS 2 và workspace đã build.

### Tầng 1: logic thuần (`fsm_logic_test`)

Không tạo node, gọi trực tiếp các hàm trong `fsm_logic.hpp`. Một `Harness` trong file test ghép các hàm theo đúng thứ tự của `update()` (validate, đếm, force_land, planner_timeout, `evaluate_transition`, reset, cập nhật `land_inhibit`) để chạy kịch bản nhiều chu kỳ.

| Nhóm | Kiểm tra |
|---|---|
| `FsmTypes` | Giá trị mặc định của `FsmParams`, `FsmContext`, `SensorInput` |
| `SensorValidate` | NaN ở `align_error`/`altitude` làm `valid = false`; NaN hoặc inf ở `delta_h`/`d_horiz` chỉ làm `geometry_valid = false`; `ekf_timeout`/`vision_timeout` xóa `marker_detected`; cờ `planner_timeout` không ảnh hưởng sensor |
| `SearchEntry`, `LandEntry` | Cần marker, NaN hình học, ngưỡng `yaw_rate` hai chiều và NaN, ngưỡng chặt của `align_threshold`/`land_entry_height`, không cần marker ở `land_entry_ok` |
| `Counters` | `marker_stable_count`, `marker_lost_time` (cộng dồn và reset khi thấy marker), `land_ok_count` chỉ đếm ở APPROACH, `land_inhibit` (đặt khi rời APPROACH trừ sang LAND, xóa khi công tắc tắt), `counters_reset` không xóa `marker_lost_time` |
| `Check*`, `EvaluateTransition` | Từng bảng chuyển state, thứ tự ưu tiên trong cùng state, `land_requires_rearm`, COMPLETE là trạng thái cuối |
| `EffectiveRc`, `ForceLand`, `PlannerTimeout` | `land_switch` hiệu lực, `force_land_transition` theo state, `planner_timeout_transition` và `ignore_planner_timeout` |
| `Scenario` | SEARCH → FOLLOW cần đủ chu kỳ và một chu kỳ xấu làm đếm lại, `yaw_rate`/NaN chặn FOLLOW, mất marker, land_inhibit không tự sống lại, `land_ok` phải liên tiếp, LAND → COMPLETE, force_land từ mọi state hoạt động (ưu tiên hơn planner_timeout), chuỗi bay đủ 5 state |

```bash
cd ros2_ws
colcon test --packages-select fsm_state_machine --ctest-args -R fsm_logic_test
```

### Tầng 2: ROS 2 với publisher giả

Dùng thời gian thật và một node probe cùng process để publish đầu vào và nghe đầu ra.

| Test | Kiểm tra |
|---|---|
| `fsm_actions_test` | Từng `action_*` publish đúng `planner/mode`, `gimbal/state_request`, `planner/apf_gain` (1.0/0.5/0.0), `gimbal/align_error_cmd` (chỉ APPROACH), `cmd/vertical_descent_rate` (0.3 khi đủ điều kiện hạ ở APPROACH, 0 khi thiếu marker, thiếu `geometry_valid` hoặc dưới `land_entry_height`, 0.4 ở LAND), `cmd/disarm_request` ở COMPLETE và COMPLETE không publish lệnh nào khác, thứ tự mode khi gọi nối tiếp |
| `fsm_node_test` | Chạy `FsmNode` thật: SEARCH → FOLLOW → APPROACH → LAND → COMPLETE, `yaw_rate` cao hoặc NaN, `delta_h`/`align_error` NaN không vào FOLLOW, mất marker, mất snapshot quá 200 ms, cờ `planner_timeout` trong snapshot, `force_land` từ SEARCH/FOLLOW/APPROACH, land_inhibit ở mức node, COMPLETE chỉ còn publish `cmd/disarm_request`, `system/killed` làm node ngừng publish mọi topic (và bỏ qua `force_land`) |

```bash
colcon test --packages-select fsm_state_machine --ctest-args -R "fsm_actions_test|fsm_node_test"
```

Chạy riêng một nhóm bằng binary:

```bash
./build/fsm_state_machine/fsm_node_test --gtest_filter=FsmNodeTest.KilledStopsAllPublishing
```

### Tầng 3: rosbag

`test/test_fsm_rosbag.py` tự sinh bag tổng hợp (`input_cache/snapshot` và `rc/fsm_input` ở 20 Hz), launch `fsm_node`, sau 3 s thì `ros2 bag play`, rồi kiểm tra chuỗi lệnh FSM xuất ra.

| Đoạn | Snapshot | `land_switch` | Thời lượng |
|---|---|---|---|
| 1 | Marker ổn định, `delta_h: 1.0` | tắt | 2.0 s |
| 2 | Như đoạn 1 | bật | 1.0 s |
| 3 | `align_error: 0.15`, `altitude: 0.4`, `delta_h: 0.3`, `d_horiz: 0.1` | bật | 2.0 s |
| 4 | `touchdown: true` | bật | 1.0 s |

Kỳ vọng: `fsm/state` đi qua `[0, 1, 2, 3, 4]` và kết thúc ở 4; `planner/mode` đi qua `[0, 1, 2, 3]`; `planner/apf_gain` có 1.0, 0.5 và 0.0; `cmd/vertical_descent_rate` có 0.3 và 0.4; `cmd/disarm_request` luôn là `true`; không có state nào đi lùi; node thoát sạch khi shutdown.

```bash
colcon test --packages-select fsm_state_machine --ctest-args -R test_fsm_rosbag
```

Để dùng bag thật, ghi bag rồi đổi đường dẫn trong `generate_test_description()` và chỉnh lại giá trị kỳ vọng theo kịch bản:

```bash
ros2 bag record -o sample_flight /input_cache/snapshot /rc/fsm_input /safety/force_land
```

### Chạy toàn bộ

```bash
colcon build --packages-select fsm_state_machine
source install/setup.bash
colcon test --packages-select fsm_state_machine --event-handlers console_direct+
colcon test-result --verbose
```

### Lưu ý khi chạy test

- Test tầng 2 và 3 dùng thời gian thật nên có thể chập chờn trên máy chậm; chạy lại một lần trước khi kết luận lỗi.
- `fsm_actions_test` và `fsm_node_test` dùng chung tên topic, nên được khóa bằng `RESOURCE_LOCK ros_graph` để không chạy song song. Nếu chạy cạnh hệ thống đang chạy, đặt domain riêng: `ROS_DOMAIN_ID=77 colcon test ...`.
- Test `fsm_node_test` dùng tham số mặc định, trong đó `ignore_planner_timeout = false`; snapshot giả được publish 20 Hz để không chạm watchdog 200 ms.
- `fsm_node.hpp` phải dùng `vdt_msgs::msg::InputSnapshot` và `vdt_msgs::msg::RcFsmInput`; test node và test rosbag dùng đúng hai kiểu này.
- Test rosbag cần `ros2 bag play` trong `PATH` (package `ros2bag`) và plugin lưu trữ `sqlite3`. Constructor `rosbag2_py.TopicMetadata` khác nhau giữa các distro, file test đã xử lý cả hai dạng.

Nếu `fsm_node_test` báo không nhận được message: kiểm tra `ros2 topic list` trong cùng domain có node FSM khác đang chạy không, vì hai node cùng publish `fsm/state` sẽ làm test nhiễu.

Nếu `test_fsm_rosbag` báo thiếu state: tăng `BAG_DELAY` trong file test (node cần thời gian khởi động trước khi bag phát).