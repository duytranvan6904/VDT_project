# Kill Switch — Guide

## 1. Package cần cài đặt ngoài

Không có package ROS2 ngoài nào mới. Phụ thuộc `rc_parser` (đã có, cần patch thêm 2 dòng export header — xem lúc viết code) và `px4_msgs` (đã cài từ trước cho `offboard_manager`).

## 2. Nguyên lý hoạt động

```
rc_parser --> rc/channels_raw (16 kênh thô, valid, failsafe)
                     |
                     v
            kill_switch_node (timer 50Hz, độc lập FSM/Offboard)
                     |
        rc_get_kill_switch() -> raw_triggered
                     |
        kill_switch_debounce() -> debounce_count >= threshold?
                     |
                    yes
                     |
                     v
        publish_disarm_command()          publish_killed_flag(true)
        (/fmu/in/vehicle_command,          (system/killed, transient_local)
         param2=21196 force disarm)
```

Mỗi chu kỳ 20ms:
1. Nếu đã `triggered` rồi hoặc chưa có dữ liệu channels → bỏ qua, không làm gì thêm (latch, không tự phục hồi).
2. Đọc `rc/channels_raw`, kiểm tra `valid` rồi lấy vị trí kill switch qua `rc_get_kill_switch()` (tái dùng logic từ `rc_parser`, không decode SBUS lần 2).
3. Debounce: đếm số chu kỳ liên tiếp switch ở vị trí HIGH, đủ `debounce_threshold` mới coi là trigger thật (chống nhiễu RC).
4. Nếu trigger: gửi lệnh force disarm tới PX4, publish `system/killed = true` (transient_local, để node khởi động sau vẫn đọc được).

Kill switch chạy hoàn toàn độc lập — không phụ thuộc `fsm_state_machine` hay `offboard_manager` có đang chạy đúng hay không.

## 3. Cách chạy

```bash
ros2 run kill_switch kill_switch_node
```

Chạy kèm tham số tùy chỉnh:

```bash
ros2 run kill_switch kill_switch_node --ros-args \
  -p kill_channel:=5 \
  -p low_threshold:=1200 \
  -p high_threshold:=1800 \
  -p debounce_threshold:=3
```

Danh sách tham số:

| Tên | Mặc định | Ý nghĩa |
|---|---|---|
| `kill_channel` | 5 | Index kênh dùng làm kill switch |
| `low_threshold` | 1200 | Ngưỡng dưới phân biệt LOW/MID |
| `high_threshold` | 1800 | Ngưỡng trên phân biệt MID/HIGH |
| `debounce_threshold` | 3 | Số chu kỳ liên tiếp cần thiết trước khi coi là trigger thật |
| `debug_enabled` | false | Bật log mỗi chu kỳ |

## 4. Cách debug

Bật log runtime:

```bash
ros2 run kill_switch kill_switch_node --ros-args -p debug_enabled:=true
```

Mỗi chu kỳ in ra dạng:

```
[INFO] [kill_switch_node]: raw=0 debounce=0 triggered=0
```

Xem cờ killed hiện tại (subscribe muộn vẫn nhận được nhờ transient_local):

```bash
ros2 topic echo /system/killed
```

Xem lệnh disarm có gửi ra không khi test trigger:

```bash
ros2 topic echo /fmu/in/vehicle_command
```

Giả lập kill switch bật (qua `rc_parser`, không cần phần cứng thật) — gạt kênh 5 lên giá trị HIGH trên transmitter, hoặc test riêng node này bằng cách publish giả `rc/channels_raw`:

```bash
ros2 topic pub /rc/channels_raw rc_parser/msg/RcChannelsRaw \
  "{ch: [1500,1500,1500,1500,1900,1500,0,0,0,0,0,0,0,0,0,0], valid: true, failsafe: false}" -r 50
```

Sau khoảng `debounce_threshold` chu kỳ (~60ms với giá trị mặc định), `triggered` phải chuyển 1 và `system/killed` phải publish `true`.

Nếu trigger rồi mà máy bay vẫn không disarm thật: kiểm tra PX4 có nhận đúng lệnh không qua log console PX4 (`commander status`), và xác nhận `param2` gửi đúng `21196` — thiếu giá trị này PX4 sẽ từ chối disarm khi chưa "landed".

Nếu muốn reset lại sau khi test (không phải bay thật): phải restart lại node — kill switch cố tình không có cơ chế tự gỡ trigger khi gạt switch về vị trí cũ.