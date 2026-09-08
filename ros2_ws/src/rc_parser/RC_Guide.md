# RC Parser — Guide

## 1. Package cần cài đặt ngoài

Không có package ROS2 ngoài nào. Chỉ dùng rclcpp và package tự viết `fsm_state_machine` (để publish đúng kiểu `RcFsmInput`). Lưu ý phần cứng: cần RC receiver hỗ trợ output SBUS nối qua UART vào Pi 5 (không phải package phải cài, nhưng bắt buộc về phần cứng để node chạy được).

## 2. Nguyên lý hoạt động

```
RC receiver (SBUS, UART) --> SbusUart::read_frame()
                                    |
                                    v
                            sbus_decode_frame()
                                    |
                                    v
                              rc_validate()
                                    |
                        ------------+------------
                        |                       |
                        v                       v
                 rc/fsm_input             rc/channels_raw
            (RcFsmInput: land_switch,   (RcChannelsRaw: 16 kênh
             kill_switch)                thô, valid, failsafe)
```

Mỗi chu kỳ 20ms (`update()`):
1. Đọc byte từ UART, gom đủ 1 frame SBUS 25 byte đúng start byte.
2. `sbus_decode_frame()` giải mã 16 kênh 11-bit, lấy cờ failsafe.
3. `rc_validate()` đánh dấu `valid = false` nếu: quá `frame_timeout` chưa có frame hợp lệ, có cờ failsafe, hoặc có kênh nằm ngoài [800, 2200].
4. `rc_get_land_trigger()` / `rc_get_kill_switch()` đọc switch position (LOW/MID/HIGH) theo ngưỡng `low_threshold`/`high_threshold` trên đúng channel index cấu hình.
5. Publish `rc/fsm_input` (chỉ `true` khi `valid == true`) và `rc/channels_raw` (dữ liệu thô để debug). Khi UART không mở được hoặc mất frame, node vẫn publish định kỳ `valid = false`, `failsafe = true` để downstream biết RC đã mất.

`TCGETS2`, `TCSETS2` và `read()` đều được kiểm tra return value. Nếu cấu hình UART thất bại, file descriptor được đóng và node chuyển sang trạng thái mất RC.

## 3. Cách chạy

```bash
ros2 run rc_parser rc_node
```

Chạy kèm tham số tùy chỉnh:

```bash
ros2 run rc_parser rc_node --ros-args \
  -p serial_device:=/dev/ttyUSB0 \
  -p baudrate:=100000 \
  -p land_channel:=4 \
  -p kill_channel:=5 \
  -p low_threshold:=1200 \
  -p high_threshold:=1800 \
  -p frame_timeout:=0.5
```

Danh sách tham số:

| Tên | Mặc định | Ý nghĩa |
|---|---|---|
| `serial_device` | /dev/ttyUSB0 | Cổng UART nối RC receiver |
| `baudrate` | 100000 | Baud rate SBUS (chuẩn) |
| `land_channel` | 4 | Index kênh dùng làm land switch |
| `kill_channel` | 5 | Index kênh dùng làm kill switch |
| `low_threshold` | 1200 | Ngưỡng dưới phân biệt LOW/MID |
| `high_threshold` | 1800 | Ngưỡng trên phân biệt MID/HIGH |
| `frame_timeout` | 0.5 | Thời gian tối đa không có frame hợp lệ trước khi báo `valid = false` |
| `debug_enabled` | false | Bật log mỗi chu kỳ |

## 4. Cách debug

Bật log runtime:

```bash
ros2 run rc_parser rc_node --ros-args -p debug_enabled:=true
```

Mỗi chu kỳ in ra dạng:

```
[INFO] [rc_node]: valid=1 failsafe=0 land=0 kill=0
```

Xem dữ liệu 16 kênh thô đang đọc được:

```bash
ros2 topic echo /rc/channels_raw
```

Xem tín hiệu FSM đang nhận:

```bash
ros2 topic echo /rc/fsm_input
```

Nếu `valid=0` và `failsafe=1`: kiểm tra UART có mở được không (log lỗi lúc khởi động `Khong mo duoc serial device ...`), thường do sai `serial_device`, lỗi cấu hình ioctl hoặc thiếu quyền truy cập cổng serial (`sudo usermod -aG dialout $USER` rồi đăng nhập lại).

Nếu `valid` luôn = 0 dù receiver có tín hiệu: kiểm tra `frame_timeout` có quá nhỏ so với tần số frame SBUS thật (thường ~14ms/frame) hoặc `baudrate` sai (một số receiver dùng SBUS đảo cực tính, cần mạch invert phần cứng riêng trước khi vào UART của Pi 5 vì UART Linux mặc định không tự đảo tín hiệu).

Nếu `land`/`kill` không đổi dù gạt switch: kiểm tra đúng `land_channel`/`kill_channel` index có khớp với cấu hình kênh thật trên transmitter không — so trực tiếp giá trị PWM trong `/rc/channels_raw` khi gạt switch để xác định đúng index.