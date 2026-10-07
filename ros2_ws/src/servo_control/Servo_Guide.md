# Servo Control - Guide

`servo_control` nhận `gimbal/target_angle_deg` và điều khiển servo SG90 gắn vào output MAIN 1 của PX4. Pi 5 gửi lệnh qua Micro XRCE-DDS (`/fmu/in/vehicle_command`, lệnh `MAV_CMD_ACTUATOR_TEST` = 310). Node không điều khiển GPIO của Pi 5. Đây là output open-loop, không có feedback góc vật lý. Node hoạt động được cả khi PX4 disarm lẫn khi đang armed và bay.

## Yêu cầu

- Micro XRCE-DDS Agent chạy trên Pi 5 và `uxrce_dds_client` trên PX4 ở trạng thái `connected`. MAVLink phải được dừng trên cổng USB đang dùng cho XRCE.
- PX4 có thể ở trạng thái disarm hoặc armed, lệnh actuator test được chấp nhận ở cả hai trạng thái.
- Servo có nguồn 5 V ngoài, chung GND với FMU. Dây tín hiệu cắm MAIN 1.
- Workspace chứa `px4_msgs` khớp version firmware PX4 được source cùng terminal chạy node.

## Cấu hình PX4

```
PWM_MAIN_FUNC1 = 201
PWM_MAIN_MIN1  = 1000
PWM_MAIN_MAX1  = 2000
PWM_MAIN_DIS1  = 1500
PWM_MAIN_TIM0  = 50
```

Tên tham số có thể khác theo bản PX4, đối chiếu trong QGC → Actuators. Giá trị lệnh nằm trong khoảng -1..1, tương ứng `PWM_MAIN_MIN1`..`PWM_MAIN_MAX1`. Giá trị 0 là vị trí giữa (1500 µs).

## Quy đổi góc

```
servo_angle = home_angle_deg + control_angle_deg   (kẹp trong angle_min..angle_max)
value       = -1 + 2 * (servo_angle - angle_min) / (angle_max - angle_min)
```

Với mặc định (`angle_min=0`, `angle_max=180`, `home=90`):

| control_angle_deg | servo_angle_deg | value |
|---|---|---|
| 0 | 90 | 0.0 |
| -45 | 45 | -0.5 |
| -90 | 0 | -1.0 |
| +90 | 180 | +1.0 |

Góc pitch gimbal trong khoảng -90..0 độ ứng với value 0..-1.

## Tham số

| Tham số | Mặc định | Ý nghĩa |
|---|---|---|
| `servo_function` | 33.0 | Mã output servo 1 trong lệnh 310 (`param5`) |
| `command_timeout_sec` | 2.5 | `param2` của lệnh 310, sau thời gian này PX4 thả output nếu không có lệnh mới |
| `min_send_interval_sec` | 0.1 | Khoảng cách tối thiểu giữa 2 lệnh khi giá trị đổi |
| `min_send_delta` | 0.01 | Chỉ gửi lệnh mới khi value đổi ít nhất bằng ngưỡng này |
| `keepalive_sec` | 1.5 | Gửi lại giá trị hiện tại để servo không bị thả (đặt 0.0 để tắt) |
| `pwm_min_us` / `pwm_max_us` | 1000 / 2000 | Chỉ dùng cho log debug, nên khớp `PWM_MAIN_MIN1/MAX1` |
| `angle_min_deg` / `angle_max_deg` | 0.0 / 180.0 | Dải góc servo |
| `home_angle_deg` | 90.0 | Góc về khi khởi động và khi mất tín hiệu |
| `input_timeout_sec` | 1.0 | Thời gian chờ lệnh gimbal trước khi về home |
| `debug_enabled` | false | In `angle`, `value`, `pwm_us` mỗi lần có lệnh |

## Cơ chế gửi lệnh

- Lệnh chỉ được gửi khi value đổi ít nhất `min_send_delta` và cách lệnh trước ít nhất `min_send_interval_sec`. Gửi liên tục ở tần số cao khiến servo giật hoặc chỉ nhúc nhích nhẹ.
- Nếu góc đứng yên, node gửi lại giá trị hiện tại mỗi `keepalive_sec` để PX4 không thả output sau `command_timeout_sec`. Khi bay nên để `keepalive_sec` bật, vì thiếu cơ chế này servo sẽ bị thả sau `command_timeout_sec` (2.5 s) mỗi lần góc không đổi. Cơ chế gửi lại cùng giá trị này chưa được kiểm chứng trên phần cứng. Nếu servo giật khi đứng yên, chạy với `-p keepalive_sec:=0.0` (servo sẽ bị thả sau 2.5 s nếu góc không đổi).
- Khi tắt node (Ctrl+C), node gửi NaN để PX4 nhả output.

## Timeout input

Node kiểm tra lệnh gimbal mỗi 100 ms. Nếu không nhận lệnh mới trong `input_timeout_sec` (mặc định 1.0 giây), node gửi lệnh về `home_angle_deg` một lần rồi ngừng gửi. Sau `command_timeout_sec` PX4 thả output. Giá trị `Float32` NaN/vô hạn bị bỏ qua.

```bash
ros2 run servo_control servo_node --ros-args \
  -p input_timeout_sec:=1.0 \
  -p home_angle_deg:=90.0
```

`angle_to_pwm_us()` và `servo_angle_to_value()` bảo vệ trường hợp khoảng góc bằng 0 và các giá trị không finite bằng cách dùng giá trị an toàn thay vì chia cho 0.

## Chạy thử

```bash
source /opt/ros/$ROS_DISTRO/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 run servo_control servo_node --ros-args -p debug_enabled:=true
```

Terminal khác:

```bash
ros2 topic pub -r 10 gimbal/target_angle_deg std_msgs/msg/Float32 "{data: -45.0}"
```

Log phải hiển thị `value=-0.500`. Servo đứng yên thì kiểm tra theo thứ tự:

1. `ros2 topic list | grep fmu` có topic `/fmu/in/vehicle_command`.
2. `uxrce_dds_client status` trên PX4 báo `connected`.
3. `ros2 topic echo /fmu/out/vehicle_command_ack` trả `result: 0` cho lệnh 310. Kiểm tra điều này ở cả hai trạng thái disarm và armed.
4. `PWM_MAIN_FUNC1` và dải `PWM_MAIN_MIN1/MAX1` đã đặt đúng trong QGC → Actuators.

## Unit test

Test chỉ kiểm tra logic quy đổi góc, không cần PX4, ROS hay phần cứng:

```bash
python3 -m pip install pytest
python3 -m pytest ros2_ws/src/servo_control/test/test_servo_logic.py
```