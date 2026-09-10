# Servo Control - Guide

`servo_control` nhận `gimbal/target_angle_deg` và điều khiển MG90S qua GPIO. Đây là output open-loop, không có feedback góc vật lý.

## Timeout input

Node kiểm tra lệnh gimbal mỗi 100 ms. Nếu không nhận lệnh mới trong `input_timeout_sec` (mặc định 1.0 giây), servo trở về `home_angle_deg` một lần và giữ ở vị trí failsafe đó. Giá trị `Float32` NaN/vô hạn bị bỏ qua.

```bash
ros2 run servo_control servo_node --ros-args \
  -p input_timeout_sec:=1.0 \
  -p home_angle_deg:=90.0
```

`angle_to_pwm_us()` bảo vệ trường hợp khoảng góc bằng 0 và các giá trị không finite bằng cách dùng giá trị an toàn thay vì chia cho 0.

## Unit test

Test không khởi tạo GPIO thật:

```bash
python3 -m pip install pytest
python3 -m pytest ros2_ws/src/servo_control/test/test_servo_logic.py
```