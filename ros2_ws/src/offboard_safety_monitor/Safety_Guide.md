# Offboard Safety Monitor - Guide

## Chính sách safety

Node kiểm tra RC override, EKF, pin và tuổi heartbeat Offboard mỗi 200 ms. Dữ liệu local position và battery chỉ được đánh giá khi đã nhận và còn fresh theo `data_freshness_timeout_sec`.

`force_land_requested` mặc định là latch an toàn: khi battery xuống mức warning, `safety/force_land` giữ `true` cho đến khi node khởi động lại. Điều này tránh nhả force-land chỉ vì một mẫu pin tạm thời tốt hơn.

Có thể dùng chế độ không latch trên bench:

```bash
ros2 run offboard_safety_monitor safety_monitor_node --ros-args -p force_land_latched:=false
```

Ở chế độ này, force-land chỉ được reset khi battery message còn fresh, finite, nằm trong `[0, 1]` và trở lại từ ngưỡng warning trở lên. Dữ liệu stale hoặc invalid không được xem là điều kiện reset.

## Tham số liên quan

| Tên | Mặc định | Ý nghĩa |
|---|---:|---|
| `battery_warning_frac` | 0.3 | Ngưỡng force-land |
| `battery_critical_frac` | 0.15 | Ngưỡng RTL và inhibit |
| `data_freshness_timeout_sec` | 1.0 | Tuổi tối đa của local position/battery |
| `force_land_latched` | true | Giữ force-land sau battery warning |

Không dùng chế độ non-latched cho chuyến bay thật nếu chưa có cơ chế mission reset và kiểm thử đầy đủ.

## Test logic

```bash
cd ros2_ws
colcon test --packages-select offboard_safety_monitor --ctest-args -R safety_logic_test
```