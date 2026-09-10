# VDT Bringup

Package này cung cấp launch orchestration cho toàn bộ ROS 2 stack của VDT.

## Startup order

```text
xrce_bridge_manager
        |
        v
input_state_cache
        |
        +--> rc_parser -> kill_switch
        |
        v
offboard_safety_monitor
        |
        v
fsm_state_machine + gimbal_control
        |
        v
offboard_manager
        |
        v
servo_control (optional)
```

Launch delay chỉ bảo đảm thứ tự tạo node và publisher/subscriber. Đây không phải readiness health check. Các node vẫn tự chặn hành vi nguy hiểm:

- `offboard_manager` chờ `VehicleStatus` và `VehicleLocalPosition` fresh, EKF hợp lệ và PX4 xác nhận `OFFBOARD` trước khi arm.
- `offboard_safety_monitor` kiểm tra freshness pin/EKF và phát inhibit/force-land.
- `input_state_cache` phát timeout flags, trong đó có `planner_timeout`.
- `system/killed` được giữ bằng QoS transient-local.

## Chạy

```bash
cd ros2_ws
source /opt/ros/<ros_distro>/setup.bash
source install/setup.bash
ros2 launch vdt_bringup vdt_system.launch.py
```

Bench không có RC phần cứng:

```bash
ros2 launch vdt_bringup vdt_system.launch.py start_hardware:=false
```

Bật servo chỉ trên Raspberry Pi có GPIO:

```bash
ros2 launch vdt_bringup vdt_system.launch.py start_servo:=true
```

Debug:

```bash
ros2 launch vdt_bringup vdt_system.launch.py debug:=true
```

Không dùng `start_hardware:=false` cho test bay; tùy chọn này chỉ bỏ qua node đọc RC/kill switch trong bench/SITL có safety control khác.
