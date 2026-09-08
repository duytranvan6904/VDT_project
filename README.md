# VDT Project

Workspace prototype cho hệ thống UAV PX4 + ROS 2 thực hiện phát hiện, bám và hạ cánh lên H-Pad bằng vision. Hệ thống gồm bộ máy trạng thái bay, điều khiển Offboard, điều khiển gimbal/servo, nhận RC SBUS, các lớp giám sát an toàn và công cụ phân tích log sau chuyến bay.

> **Trạng thái:** Đây là skeleton tích hợp/prototype, chưa phải hệ thống sẵn sàng bay. Cần sửa các lỗi build và hoàn thiện kiểm thử an toàn được nêu trong [Các vấn đề cần xử lý](#các-vấn-đề-cần-xử-lý) trước khi kết nối phương tiện thật.

## Mục lục

- [Kiến trúc tổng quan](#kiến-trúc-tổng-quan)
- [Cấu trúc repository](#cấu-trúc-repository)
- [ROS 2 packages](#ros-2-packages)
- [Messages](#messages)
- [Topics chính](#topics-chính)
- [Luồng hoạt động](#luồng-hoạt-động)
- [Cài đặt và build](#cài-đặt-và-build)
- [Chạy hệ thống](#chạy-hệ-thống)
- [Landing diagnostics](#landing-diagnostics)
- [Tích hợp PX4 và QGroundControl](#tích-hợp-px4-và-qgroundcontrol)
- [Kiểm thử và validation](#kiểm-thử-và-validation)
- [Các vấn đề cần xử lý](#các-vấn-đề-cần-xử-lý)
- [Tài liệu tham khảo trong repository](#tài-liệu-tham-khảo-trong-repository)

## Kiến trúc tổng quan

```text
PX4 / Vision / Altimeter / Planner / RC
                  |
                  v
          ROS 2 qua Micro XRCE-DDS
                  |
       +----------+-----------+
       |                      |
       v                      v
input_state_cache       FSM + Gimbal
       |                      |
       +----------+-----------+
                  v
          Offboard manager
                  |
                  v
             PX4 /fmu/in/*

PX4 status + battery + local position + offboard status
                  |
                  v
        offboard_safety_monitor
          |                  |
          v                  v
   safety/inhibit       safety/force_land

RC SBUS -> rc_parser -> kill_switch -> force disarm + system/killed
```

Các node tự lưu bản sao payload đầu vào của mình. `input_state_cache` chỉ lưu thời điểm nhận message và phát cờ timeout, không phải bộ nhớ payload trung tâm.

## Cấu trúc repository

| Đường dẫn | Nội dung |
|---|---|
| `ros2_ws/src/` | Chín ROS 2 packages của hệ thống bay |
| `landing_diagnostics/` | Ghi log, tính metric và đề xuất tuning từ CSV |
| `PX4_Control/` | Hướng dẫn và template patch cho PX4 |
| `Embedded_Plan.md` | Kế hoạch thiết kế embedded tổng thể |
| `PX4_Architecture.md` | Kiến trúc giao tiếp PX4/ROS 2 |
| `QGC_Calibration_Guide.md` | Quy trình calibration trên QGroundControl |
| `References/Link.txt` | Nguồn tham khảo và repository liên quan |

## ROS 2 packages

### `fsm_state_machine` (C++)

Điều khiển chuỗi trạng thái:

```text
SEARCH -> FOLLOW -> APPROACH -> LAND -> COMPLETE
```

- Timer 10 Hz.
- Nhận odometry, marker vision, altitude, RC, timeout flags và yêu cầu force-land.
- Phát state, mode planner, yaw rate, descent rate, gimbal request và disarm request.
- `SEARCH -> FOLLOW` sau 10 chu kỳ marker ổn định.
- Mất marker đưa `FOLLOW -> SEARCH` sau 5 giây và `APPROACH -> FOLLOW` sau 3 giây.
- `FOLLOW -> APPROACH` khi land switch bật và marker còn nhìn thấy.
- `APPROACH -> LAND` khi sai số căn chỉnh nhỏ hơn `0.3` và chênh cao nhỏ hơn `land_entry_height`.
- `LAND -> COMPLETE` khi có touchdown flag.

Tham số chính: `land_entry_height=0.5`, `yaw_search_rate=0.3`, `land_descent_rate=0.4`.

### `gimbal_control` (C++)

Tính góc mục tiêu theo state và hình học tương đối giữa UAV với H-Pad. Góc đi qua PID phần mềm và slew-rate limiter trước khi phát trên `gimbal/target_angle_deg`.

- Timer 20 Hz.
- SEARCH: `0` độ.
- FOLLOW/APPROACH: `-atan2(delta_h, d_horiz)` đổi sang độ.
- LAND: nội suy từ `-60` đến `-90` độ.
- Mặc định PID: `kp=1.0`, `ki=0.0`, `kd=0.1`.
- Đây là điều khiển open-loop ở cấp servo; `current_angle_` là góc lệnh trước đó, không phải feedback vật lý.

### `servo_control` (Python)

Điều khiển MG90S bằng `gpiozero.AngularServo` và `LGPIOFactory` trên Raspberry Pi/Linux.

```text
gimbal/target_angle_deg
  -> cộng home_angle_deg
  -> clamp [angle_min_deg, angle_max_deg]
  -> AngularServo.angle
```

Mặc định: GPIO `18`, PWM `600..2400 us`, góc servo `0..180`, home `90` độ. Package không chạy trực tiếp trên Windows hoặc PX4 flight controller.

### `input_state_cache` (C++)

Theo dõi freshness của EKF, vision, altitude và planner rồi phát `input_cache/timeout_flags`.

- Timer 20 Hz.
- Timeout mặc định: EKF `0.5 s`, vision `0.5 s`, altitude `0.5 s`, planner `1.0 s`.
- Chỉ kiểm tra thời điểm nhận, không kiểm tra tính hợp lệ của payload.

### `offboard_manager` (C++)

Chuyển state và planner output thành lệnh PX4:

- Timer 20 Hz.
- Phát `OffboardControlMode`, `TrajectorySetpoint` và `VehicleCommand`.
- Gửi 10 chu kỳ setpoint trước khi yêu cầu chuyển Offboard.
- Sau engage sẽ gửi lệnh arm tự động.
- SEARCH dừng vị trí và quay theo `yaw_search_rate`.
- FOLLOW/APPROACH dùng velocity từ planner.
- LAND dừng vận tốc ngang và dùng `land_descent_rate`.

Tham số chính: `required_engage_cycles=10`, `watchdog_timeout_sec=0.5`, `yaw_search_rate=0.3`, `land_descent_rate=0.4`.

### `offboard_safety_monitor` (C++)

Đánh giá RC override, EKF, pin và tuổi heartbeat Offboard. Các mức lỗi gồm `NONE`, `RC_OVERRIDE`, `EKF_UNHEALTHY`, `BATTERY_WARNING`, `BATTERY_CRITICAL`, `OFFBOARD_LOST_SHORT` và `OFFBOARD_LOST_LONG`.

- Timer 5 Hz.
- Pin warning dưới `30%`, critical dưới `15%`.
- Mất Offboard 1 giây: yêu cầu HOLD.
- Mất Offboard 5 giây: yêu cầu RTL.
- Phát lệnh PX4, `safety/inhibit_offboard` và `safety/force_land`.

### `rc_parser` (C++)

Đọc SBUS qua UART, giải mã 16 channel 11-bit, đổi thành microseconds và phát dữ liệu RC.

- Mặc định `/dev/ttyUSB0`, `100000 baud`, 8E2.
- Land switch channel mặc định `4`, kill switch channel `5` (zero-based).
- Ngưỡng thấp/cao `1200/1800 us`.
- Frame timeout `0.5 s`.
- Phụ thuộc Linux/POSIX (`termios`, `TCGETS2`, `BOTHER`), không build native trên Windows.

### `kill_switch` (C++)

Debounce kill channel trong ba chu kỳ, sau đó latch trạng thái killed và gửi `VEHICLE_CMD_COMPONENT_ARM_DISARM` với force-disarm (`param2=21196`). Phát thêm `system/killed` với QoS transient-local.

Kill switch hiện không tự kích hoạt khi mất RC và chưa được `offboard_manager` dùng để dừng setpoint.

### `xrce_bridge_manager` (Python)

Kiểm tra và khởi động `MicroXRCEAgent` bằng lệnh:

```bash
MicroXRCEAgent serial --dev <serial_port> -b <baudrate>
```

Mặc định dùng `/dev/ttyAMA0`, `921600 baud`, timeout kết nối `2.0 s`. Trạng thái kết nối được suy ra gián tiếp từ thời điểm nhận `/fmu/out/vehicle_status`.

## Messages

| Package | Message | Trường |
|---|---|---|
| `fsm_state_machine` | `VisionMarker` | `marker_visible`, `pixel_align_error` |
| `fsm_state_machine` | `AltEstimate` | `altitude`, `touchdown_flag` |
| `fsm_state_machine` | `RcFsmInput` | `land_switch`, `kill_switch` |
| `fsm_state_machine` | `TimeoutFlags` | `ekf_timeout`, `vision_timeout`, `alt_timeout`, `planner_timeout` |
| `rc_parser` | `RcChannelsRaw` | `int16[16] ch`, `valid`, `failsafe` |
| `offboard_manager` | `PlannerOutput` | `vx`, `vy`, `vz`, `yaw` |
| `offboard_manager` | `OffboardStatus` | `offboard_active`, `heartbeat_age_sec` |

Repository hiện không định nghĩa service hoặc action interface.

## Topics chính

### Input của FSM

| Topic | Type |
|---|---|
| `hpad/state_filtered` | `nav_msgs/msg/Odometry` |
| `hpad/pose` | `fsm_state_machine/msg/VisionMarker` |
| `alt_estimator/state` | `fsm_state_machine/msg/AltEstimate` |
| `rc/fsm_input` | `fsm_state_machine/msg/RcFsmInput` |
| `input_cache/timeout_flags` | `fsm_state_machine/msg/TimeoutFlags` |
| `safety/force_land` | `std_msgs/msg/Bool` |

### Output nội bộ

`fsm/state`, `cmd/yaw_rate`, `gimbal/state_request`, `planner/mode`, `planner/apf_gain`, `gimbal/align_error_cmd`, `cmd/vertical_descent_rate`, `cmd/disarm_request`, `gimbal/target_angle_deg`, `rc/fsm_input`, `rc/channels_raw`, `offboard/status`, `safety/inhibit_offboard`, `safety/force_land`, `system/killed`.

### Giao tiếp PX4

- Input từ PX4: `/fmu/out/vehicle_status`, `/fmu/out/vehicle_local_position`, `/fmu/out/battery_status`.
- Output đến PX4: `/fmu/in/offboard_control_mode`, `/fmu/in/trajectory_setpoint`, `/fmu/in/vehicle_command`.

Tên topic và message PX4 còn phụ thuộc phiên bản `px4_msgs`, firmware PX4 và cấu hình `dds_topics.yaml`.

## Luồng hoạt động

1. `xrce_bridge_manager` kết nối ROS 2 với PX4 qua Micro XRCE-DDS.
2. Vision, estimator, planner và RC đưa dữ liệu vào ROS 2.
3. `input_state_cache` phát các cờ timeout khi nguồn dữ liệu không còn mới.
4. FSM quyết định phase bay và phát lệnh cho planner, gimbal và Offboard manager.
5. Gimbal tính góc camera; `servo_control` biến góc điều khiển thành góc servo vật lý.
6. Offboard manager phát heartbeat và velocity setpoint đến PX4.
7. Safety monitor có thể yêu cầu HOLD, RTL, force-land hoặc inhibit Offboard.
8. Kill switch hoạt động độc lập để force disarm.

## Cài đặt và build

### Phụ thuộc

- ROS 2 với `ament_cmake` và `ament_python`.
- `rclcpp`, `rclpy`, `std_msgs`, `nav_msgs`, `px4_msgs`.
- `rosidl_default_generators`, `rosidl_default_runtime`.
- `MicroXRCEAgent`.
- Raspberry Pi: `gpiozero`, `lgpio`.
- Phần cứng UART và SBUS inverter phù hợp.
- QGroundControl cho calibration và tuning.

### Build ROS 2

Thực hiện trên Linux có ROS 2 đã cài:

```bash
cd ros2_ws
source /opt/ros/<ros_distro>/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build
source install/setup.bash
```

Hiện chưa có launch file, parameter YAML tập trung, CI hoặc bộ test tự động. Build nguyên trạng đang bị chặn bởi lỗi khai báo trùng `next_state` trong `fsm_state_machine/src/fsm_node.cpp`.

### Build diagnostics

```bash
cd landing_diagnostics
python3 -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt
```

`requirements.txt` gồm `pandas` và `numpy`. Logger còn cần môi trường ROS 2, `px4_msgs` và `fsm_state_machine`.

## Chạy hệ thống

Chưa có orchestration tự động; các node hiện phải chạy riêng:

```bash
source /opt/ros/<ros_distro>/setup.bash
source ros2_ws/install/setup.bash

ros2 run xrce_bridge_manager xrce_bridge_node
ros2 run input_state_cache input_cache_node
ros2 run rc_parser rc_node
ros2 run kill_switch kill_switch_node
ros2 run offboard_safety_monitor safety_monitor_node
ros2 run fsm_state_machine fsm_node
ros2 run gimbal_control gimbal_node
ros2 run offboard_manager offboard_node
ros2 run servo_control servo_node
```

Thứ tự trên chỉ là thứ tự khởi động đề xuất, không thay thế launch system hoặc readiness handshake. Không thử arm hoặc chạy với cánh lắp trong giai đoạn validation.

Một số lệnh kiểm tra thủ công hữu ích:

```bash
ros2 topic list
ros2 topic echo /fmu/out/vehicle_status
ros2 topic echo /fmu/out/battery_status
ros2 topic echo offboard/status
ros2 topic echo input_cache/timeout_flags
```

## Landing diagnostics

`landing_diagnostics` cung cấp pipeline phân tích log:

```text
flight_data_logger -> CSV -> metrics -> classify -> tuning -> report
```

Logger ghi dữ liệu khoảng 50 ms, gồm position, velocity, trajectory setpoint, trạng thái PX4, FSM state, touchdown và landed flag. Metrics chỉ lấy các dòng `fsm_state == 3` (LAND), sau đó tính:

- Sai số velocity trung bình, lớn nhất và độ lệch chuẩn.
- Sai số vận tốc X/Y.
- Tần số trội bằng FFT.
- Tương quan altitude và error.
- Drift ngang trong phase LAND.

Classifier nhận diện `underdamped`, `steady_state_offset`, `ground_effect` hoặc `setpoint_or_estimator_issue`. Tuning đề xuất thay đổi `MPC_XY_VEL_P_ACC`, `MPC_XY_VEL_D_ACC`, `MPC_XY_VEL_I_ACC`, `MPC_LAND_SPEED` và `MPC_LAND_ALT2` theo từng loại lỗi.

Chạy report:

```bash
python3 -m landing_diagnostics.report flight_log.csv --params-json current_params.json
```

File `current_params.json` là input được guide yêu cầu nhưng chưa có sẵn trong repository. Pipeline cũng chưa có sample log hoặc test dữ liệu mẫu; cần kiểm tra NaN, timestamp và giới hạn tham số trước khi dùng kết quả để tuning thật.

## Tích hợp PX4 và QGroundControl

- [PX4_Architecture.md](PX4_Architecture.md) mô tả kiến trúc PX4/ROS 2.
- [PX4_Control/Patch_Guide.md](PX4_Control/Patch_Guide.md) mô tả quy trình patch PX4.
- `PX4_Control/templates/velocity_pid_patch_template.cpp` là template điều chỉnh velocity PID.
- `PX4_Control/templates/param_template.c` là template khai báo parameter.
- `PX4_Control/templates/apply_patch.sh` áp dụng các patch trong `../patches/*.patch`.
- [QGC_Calibration_Guide.md](QGC_Calibration_Guide.md) mô tả calibration bằng QGroundControl.

Các file trong `PX4_Control/templates` chưa phải patch hoàn chỉnh: chưa khóa phiên bản PX4, chưa có patch thực tế, chưa cập nhật đầy đủ header/parameter declaration và chưa có test chứng minh tham số hoạt động.

## Kiểm thử và validation

Hiện có thể kiểm tra thủ công bằng `ros2 topic pub`, `ros2 topic echo`, debug logs và SITL theo các guide của từng package. Chưa có:

- Unit test C++/Python.
- Integration test ROS 2 hoặc launch test.
- Vector test cho SBUS.
- Test safety state machine, command acknowledgement và startup timeout.
- Test HIL, sample flight log hoặc CI build/lint.

Trước khi bay nên kiểm tra tối thiểu: FSM transitions, timeout startup, force-land, kill switch, HOLD/RTL, PX4 mode acknowledgement, planner velocity bounds và behavior khi XRCE/RC bị ngắt.

## Các vấn đề cần xử lý

### Mức cao

1. Sửa lỗi compile do khai báo `const auto next_state` trùng trong `fsm_node.cpp`.
2. Sửa FSM để đánh giá `effective_rc`; hiện `safety/force_land` chưa thực sự ép chuyển trạng thái.
3. Propagate `system/killed` đến Offboard manager và các node liên quan.
4. Không đánh giá EKF/pin là lỗi trước khi nhận message PX4 đầu tiên và xác nhận freshness.
5. Không tự arm nếu chưa xác nhận PX4 healthy, đúng mode và đã sẵn sàng.
6. Bổ sung launch file, readiness ordering và parameter YAML.

### Mức trung bình

- Dùng `planner_timeout` trong Offboard/FSM.
- Reset `force_land_requested` khi điều kiện pin trở lại bình thường hoặc định nghĩa rõ cơ chế latch.
- Thêm anti-windup, finite-value checks và bảo vệ chia cho 0 cho gimbal PID.
- Thêm timeout an toàn cho servo khi mất input.
- Kiểm tra return value của UART/ioctl và xử lý mất RC.
- Thêm giới hạn và validation cho velocity từ planner.
- Cải thiện XRCE health check thay vì chỉ suy ra từ `VehicleStatus`.
- Xử lý NaN, dữ liệu ngắn và timestamp lỗi trong diagnostics.

### Mức tài liệu và phát hành

- Đồng bộ tài liệu “4 state” với enum thực tế có 5 state, gồm `COMPLETE`.
- Khóa phiên bản ROS 2, PX4 và `px4_msgs`.
- Tạo patch PX4 có thể áp dụng và ghi rõ commit/version.
- Đồng bộ thiết kế cache trung tâm với implementation thực tế.
- Bổ sung bảng parameter tập trung và sơ đồ launch.

## Tài liệu tham khảo trong repository

- [Embedded_Plan.md](Embedded_Plan.md)
- [PX4_Architecture.md](PX4_Architecture.md)
- [QGC_Calibration_Guide.md](QGC_Calibration_Guide.md)
- [landing_diagnostics/Diagnostic_Guide.md](landing_diagnostics/Diagnostic_Guide.md)
- [ros2_ws/src/fsm_state_machine/FSM_Guide.md](ros2_ws/src/fsm_state_machine/FSM_Guide.md)
- [ros2_ws/src/gimbal_control/Gimbal_Guide.md](ros2_ws/src/gimbal_control/Gimbal_Guide.md)
- [ros2_ws/src/input_state_cache/ISC_Guide.md](ros2_ws/src/input_state_cache/ISC_Guide.md)
- [ros2_ws/src/offboard_manager/Offboard_Guide.md](ros2_ws/src/offboard_manager/Offboard_Guide.md)
- [ros2_ws/src/rc_parser/RC_Guide.md](ros2_ws/src/rc_parser/RC_Guide.md)
- [ros2_ws/src/xrce_bridge_manager/XRCE_Guide.md](ros2_ws/src/xrce_bridge_manager/XRCE_Guide.md)
- [References/Link.txt](References/Link.txt)