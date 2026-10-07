# VDT Project

Workspace prototype cho hệ thống UAV PX4 + ROS 2 thực hiện phát hiện, bám và hạ cánh lên H-Pad bằng vision. Hệ thống gồm bộ máy trạng thái bay, điều khiển Offboard, điều khiển gimbal/servo, nhận RC, các lớp giám sát an toàn, các node cầu nối giữa PX4 và module vision, các package vision (`aruco_detector`, `ekf_adapter`, `ibvs`), planner tránh vật cản `apf_planner` (APF / I-APF) kèm node ghép lệnh cho Offboard, gói message dùng chung `vdt_msgs`, và công cụ phân tích log sau chuyến bay.

> **Trạng thái:** Hệ thống đã có đủ các package và bộ test tới bước SITL. Đang chờ phần code module landing và bài test HITL.

## Mục lục

- [Kiến trúc tổng quan](#kiến-trúc-tổng-quan)
- [Cấu trúc repository](#cấu-trúc-repository)
- [Quy ước frame và đơn vị](#quy-ước-frame-và-đơn-vị)
- [ROS 2 packages](#ros-2-packages)
- [Giao diện giữa embedded và vision](#giao-diện-giữa-embedded-và-vision)
- [Messages](#messages)
- [Giao tiếp PX4](#giao-tiếp-px4)
- [Tham số tập trung](#tham-số-tập-trung)
- [Sơ đồ launch](#sơ-đồ-launch)
- [Luồng hoạt động](#luồng-hoạt-động)
- [Cài đặt và build](#cài-đặt-và-build)
- [Chạy hệ thống](#chạy-hệ-thống)

## Kiến trúc tổng quan

```text
PX4 flight controller
        |
        |  Micro XRCE-DDS
        v
 px4_state_bridge ──► /odom, TF (world -> base_link -> gimbal_link -> camera_optical_frame),
        |             alt_estimator/state
        |                         |
        |                         v
        |        Module vision: aruco_detector (aruco_node, depth_to_image_node),
        |                       ekf_adapter (ekf_node), ibvs (ibvs_controller)
        |                         |
        |        /hpad/detected, /hpad/bbox, /ekf/target_state,
        |        /ekf/tracking_mode, /ibvs/yaw_cmd, /ibvs/pitch_trim_deg
        |                         |
        |                         ├──► apf_planner (planner_node) ──► /apf/velocity_cmd, /apf/yaw_cmd
        |                         |                                        |
        |                         |        /ibvs/yaw_cmd ──────────────────┤
        |                         |                                        v
        |                         |                              planner_merge_node
        |                         |                                        |
        |                         |                         planner/velocity_setpoint
        |                         v                                        |
        |             vision_interface_bridge ──► vision/marker, /mission/phase
        |                         |                                        |
        v                         v                                        |
          input_state_cache (State Aggregator & Watchdog) <────────────────┤
                  |                                                        |
                  ├──────(input_cache/snapshot)──────► FSM + Gimbal        |
                  │                                          │             |
                  ▼                                          ▼             |
       offboard_safety_monitor ──────────────► Offboard manager <──────────┘
                  |                                          |
                  v                                          v
          safety/inhibit + safety/force_land            PX4 /fmu/in/*

RC SBUS -> rc_parser -> kill_switch -> force disarm + system/killed


takeoff (chạy tay, một lần) ──► /fmu/in/vehicle_command (ARM, NAV_TAKEOFF) ──► PX4

ibvs_controller ──► /ibvs/pitch_trim_deg ──► gimbal_control (cộng vào góc mục tiêu)

gimbal_control ──► gimbal/target_angle_deg ──► servo_control ──► /fmu/in/vehicle_command (ACTUATOR_TEST, MAIN 1) ──► PX4
```

## Cấu trúc repository

| Đường dẫn | Nội dung |
|---|---|
| `ros2_ws/src/` | Mười tắm ROS 2 packages của hệ thống bay, module vision, planner, cầu nối vision, message dùng chung và bringup |
| `ros2_ws/src/vdt_msgs/` | Message hub: toàn bộ 8 custom message của hệ thống |
| `ros2_ws/src/vdt_bringup/` | Launch orchestration và startup ordering |
| `ros2_ws/src/px4_state_bridge/` | Cầu nối trạng thái PX4 sang `/odom`, TF và `alt_estimator/state` |
| `ros2_ws/src/vision_interface_bridge/` | Cầu nối topic vision sang `vision/marker` và `/mission/phase` |
| `ros2_ws/src/aruco_detector/` | Phát hiện ArUco H-Pad, pose trong frame camera, mask depth |
| `ros2_ws/src/ekf_adapter/` | EKF ước lượng trạng thái H-Pad trong frame `world` |
| `ros2_ws/src/ibvs/` | Visual servoing: hiệu chỉnh pitch gimbal và lệnh yaw |
| `ros2_ws/src/apf_planner/` | Planner APF / I-APF tránh vật cản, node ghép `planner/velocity_setpoint`, bộ sinh point cloud thử nghiệm |
| `ros2_ws/src/takeoff/` | Lệnh cất cánh một lần qua uXRCE-DDS: arm, PX4 takeoff, chờ đạt độ cao rồi thoát |
| `landing_diagnostics/` | Ghi log, tính metric và đề xuất tuning từ CSV |
| `PX4_Control/` | Hướng dẫn và template patch cho PX4 |

## Quy ước frame và đơn vị

| Hạng mục | Quy ước |
|---|---|
| Frame gốc `world` | ENU (x Đông, y Bắc, z lên), trùng local frame của PX4 sau khi đổi sang ENU |
| Thân UAV `base_link` | FLU (x trước, y trái, z lên) |
| Camera `camera_optical_frame` | x phải, y xuống, z phía trước |
| `/odom`, `/ekf/target_state` | Frame `world`, vị trí tuyệt đối |
| `/hpad/pose`, `/hpad/position_camera` | Frame `camera_optical_frame`; quaternion thứ tự x, y, z, w của ROS |
| `/hpad/bbox` | Pixel của ảnh IR1 |
| `/apf/velocity_cmd` | `Twist` ENU, `linear.x` Đông, `linear.y` Bắc, `linear.z` lên (m/s) |
| `/apf/yaw_cmd`, `/ibvs/yaw_cmd` | Heading ENU tuyệt đối (rad, 0 là hướng Đông, ngược chiều kim đồng hồ là dương) |
| `PlannerOutput` | ENU: `vx` Đông, `vy` Bắc, `vz` lên (m/s), `yaw` là heading ENU (rad), `yaw = NaN` là giữ yaw hiện tại |
| Setpoint gửi PX4 | NED; `offboard_manager` tự đổi từ ENU: `(vx, vy, vz)_NED = (vy, vx, -vz)_ENU`, `yaw_NED = wrap(pi/2 - yaw_ENU)` |
| Góc gimbal `gimbal/target_angle_deg` | Độ; âm là camera chúi xuống |
| `/ibvs/pitch_trim_deg` | Độ; âm là chúi thêm xuống, cộng vào góc gimbal ở FOLLOW/APPROACH |
| `InputSnapshot.delta_h` | UAV cao hơn H-Pad là dương |
| `InputSnapshot.align_error`, `d_horiz` | Khoảng cách ngang UAV tới H-Pad (m) |
| `InputSnapshot.yaw_rate` | Tốc độ góc quanh trục yaw của UAV (rad/s) |

Stamp của ảnh, odom và TF phải cùng một đồng hồ (cùng thời gian thật hoặc cùng sim time).

## ROS 2 packages

Mỗi package gồm: sơ đồ luồng, bảng Input, bảng Output, bảng hành vi chính và link guide. Toàn bộ message `vdt_msgs/msg/*` xem [Messages](#messages).

### `vdt_msgs` (interface)

Message hub độc lập cho toàn workspace, không phụ thuộc package nội bộ nào khác. Mọi package chỉ `depend` vào `vdt_msgs` để lấy định nghĩa message, nhờ đó đồ thị phụ thuộc là DAG và không còn phụ thuộc vòng.

```text
vdt_msgs
├── Cảm biến:      AltEstimate, VisionMarker
├── State cache:   InputSnapshot, TimeoutFlags
├── Dẫn đường:     PlannerOutput, OffboardStatus
└── RC:            RcFsmInput, RcChannelsRaw
```

Xem [Msgs_Guide.md](ros2_ws/src/vdt_msgs/Msgs_Guide.md).

### `fsm_state_machine` (C++, 10 Hz)

Máy trạng thái bay: quyết định pha SEARCH → FOLLOW → APPROACH → LAND → COMPLETE và phát lệnh cho planner, gimbal, Offboard.

```text
input_cache/snapshot ─┐
rc/fsm_input ─────────┤
safety/force_land ────┼──► fsm_node ──┬──► fsm/state
system/killed ────────┘               ├──► planner/mode, gimbal/state_request
                                      ├──► planner/apf_gain, gimbal/align_error_cmd
                                      └──► cmd/vertical_descent_rate, cmd/disarm_request
```

**Input**

| Topic | Type | Nguồn | Ghi chú |
|---|---|---|---|
| `input_cache/snapshot` | `vdt_msgs/msg/InputSnapshot` | `input_state_cache` | Quá 200 ms không nhận thì coi là mất snapshot |
| `rc/fsm_input` | `vdt_msgs/msg/RcFsmInput` | `rc_parser` | `land_switch` hiệu lực = `land_switch \|\| safety/force_land` |
| `safety/force_land` | `std_msgs/msg/Bool` | `offboard_safety_monitor` | Ưu tiên cao nhất |
| `system/killed` | `std_msgs/msg/Bool` | `kill_switch` | QoS transient-local; `true` thì FSM thoát `update()`, không publish gì |

**Output**

| Topic | Type | Đích |
|---|---|---|
| `fsm/state` | `std_msgs/msg/UInt8` | `offboard_manager`, `vision_interface_bridge` |
| `planner/mode`, `gimbal/state_request` | `UInt8` (enum `State`: SEARCH=0 … COMPLETE=4) | planner, `gimbal_control` |
| `planner/apf_gain` | `Float` | planner |
| `gimbal/align_error_cmd` | `Float` | `gimbal_control` (chỉ APPROACH) |
| `cmd/vertical_descent_rate` | `Float` | `offboard_manager` |
| `cmd/disarm_request` | `Bool` | Chỉ ở COMPLETE |

**Chuyển state** (xét từ trên xuống, dòng đầu thỏa thì thắng)

| Ưu tiên | Điều kiện | Chuyển |
|---|---|---|
| 1 | `safety/force_land = true` | SEARCH/FOLLOW/APPROACH → LAND |
| 2 | `planner_timeout = true` (gồm mất snapshot > 200 ms; bỏ qua nếu `ignore_planner_timeout`) | FOLLOW/APPROACH → SEARCH |
| 3 | `marker_stable_count >= enter_follow_cycles` | SEARCH → FOLLOW |
| 4 | Mất marker > `follow_lost_timeout` (2.5 s) | FOLLOW → SEARCH |
| 5 | `land_switch && marker_detected && !land_inhibit` | FOLLOW → APPROACH |
| 6 | `!land_switch` | APPROACH → FOLLOW |
| 7 | Mất marker > `approach_lost_timeout` (1.5 s) | APPROACH → FOLLOW |
| 8 | `land_ok_count >= land_entry_cycles` | APPROACH → LAND |
| 9 | `touchdown = true` | LAND → COMPLETE (trạng thái cuối) |

**Bộ đếm**

| Bộ đếm | Quy tắc |
|---|---|
| `marker_stable_count` | Tăng khi thấy marker, `geometry_valid` và `abs(yaw_rate) < yaw_settle_rate`; sai một chu kỳ về 0 |
| `marker_lost_time` | Cộng `dt` khi không thấy marker; về 0 khi thấy lại; không reset khi đổi state |
| `land_ok_count` | Chỉ đếm ở APPROACH: `geometry_valid && align_error < align_threshold && delta_h < land_entry_height`; sai một chu kỳ về 0 |
| `land_inhibit` | Đặt khi rời APPROACH (trừ sang LAND) mà công tắc vẫn bật; xóa khi công tắc tắt; chỉ có tác dụng khi `land_requires_rearm = true` |

`geometry_valid = valid && isfinite(delta_h) && isfinite(d_horiz)`; `valid = false` nếu `align_error` hoặc `altitude` là NaN.

**Lệnh phát theo state**

| State | `planner/apf_gain` | `gimbal/align_error_cmd` | `cmd/vertical_descent_rate` |
|---|---|---|---|
| SEARCH | không publish | không publish | 0 |
| FOLLOW | 1.0 | không publish | 0 |
| APPROACH | 0.5 | `align_error` | `approach_descent_rate` nếu `marker_detected && geometry_valid && delta_h >= land_entry_height`, ngược lại 0 |
| LAND | 0.0 | không publish | `land_descent_rate` |
| COMPLETE | không publish | không publish | không publish (`cmd/disarm_request = true` mỗi chu kỳ) |

FSM không publish `cmd/yaw_rate`. Quét yaw ở SEARCH do `offboard_manager` đảm nhiệm; FSM chỉ kiểm tra yaw đã ổn định bằng `yaw_settle_rate`.

Xem [FSM_Guide.md](ros2_ws/src/fsm_state_machine/FSM_Guide.md).

### `gimbal_control` (C++, 20 Hz)

Tính góc pitch mục tiêu theo state và hình học UAV/H-Pad, làm mượt rồi phát góc lệnh.

```text
gimbal/state_request ─┐
input_cache/snapshot ─┼──► gimbal_node:  góc mục tiêu ──► PID ──► slew limiter ──► gimbal/target_angle_deg
/ibvs/pitch_trim_deg ─┘
```

**Input**

| Topic | Type | Nguồn | Ghi chú |
|---|---|---|---|
| `gimbal/state_request` | `std_msgs/msg/UInt8` | `fsm_state_machine` | Phase hiện tại |
| `input_cache/snapshot` | `vdt_msgs/msg/InputSnapshot` | `input_state_cache` | Dùng `delta_h`, `d_horiz`, `valid`; `valid = false` hoặc mất snapshot > 200 ms thì giữ góc cũ |
| `/ibvs/pitch_trim_deg` | `std_msgs/msg/Float32` | `ibvs_controller` | Giao ước: chỉ cộng ở FOLLOW/APPROACH; coi như 0 nếu không hữu hạn hoặc cũ hơn khoảng 0.3 s; tổng góc kẹp trong `[out_min_deg, out_max_deg]` |

**Output**

| Topic | Type | Đích |
|---|---|---|
| `gimbal/target_angle_deg` | `std_msgs/msg/Float32` (độ) | `servo_control`, `px4_state_bridge` (dựng TF camera), `ibvs_controller` |

**Góc mục tiêu theo state**

| State | Góc mục tiêu |
|---|---|
| SEARCH, COMPLETE | `0` độ |
| FOLLOW, APPROACH | `-atan2(delta_h, d_horiz)` đổi sang độ |
| LAND | Nội suy từ `-60` đến `-90` độ theo `delta_h / land_entry_height` |

PID mặc định `kp=1.0, ki=0.0, kd=0.1`, có anti-windup; đổi state thì reset `integral` và `prev_error`. Dữ liệu NaN/Inf bị loại. Góc phát ra là góc lệnh open-loop, không phải góc vật lý của servo.

Xem [Gimbal_Guide.md](ros2_ws/src/gimbal_control/Gimbal_Guide.md) và [IBVS_Guide.md](ros2_ws/src/ibvs/IBVS_Guide.md).

### `servo_control` (Python)

Điều khiển servo gimbal thông qua PX4: gửi lệnh `MAV_CMD_ACTUATOR_TEST` (310) tới output MAIN 1 qua Micro XRCE-DDS. Node không điều khiển GPIO, hoạt động được trong lúc bay, đầu ra open-loop.

```text
gimbal/target_angle_deg
   │
   ▼
servo_angle = home_angle_deg + góc lệnh   (kẹp [angle_min_deg, angle_max_deg])
   │
   ▼
value = -1 + 2·(servo_angle - angle_min)/(angle_max - angle_min)     (dải -1..1)
   │
   ▼
/fmu/in/vehicle_command  (lệnh 310, param5 = servo_function)  ──►  PX4 MAIN 1
```

**Input**

| Topic | Type | Nguồn |
|---|---|---|
| `gimbal/target_angle_deg` | `std_msgs/msg/Float32` (độ) | `gimbal_control` |

**Output**

| Topic | Type | Đích |
|---|---|---|
| `/fmu/in/vehicle_command` | `px4_msgs/msg/VehicleCommand` | PX4 (output MAIN 1) |

**Quy đổi** (mặc định `angle_min=0`, `angle_max=180`, `home=90`)

| Góc gimbal (độ) | `servo_angle` (độ) | `value` |
|---|---|---|
| 0 | 90 | 0.0 |
| -45 | 45 | -0.5 |
| -90 | 0 | -1.0 |

**Cơ chế gửi lệnh**

| Cơ chế | Hành vi |
|---|---|
| Giới hạn tần số | Chỉ gửi khi `value` đổi ít nhất `min_send_delta` và cách lệnh trước ít nhất `min_send_interval_sec` |
| Keepalive | Gửi lại giá trị hiện tại mỗi `keepalive_sec` để PX4 không thả output sau `command_timeout_sec`; chưa kiểm chứng trên phần cứng, tắt bằng `keepalive_sec:=0.0` |
| Mất lệnh gimbal | Quá `input_timeout_sec` (1.0 s): gửi về `home_angle_deg` một lần rồi ngừng gửi; NaN/Inf bị bỏ qua |
| Tắt node | Gửi NaN để PX4 nhả output |

Xem [Servo_Guide.md](ros2_ws/src/servo_control/Servo_Guide.md).

### `input_state_cache` (C++, 20 Hz)

Bộ tập hợp trạng thái trung tâm (State Aggregator & Watchdog): gom mọi nguồn, kiểm tra độ tươi và hợp lệ, tính hình học UAV/H-Pad rồi phát snapshot nguyên tử.

```text
hpad/state_filtered ─┐   (remap từ /ekf/target_state)
ekf/tracking_mode ───┤
odom ────────────────┼──► input_cache_node ──┬──► input_cache/snapshot
vision/marker ───────┤    (frame check →     └──► input_cache/timeout_flags
alt_estimator/state ─┤     watchdog →
planner/velocity_    ┘     sanitize →
  setpoint                 hình học)
```

**Input**

| Topic | Type | Nguồn | Timeout mặc định |
|---|---|---|---|
| `hpad/state_filtered` | `nav_msgs/msg/Odometry` | `ekf_node` (`/ekf/target_state`) | 0.5 s |
| `ekf/tracking_mode` | `std_msgs/msg/String` | `ekf_node` | 0.5 s |
| `odom` | `nav_msgs/msg/Odometry` | `px4_state_bridge` | 0.5 s |
| `vision/marker` | `vdt_msgs/msg/VisionMarker` | `vision_interface_bridge` | 0.5 s |
| `alt_estimator/state` | `vdt_msgs/msg/AltEstimate` | `px4_state_bridge` | 0.5 s |
| `planner/velocity_setpoint` | `vdt_msgs/msg/PlannerOutput` | `planner_merge_node` | 1.0 s (chỉ theo dõi thời điểm nhận) |

Node chạy kèm remap `hpad/state_filtered:=/ekf/target_state`. Nguồn chưa nhận lần nào được coi là timeout. `hpad/state_filtered` và `odom` có `frame_id` khác `world_frame` bị loại (đặt `world_frame` rỗng để tắt kiểm tra).

**Output**

| Topic | Type | Đích |
|---|---|---|
| `input_cache/snapshot` | `vdt_msgs/msg/InputSnapshot` | `fsm_state_machine`, `gimbal_control` |
| `input_cache/timeout_flags` | `vdt_msgs/msg/TimeoutFlags` | `offboard_manager`, `fsm_state_machine` |

**Tính hợp lệ và hình học**

| Giá trị | Điều kiện |
|---|---|
| `ekf_valid` | `hpad/state_filtered` và `ekf/tracking_mode` còn tươi, mode thuộc `TRACKING`/`PREDICTING`/`PREDICTING_DEGRADED` (`EXPIRED` hoặc chuỗi lạ là không hợp lệ), vị trí hữu hạn |
| `odom_valid` | `odom` còn tươi, vị trí hữu hạn |
| `alt_valid` | `alt_estimator/state` còn tươi, `altitude` hữu hạn |
| `valid` | `odom_valid && alt_valid` (không phụ thuộc có mục tiêu hay không) |
| `marker_detected` | `vision/marker` còn tươi, `marker_visible = true`, `pixel_align_error` hữu hạn |
| `touchdown` | `alt_valid && touchdown_flag` |
| `d_horiz`, `align_error` | `hypot(x_ekf - x_odom, y_ekf - y_odom)`; chỉ tính khi `ekf_valid && odom_valid`, ngược lại `NaN` |
| `delta_h` | `z_odom - z_ekf`; chỉ tính khi `ekf_valid && odom_valid`, ngược lại `NaN` |
| `planner_timeout` | Chưa từng nhận hoặc quá `planner_timeout_sec` kể từ lần nhận cuối |

Yêu cầu: publish `input_cache/snapshot` đều (khoảng cách giữa hai bản tin không quá 200 ms), điền `yaw_rate` (NaN thì FSM không vào được FOLLOW). `planner/velocity_setpoint` phải được publish đều ở mọi phase, nếu không `planner_timeout` bật ngay khi vào FOLLOW.

Xem [ISC_Guide.md](ros2_ws/src/input_state_cache/ISC_Guide.md).

### `offboard_manager` (C++, 20 Hz)

Đổi state và planner output thành lệnh PX4: chuỗi engage Offboard, đổi frame ENU sang NED, watchdog heartbeat.

```text
fsm/state ────────────────┐
planner/velocity_setpoint ┤
input_cache/timeout_flags ┼──► offboard_node ──┬──► /fmu/in/offboard_control_mode
system/killed ────────────┤                    ├──► /fmu/in/trajectory_setpoint
safety/inhibit_offboard ──┤                    ├──► /fmu/in/vehicle_command
/fmu/out/vehicle_status ──┤                    └──► offboard/status
/fmu/out/vehicle_local_  ─┘
  position
```

**Input**

| Topic | Type | Nguồn |
|---|---|---|
| `fsm/state` | `std_msgs/msg/UInt8` | `fsm_state_machine` |
| `planner/velocity_setpoint` | `vdt_msgs/msg/PlannerOutput` (ENU) | `planner_merge_node` |
| `input_cache/timeout_flags` | `vdt_msgs/msg/TimeoutFlags` | `input_state_cache` |
| `system/killed`, `safety/inhibit_offboard` | `std_msgs/msg/Bool` | `kill_switch`, `offboard_safety_monitor` (`true` thì reset engage và ngừng publish setpoint) |
| `/fmu/out/vehicle_status`, `/fmu/out/vehicle_local_position` | `px4_msgs` | PX4 (còn fresh trong `data_freshness_timeout_sec`) |

**Output**

| Topic | Type | Đích |
|---|---|---|
| `/fmu/in/offboard_control_mode` | `px4_msgs` (position=false, velocity=true) | PX4 (heartbeat) |
| `/fmu/in/trajectory_setpoint` | `px4_msgs` (NED) | PX4 |
| `/fmu/in/vehicle_command` | `px4_msgs` | PX4 (chuyển mode OFFBOARD, arm, HOLD khi failsafe) |
| `offboard/status` | `vdt_msgs/msg/OffboardStatus` | `offboard_safety_monitor` |

**Chuỗi engage** (khi chưa `offboard_active`; cất cánh bằng package `takeoff` hoặc PX4 takeoff, node không cất cánh)

```text
Đã bay?  ARMED + z_valid + -z >= min_engage_altitude_m (2.0 m), dữ liệu fresh
   │ không: reset chuỗi, không gửi gì
   ▼
Heartbeat + setpoint SEARCH × required_engage_cycles (10 chu kỳ)
   ▼
Yêu cầu OFFBOARD ──► chờ nav_state == OFFBOARD (fresh)
   ▼
Chờ health: EKF xy_valid/z_valid, PX4 không failsafe
   ▼
Arm ──► PX4 báo ARMED ──► offboard_active = true
```

Mode hoặc health timeout thì reset chuỗi và không arm.

**Setpoint khi `offboard_active`** (NED)

| FSM state | Velocity | Yaw |
|---|---|---|
| SEARCH | 0 | quay theo `yaw_search_rate` |
| FOLLOW, APPROACH | Từ `planner/velocity_setpoint` (clamp `max_horizontal_velocity`, `max_vertical_velocity`); planner stale hoặc non-finite thì 0 | Từ planner; NaN hoặc planner stale thì giữ yaw |
| LAND | Ngang 0, `vz = +land_descent_rate` (dương là xuống) | Giữ yaw |
| COMPLETE, state khác | 0 | yaw_rate 0 |

Planner stale = `timeout_flags.planner_timeout = true` hoặc không nhận planner quá `planner_timeout_sec`; chỉ ảnh hưởng FOLLOW/APPROACH. Watchdog heartbeat quá `watchdog_timeout_sec` thì vào failsafe (HOLD).

Xem [Offboard_Guide.md](ros2_ws/src/offboard_manager/Offboard_Guide.md).


### `takeoff` (Python, chạy một lần)

Arm và ra lệnh PX4 cất cánh tại chỗ, chờ đạt độ cao rồi thoát; không publish setpoint Offboard.

```text
/fmu/out/vehicle_status          ──┐
/fmu/out/vehicle_local_position  ──┼──► takeoff ──► /fmu/in/vehicle_command (ARM, NAV_TAKEOFF, NAV_LAND khi timeout)
system/killed                    ──┘
```

| Chiều | Mô tả |
|---|---|
| Input | `/fmu/out/vehicle_status`, `/fmu/out/vehicle_local_position` (`px4_msgs`), `system/killed` (`std_msgs/msg/Bool`, latched) |
| Output | `/fmu/in/vehicle_command` (`VEHICLE_CMD_COMPONENT_ARM_DISARM`, `VEHICLE_CMD_NAV_TAKEOFF`, `VEHICLE_CMD_NAV_LAND` khi timeout); exit code 0 nếu thành công, 1 nếu lỗi |

Từ chối chạy khi `system/killed = true` hoặc UAV đã armed. Độ cao do tham số PX4 `MIS_TAKEOFF_ALT` quyết định (param7 của `NAV_TAKEOFF` là NaN); `--alt` chỉ dùng để xác nhận và phải khớp tham số này. Thành công khi `nav_state = OFFBOARD` hoặc `-z >= 0.9 * --alt`; không đạt trong `--climb-timeout` thì gửi `NAV_LAND`. Không kiểm tra `safety/force_land` và `safety/inhibit_offboard`. Chạy tay sau khi hệ thống đã lên, không đưa vào launch.

Xem [Takeoff_Guide.md](ros2_ws/src/takeoff/Takeoff_Guide.md).

### `px4_state_bridge` (C++)

Cầu nối trạng thái PX4 (NED/FRD) sang ENU/FLU frame `world` cho vision và `input_state_cache`.

```text
/fmu/out/vehicle_odometry ──┐
/fmu/out/vehicle_land_      ├──► px4_state_bridge ──┬──► /odom
  detected                  │                       ├──► TF world → base_link → gimbal_link → camera_optical_frame
gimbal/target_angle_deg ────┘                       └──► alt_estimator/state (20 Hz)
```

**Input**

| Topic | Type | Nguồn |
|---|---|---|
| `/fmu/out/vehicle_odometry` | `px4_msgs` (NED/FRD) | PX4; bỏ bản tin có `pose_frame` không phải NED, NaN/Inf hoặc quaternion suy biến |
| `/fmu/out/vehicle_land_detected` | `px4_msgs` | PX4 |
| `gimbal/target_angle_deg` | `std_msgs/msg/Float32` | `gimbal_control` (góc lệnh, chưa nhận thì 0) |

**Output**

| Topic | Type | Đích | Ghi chú |
|---|---|---|---|
| `/odom` | `nav_msgs/msg/Odometry` | `input_state_cache`, `planner_node`, `ibvs_controller` | Frame `world`, ENU, orientation FLU; stamp là thời điểm ROS nhận |
| TF `world → base_link → gimbal_link → camera_optical_frame` | TF | `ekf_node` | Dùng `gimbal_pivot_xyz`, `camera_in_gimbal_xyz` và góc gimbal lệnh |
| `alt_estimator/state` | `vdt_msgs/msg/AltEstimate` | `input_state_cache` | Chỉ publish khi odometry còn tươi (`odom_timeout_sec`); `altitude` là độ cao ENU so với gốc local frame PX4; `touchdown_flag` từ `landed` (tùy chọn `ground_contact`) |

Xem [PX4_Bridge_Guide.md](ros2_ws/src/px4_state_bridge/PX4_Bridge_Guide.md).

### `vision_interface_bridge` (C++)

Cầu nối topic của module vision sang giao diện của embedded.

```text
/hpad/detected ─┐
/hpad/bbox ─────┼──► vision_interface_bridge ──┬──► vision/marker
/camera_info ───┤                              └──► /mission/phase (10 Hz)
fsm/state ──────┘
```

**Input**

| Topic | Type | Nguồn |
|---|---|---|
| `/hpad/detected` | `std_msgs/msg/Bool` | `aruco_node` |
| `/hpad/bbox` | `vision_msgs/msg/BoundingBox2D` | `aruco_node` |
| `/camera_info` | `sensor_msgs/msg/CameraInfo` | Camera driver (lấy kích thước ảnh; dự phòng `image_width`/`image_height`) |
| `fsm/state` | `std_msgs/msg/UInt8` | `fsm_state_machine` |

**Output**

| Topic | Type | Đích |
|---|---|---|
| `vision/marker` | `vdt_msgs/msg/VisionMarker` | `input_state_cache` |
| `/mission/phase` | `std_msgs/msg/String` | `ekf_node`, `planner_node`, `planner_merge_node`, `ibvs_controller` |

**Quy tắc**

| Sự kiện đầu vào | Kết quả |
|---|---|
| `/hpad/bbox` đến | `vision/marker`: `marker_visible = true`, `pixel_align_error` chuẩn hóa theo nửa kích thước ảnh |
| `/hpad/detected = false` | `vision/marker`: `marker_visible = false`, `pixel_align_error = NaN` |
| `/hpad/detected = true` | Không phát gì, chờ `bbox` của cùng khung hình |

| `fsm/state` | `/mission/phase` |
|---|---|
| 0 / 1 / 2 / 3 | `SEARCH` / `FOLLOW` / `APPROACH` / `LAND` |
| 4 (COMPLETE), giá trị lạ, chưa nhận hoặc cũ hơn `fsm_state_timeout_sec` | `IDLE` |

Xem [Vision_Interface_Guide.md](ros2_ws/src/vision_interface_bridge/Vision_Interface_Guide.md).

### `aruco_detector` (Python)

Phát hiện ArUco bằng `cv2.aruco` trên ảnh IR1 (mono8), giải PnP `SOLVEPNP_IPPE_SQUARE`. Gồm hai node.

```text
ảnh IR1 + /camera_info ──► aruco_node ──┬──► /hpad/detected, /hpad/pose, /hpad/position_camera
                                        ├──► /hpad/bbox
                                        └──► /hpad/annotated

/depth_camera + /hpad/mask_polygon ──► depth_to_image_node ──► /depth_camera/image_mono
```

**`aruco_node`**

| Chiều | Topic | Type | Ghi chú |
|---|---|---|---|
| Input | `image_topic` (`/camera`) | `sensor_msgs/msg/Image` | Ảnh IR1, BEST_EFFORT |
| Input | `camera_info_topic` (`/camera_info`) | `sensor_msgs/msg/CameraInfo` | BEST_EFFORT; chưa có thì dùng matrix dự phòng theo `fallback_horizontal_fov_rad`, và chặn publish pose nếu `require_camera_info = true` |
| Output | `/hpad/detected` | `std_msgs/msg/Bool` | Mọi frame |
| Output | `/hpad/annotated` | `sensor_msgs/msg/Image` (bgr8) | Mọi frame |
| Output | `/hpad/pose` | `geometry_msgs/msg/PoseStamped` | Chỉ khi thấy marker, frame `camera_frame_id` |
| Output | `/hpad/position_camera` | `geometry_msgs/msg/PointStamped` | Chỉ khi thấy marker; tới `ekf_node` |
| Output | `/hpad/bbox` | `vision_msgs/msg/BoundingBox2D` | Chỉ khi thấy marker (pixel) |

Chọn marker đúng `marker_id`, loại detection có khoảng cách < `min_detection_distance_m` hoặc `z` < `min_z_m`; chỉ detection đầu tiên được publish. `aruco_node` hiện chưa publish `/hpad/mask_polygon` (tham số `mask_margin_percent` khai báo nhưng chưa dùng).

**`depth_to_image_node`**

| Chiều | Topic | Type | Ghi chú |
|---|---|---|---|
| Input | `/depth_camera` | `sensor_msgs/msg/Image` (`32FC1`/`16UC1`) | Phải align với IR1, cùng độ phân giải |
| Input | `/hpad/mask_polygon` | `geometry_msgs/msg/PolygonStamped` | Chưa có publisher; có polygon (≥ 3 đỉnh, stamp lệch ≤ 0.2 s) thì vùng đó gán NaN |
| Output | `/depth_camera/image_mono` | `sensor_msgs/msg/Image` (mono8) | Depth kẹp 0.2-8.0 m, ánh xạ 0-255 |

Xem [Aruco_Guide.md](ros2_ws/src/aruco_detector/Aruco_Guide.md).

### `ekf_adapter` (Python)

Ước lượng trạng thái H-Pad `[x, y, z, vx, vy, vz]` trong frame `world` (vận tốc không đổi, nhiễu gia tốc trắng).

```text
/hpad/position_camera ──┐
TF (tại stamp ảnh) ─────┼──► ekf_node:  đổi sang world + ma trận R
/mission/phase ─────────┘               ──► TargetTracker (gate, NIS, tái bắt, decay)
                                        ──► timer 50 Hz: ngoại suy tới "now"
                                                  │
                                                  ├──► /ekf/target_state
                                                  └──► /ekf/tracking_mode
```

**Input**

| Topic | Type | Nguồn | Ghi chú |
|---|---|---|---|
| `/hpad/position_camera` | `geometry_msgs/msg/PointStamped` | `aruco_node` | Loại nếu thiếu stamp/`frame_id`, không hữu hạn hoặc `z <= 0` |
| TF `world ← camera_optical_frame` | TF | `px4_state_bridge` | Tra đúng tại stamp ảnh; lỗi TF thì bỏ measurement, tăng `tf_rejects` |
| `/mission/phase` | `std_msgs/msg/String` | `vision_interface_bridge` | `APPROACH` hoặc `FOLLOW`; giá trị khác coi như `FOLLOW` |
| `/odom` | `nav_msgs/msg/Odometry` | `px4_state_bridge` | Chỉ `odom_tf_node` dùng (tắt khi dùng `px4_state_bridge`) |

**Output**

| Topic | Type | Đích | Ghi chú |
|---|---|---|---|
| `/ekf/target_state` | `nav_msgs/msg/Odometry` (`world`) | `input_state_cache`, `planner_node`, `ibvs_controller` | Có covariance vị trí và vận tốc; không ước lượng orientation; vận tốc trong hệ `world`; publish liên tục kể cả khi `EXPIRED` |
| `/ekf/tracking_mode` | `std_msgs/msg/String` | `input_state_cache`, `planner_node`, `ibvs_controller` | Xem bảng mode |

`age` là tuổi của measurement hợp lệ gần nhất; `limit` là 1.0 s ở APPROACH và 2.0 s ở FOLLOW:

| Mode | Điều kiện |
|---|---|
| `TRACKING` | Đang detect và `age <= 0.25 s` |
| `PREDICTING` | `age <= 0.5 * limit` (APPROACH 0.5 s, FOLLOW 1.0 s) |
| `PREDICTING_DEGRADED` | `age <= limit` (APPROACH 1.0 s, FOLLOW 2.0 s) |
| `EXPIRED` | Quá `limit` hoặc chưa từng có measurement |

Consumer phải lọc theo mode, không dựa vào độ tươi của timestamp. Trong hệ thống này `px4_state_bridge` đã publish toàn bộ chuỗi TF nên chạy `ekf.launch.py` với `publish_odom_tf:=false publish_camera_tf:=false`. Tên topic trạng thái đặt qua tham số `state_topic` (đặt `/ekf/target_state`).

Xem [EKF_Guide.md](ros2_ws/src/ekf_adapter/EKF_Guide.md).

### `ibvs` (Python, 30 Hz + theo sự kiện `bbox`)

Visual servoing theo pixel: không ra góc pitch tuyệt đối, chỉ hiệu chỉnh nhỏ để bù sai số EKF và sai lệch góc servo, và tính lệnh yaw.

```text
/hpad/bbox, /hpad/detected ─┐
/camera_info ───────────────┤
/odom, /gimbal/target_      ├──► ibvs_controller ──┬──► /ibvs/pitch_trim_deg
  angle_deg ────────────────┤                      └──► /ibvs/yaw_cmd
/ekf/target_state, /ekf/    │
  tracking_mode ────────────┤
/mission/phase ─────────────┘
```

**Input**

| Topic | Type | Nguồn | Ghi chú |
|---|---|---|---|
| `/hpad/bbox`, `/hpad/detected` | `BoundingBox2D`, `Bool` | `aruco_node` | Pixel chỉ dùng khi `detected = true` |
| `/camera_info` | `CameraInfo` | Camera driver | Ghi đè `focal_x/focal_y/u0/v0` khi `K[0] > 0` |
| `/odom` | `Odometry` | `px4_state_bridge` | Lấy yaw thực |
| `/gimbal/target_angle_deg` | `Float32` | `gimbal_control` | Chưa nhận thì giả định -30° |
| `/ekf/target_state`, `/ekf/tracking_mode` | `Odometry`, `String` | `ekf_node` | Thiếu thì chạy chế độ chỉ dùng pixel |
| `/mission/phase` | `String` | `vision_interface_bridge` | |

**Output**

| Topic | Type | Đích |
|---|---|---|
| `/ibvs/pitch_trim_deg` | `std_msgs/msg/Float32` (độ) | `gimbal_control` |
| `/ibvs/yaw_cmd` | `std_msgs/msg/Float64` (rad, heading ENU tuyệt đối) | `planner_merge_node` |

**Hành vi theo phase**

| Phase | Hiệu chỉnh pitch | Yaw |
|---|---|---|
| IDLE (mặc định khi chưa nhận phase) | 0 | `yaw_cmd` bằng yaw thực, không điều khiển |
| SEARCH | 0 | `yaw_cmd` bằng yaw thực; IBVS không quét, việc quét yaw do `offboard_manager` điều khiển |
| FOLLOW, APPROACH | Theo pixel, kẹp ±`pitch_trim_limit_deg` | Bám pixel kèm feedforward tiếp tuyến khi có EKF |
| LAND | 0 | Chỉnh nhẹ theo pixel, chỉ khi có bbox mới; mất marker thì giữ giá trị cuối |

Tên topic cố định trong code, đổi bằng remap. Xem [IBVS_Guide.md](ros2_ws/src/ibvs/IBVS_Guide.md).

### `apf_planner` (Python)

Planner tránh vật cản cho FOLLOW/APPROACH. Gồm ba node và hai lõi thuần Python (`apf_core.py` 2D, `iapf_core.py` 3D).

```text
/odom, /ekf/target_state, /ekf/tracking_mode, /mission/phase, /map_generator/global_cloud
                              │
                              ▼
                        planner_node (30 Hz) ──► /apf/velocity_cmd, /apf/yaw_cmd, /apf/force_markers
                                                          │
/ibvs/yaw_cmd, /mission/phase ──────────────────────────► planner_merge_node (20 Hz)
                                                          │
                                                          ▼
                                                planner/velocity_setpoint

pointcloud_generator (1 Hz) ──► /map_generator/global_cloud
```

**`planner_node`**

| Chiều | Topic | Type | Ghi chú |
|---|---|---|---|
| Input | `/odom` | `nav_msgs/msg/Odometry` | BEST_EFFORT, frame `world` |
| Input | `target_topic` (`/ekf/target_state`) | `nav_msgs/msg/Odometry` | Vị trí và `twist.linear` của H-Pad |
| Input | `/ekf/tracking_mode` | `std_msgs/msg/String` | Phải thuộc `allowed_tracking_modes` (`TRACKING`, `PREDICTING`) |
| Input | `/mission/phase` | `std_msgs/msg/String` | Chỉ `FOLLOW`, `APPROACH` |
| Input | `/map_generator/global_cloud` | `sensor_msgs/msg/PointCloud2` | Khi `obstacle_source` là `pointcloud`/`both` |
| Output | `/apf/velocity_cmd` | `geometry_msgs/msg/Twist` (ENU) | Publish mỗi tick, bằng 0 khi không tính lệnh |
| Output | `/apf/yaw_cmd` | `std_msgs/msg/Float64` | Heading ENU từ UAV tới H-Pad; chỉ publish khi đang tính lệnh; 0 rad khi cách H-Pad < 0.1 m theo phương ngang |
| Output | `/apf/force_markers` | `visualization_msgs/msg/MarkerArray` | Debug RViz |

Chỉ tính lệnh khi đồng thời: phase đúng, mode đúng, odom và target tươi trong `data_timeout_sec` (0.5 s). Message sai `world_frame` hoặc non-finite bị loại; kết quả non-finite thay bằng vận tốc 0.

| Hạng mục | APPROACH | FOLLOW |
|---|---|---|
| Goal | Vị trí H-Pad (3D) | Điểm stand-off cách H-Pad `follow_distance` (3D), giữ độ cao `target_altitude`; cộng lead theo vận tốc target và feedforward vận tốc ngang khi target di chuyển |
| Gain lực đẩy | `k_rep_approach` | `k_rep` |

| Lõi (`planner_type`) | Cơ chế |
|---|---|
| `apf` | Lực 2D ngang; vật cản thêm lực tiếp tuyến; giảm tốc tuyến tính từ `d_slow` đến `goal_threshold`; `vz` điều khiển P theo độ cao (`k_z`, deadband 8 cm, kẹp `vz_max`) |
| `iapf` | Lực 3D, đẩy kiểu GNRON; phát hiện cực tiểu cục bộ có hysteresis (`iapf_f_enter`/`iapf_f_exit`); thoát bằng lực tiếp tuyến 3D chọn trong `iapf_n_tangent` ứng viên; giảm dao động hướng; `vz` kẹp `vz_max` |

| `obstacle_source` | Nguồn vật cản |
|---|---|
| `pointcloud` (mặc định) | Voxel downsample, lấy điểm trong `cloud_query_radius` (mặc định `2·d0`), mỗi cụm một điểm gần nhất, tối đa `max_cloud_points` |
| `sdf` | Cylinder từ `world_sdf` (model `cyl_*`/`cylinder_obs_*`) |
| `both` | Cả hai |

Lõi được reset khi đổi phase. `k_rep` cần hiệu chỉnh lại khi chuyển giữa cylinder và point cloud.

**`planner_merge_node`**

| Chiều | Topic | Type | Ghi chú |
|---|---|---|---|
| Input | `/apf/velocity_cmd`, `/apf/yaw_cmd` | `Twist`, `Float64` | `planner_node` |
| Input | `/ibvs/yaw_cmd` | `Float64` | `ibvs_controller` |
| Input | `/mission/phase` | `String` | Quá `phase_timeout_sec` coi là `IDLE` |
| Output | `planner/velocity_setpoint` | `vdt_msgs/msg/PlannerOutput` (ENU) | `offboard_manager`, `input_state_cache` |

| Tình huống | Đầu ra |
|---|---|
| FOLLOW/APPROACH | Vận tốc APF; yaw theo `yaw_source` (`ibvs_apf`: IBVS → APF → NaN; hoặc `ibvs`, `apf`, `hold`), mỗi nguồn phải tươi trong `yaw_timeout_sec` |
| Phase khác | Vận tốc 0, yaw NaN (vẫn publish để không có `planner_timeout` giả) |
| Mất `/apf/velocity_cmd` quá `upstream_timeout_sec` (FOLLOW/APPROACH) | Hover thêm `stale_hover_sec`, sau đó ngừng publish để `planner_timeout` lan tới FSM và Offboard |
| Vận tốc non-finite | Hover |

**`pointcloud_generator`** (node `apf_pointcloud_generator`): sinh "rừng" trụ ngẫu nhiên (`num_obs=35`, `map_size=25 m`, `clear_radius=2 m`), publish `/map_generator/global_cloud` (`PointCloud2`, frame `world`) ở 1 Hz, tùy chọn TF tĩnh `world -> map`.

Xem [APF_Guide.md](ros2_ws/src/apf_planner/APF_Guide.md).

### `offboard_safety_monitor` (C++, 5 Hz)

Đánh giá an toàn độc lập: RC override, EKF, pin, tuổi heartbeat Offboard.

```text
/fmu/out/vehicle_status ──────┐
/fmu/out/vehicle_local_pos. ──┼──► safety_monitor_node ──┬──► safety/inhibit_offboard
/fmu/out/battery_status ──────┤                          ├──► safety/force_land
offboard/status ──────────────┘                          └──► /fmu/in/vehicle_command (HOLD/RTL)
```

**Input**

| Topic | Type | Nguồn |
|---|---|---|
| `/fmu/out/vehicle_status`, `/fmu/out/vehicle_local_position`, `/fmu/out/battery_status` | `px4_msgs` | PX4 (chỉ đánh giá khi đã nhận và còn fresh trong `data_freshness_timeout_sec`) |
| `offboard/status` | `vdt_msgs/msg/OffboardStatus` | `offboard_manager` |

**Output**

| Topic | Type | Đích |
|---|---|---|
| `safety/inhibit_offboard` | `std_msgs/msg/Bool` | `offboard_manager` |
| `safety/force_land` | `std_msgs/msg/Bool` | `fsm_state_machine` |
| `/fmu/in/vehicle_command` | `px4_msgs` | PX4 |

| Mức lỗi | Điều kiện | Kết quả |
|---|---|---|
| `BATTERY_WARNING` | Pin dưới `30%` | `safety/force_land = true`, mặc định latch đến khi node restart (`force_land_latched`) |
| `BATTERY_CRITICAL` | Pin dưới `15%` | RTL và inhibit |
| `OFFBOARD_LOST_SHORT` | Mất heartbeat Offboard 1 s | Yêu cầu HOLD |
| `OFFBOARD_LOST_LONG` | Mất heartbeat Offboard 5 s | Yêu cầu RTL |
| `RC_OVERRIDE`, `EKF_UNHEALTHY` | Theo đánh giá của node | Xem Safety_Guide |

`force_land_latched=false` chỉ dùng cho bench/test với battery fresh và hợp lệ. Xem [Safety_Guide.md](ros2_ws/src/offboard_safety_monitor/Safety_Guide.md).

### `rc_parser` (C++, 50 Hz)

Đọc SBUS, giải mã 16 kênh 11-bit, đổi sang microseconds.

```text
RC receiver (SBUS, UART) ──► SbusUart ──► sbus_decode_frame ──► rc_validate ──┬──► rc/fsm_input
                                                                              └──► rc/channels_raw
```

**Input:** SBUS qua UART (`serial_device`, `baudrate`).

**Output**

| Topic | Type | Đích | Ghi chú |
|---|---|---|---|
| `rc/fsm_input` | `vdt_msgs/msg/RcFsmInput` | `fsm_state_machine` | `land_switch`, `kill_switch`; chỉ `true` khi `valid` |
| `rc/channels_raw` | `vdt_msgs/msg/RcChannelsRaw` | `kill_switch` | 16 kênh thô, `valid`, `failsafe` |

`valid = false` khi quá `frame_timeout` (0.5 s) chưa có frame hợp lệ, có cờ failsafe, hoặc có kênh ngoài `[800, 2200]`; khi đó node vẫn publish định kỳ `valid=false`, `failsafe=true`. Land switch mặc định kênh `4`, kill switch kênh `5`, ngưỡng `1200/1800 us`.

Xem [RC_Guide.md](ros2_ws/src/rc_parser/RC_Guide.md).

### `kill_switch` (C++, 50 Hz)

Công tắc khẩn cấp, chạy độc lập với FSM và Offboard.

```text
rc/channels_raw ──► kill_switch_node: rc_get_kill_switch ──► debounce (3 chu kỳ) ──► latch ──┬──► /fmu/in/vehicle_command (force disarm)
                                                                                             └──► system/killed
```

**Input**

| Topic | Type | Nguồn |
|---|---|---|
| `rc/channels_raw` | `vdt_msgs/msg/RcChannelsRaw` | `rc_parser` |

**Output**

| Topic | Type | Đích | Ghi chú |
|---|---|---|---|
| `/fmu/in/vehicle_command` | `px4_msgs` | PX4 | `VEHICLE_CMD_COMPONENT_ARM_DISARM`, `param2=21196` (force disarm) |
| `system/killed` | `std_msgs/msg/Bool` | `fsm_state_machine`, `offboard_manager` | QoS transient-local |

Đã trigger thì latch, không tự phục hồi (muốn reset phải restart node). Kill switch không tự kích hoạt khi mất RC. Khi `system/killed` đã phát, FSM dừng update và `offboard_manager` reset engage, dừng heartbeat/setpoint.

Xem [Kill_Switch_Guide.md](ros2_ws/src/kill_switch/Kill_Switch_Guide.md).

### `xrce_bridge_manager` (Python, 1 Hz)

Giữ `MicroXRCEAgent` luôn chạy; không tạo topic nào.

```text
xrce_bridge_node ──► spawn/kiểm tra `MicroXRCEAgent serial --dev <serial_port> -b <baudrate>`
/fmu/out/vehicle_status ──► theo dõi độ tươi ──► connected / reconnect
```

| Chiều | Mô tả |
|---|---|
| Input | `/fmu/out/vehicle_status` (`px4_msgs`) |
| Output | Tiến trình `MicroXRCEAgent`; log trạng thái `connected`, `retry_count` |

`connected` yêu cầu Agent còn sống và `VehicleStatus` còn fresh (`connection_timeout_sec`, mặc định `2.0 s`); process chết hoặc status stale thì tăng `retry_count` và khởi động lại Agent.

Xem [XRCE_Guide.md](ros2_ws/src/xrce_bridge_manager/XRCE_Guide.md).

### `vdt_bringup`

Launch orchestration cho các node của workspace và nạp [System_Params.yaml](System_Params.yaml). Không khởi động module vision và `apf_planner`. Xem [Bringup_Guide.md](ros2_ws/src/vdt_bringup/Bringup_Guide.md).

## Giao diện giữa embedded và vision

| Topic | Kiểu | Từ | Đến |
|---|---|---|---|
| `/odom` | `nav_msgs/Odometry` (`world`, ENU) | `px4_state_bridge` | `planner_node`, `ibvs_controller`, `input_state_cache` |
| TF `world` -> `base_link` -> `gimbal_link` -> `camera_optical_frame` | TF | `px4_state_bridge` | `ekf_node` |
| `/mission/phase` | `std_msgs/String` | `vision_interface_bridge` | `ekf_node`, `planner_node`, `planner_merge_node`, `ibvs_controller` |
| `/gimbal/target_angle_deg` | `std_msgs/Float32` | `gimbal_control` | `ibvs_controller`, `px4_state_bridge`, `servo_control` |
| `/hpad/detected`, `/hpad/bbox` | `Bool`, `BoundingBox2D` | `aruco_node` | `vision_interface_bridge`, `ibvs_controller` |
| `/hpad/position_camera` | `PointStamped` (camera frame) | `aruco_node` | `ekf_node` |
| `/ekf/target_state` | `nav_msgs/Odometry` (`world`) | `ekf_node` | `input_state_cache` (remap thành `hpad/state_filtered`), `planner_node`, `ibvs_controller` |
| `/ekf/tracking_mode` | `std_msgs/String` | `ekf_node` | `input_state_cache`, `planner_node`, `ibvs_controller` |
| `/ibvs/pitch_trim_deg` | `std_msgs/Float32` | `ibvs_controller` | `gimbal_control` |
| `/ibvs/yaw_cmd` | `std_msgs/Float64` | `ibvs_controller` | `planner_merge_node` |
| `/apf/velocity_cmd`, `/apf/yaw_cmd` | `Twist`, `Float64` | `planner_node` | `planner_merge_node` |
| `planner/velocity_setpoint` | `vdt_msgs/PlannerOutput` (ENU) | `planner_merge_node` | `offboard_manager`, `input_state_cache` |
| `vision/marker` | `vdt_msgs/VisionMarker` | `vision_interface_bridge` | `input_state_cache` |
| `alt_estimator/state` | `vdt_msgs/AltEstimate` | `px4_state_bridge` | `input_state_cache` |

Yêu cầu với node phát hiện: publish `/hpad/detected` ở mọi khung hình và chỉ publish `/hpad/bbox` khi phát hiện thành công. `/camera_info` cấp cho `aruco_node` và `ibvs_controller` phải cùng camera với ảnh phát hiện marker (IR1).

## Messages

Toàn bộ message tùy chỉnh nằm trong `vdt_msgs`; không package nghiệp vụ nào định nghĩa message riêng.

| Message | Trường | Producer | Topic mặc định | Consumer chính |
|---|---|---|---|---|
| `InputSnapshot` | `valid`, `marker_detected`, `align_error`, `altitude`, `delta_h`, `d_horiz`, `yaw_rate`, `touchdown`, `planner_timeout` | `input_state_cache` | `input_cache/snapshot` | `fsm_state_machine`, `gimbal_control` |
| `TimeoutFlags` | `ekf_timeout`, `vision_timeout`, `alt_timeout`, `planner_timeout` | `input_state_cache` | `input_cache/timeout_flags` | `offboard_manager`, `fsm_state_machine` |
| `VisionMarker` | `marker_visible`, `pixel_align_error` | `vision_interface_bridge` | `vision/marker` | `input_state_cache` |
| `AltEstimate` | `altitude`, `touchdown_flag` | `px4_state_bridge` | `alt_estimator/state` | `input_state_cache` |
| `PlannerOutput` | `vx`, `vy`, `vz` (ENU, m/s), `yaw` (heading ENU, rad, NaN là giữ yaw) | `planner_merge_node` | `planner/velocity_setpoint` | `offboard_manager`, `input_state_cache` |
| `OffboardStatus` | `offboard_active`, `heartbeat_age_sec` | `offboard_manager` | `offboard/status` | `offboard_safety_monitor` |
| `RcFsmInput` | `land_switch`, `kill_switch` | `rc_parser` | `rc/fsm_input` | `fsm_state_machine` |
| `RcChannelsRaw` | `int16[16] ch`, `valid`, `failsafe` | `rc_parser` | `rc/channels_raw` | `kill_switch` |

Ghi chú:

- `InputSnapshot.delta_h`, `d_horiz`, `align_error` là `NaN` khi không có mục tiêu hợp lệ; `altitude` là `NaN` khi `alt_estimator/state` không hợp lệ.
- `InputSnapshot.yaw_rate` là rad/s quanh trục yaw; `NaN` làm FSM không vào được FOLLOW. `align_error` hoặc `altitude` là `NaN` thì FSM đặt `valid = false`.
- `VisionMarker.pixel_align_error` là sai số chuẩn hóa theo nửa kích thước ảnh; là `NaN` khi `marker_visible = false`. FSM không dùng giá trị này.

Repository hiện không định nghĩa service hoặc action interface. Các topic vision dùng message chuẩn (`geometry_msgs`, `nav_msgs`, `sensor_msgs`, `std_msgs`, `vision_msgs`, `visualization_msgs`).

## Giao tiếp PX4

| Hướng | Topic | Dùng bởi |
|---|---|---|
| Từ PX4 | `/fmu/out/vehicle_status` | `offboard_manager`, `offboard_safety_monitor`, `xrce_bridge_manager`, `takeoff` |
| Từ PX4 | `/fmu/out/vehicle_local_position` | `offboard_manager`, `offboard_safety_monitor`, `takeoff` |
| Từ PX4 | `/fmu/out/battery_status` | `offboard_safety_monitor` |
| Từ PX4 | `/fmu/out/vehicle_odometry` | `px4_state_bridge` |
| Từ PX4 | `/fmu/out/vehicle_land_detected` | `px4_state_bridge` |
| Tới PX4 | `/fmu/in/offboard_control_mode`, `/fmu/in/trajectory_setpoint` | `offboard_manager` |
| Tới PX4 | `/fmu/in/vehicle_command` | `offboard_manager`, `offboard_safety_monitor`, `kill_switch`, `servo_control`, `takeoff` |

Tên topic và message PX4 còn phụ thuộc phiên bản `px4_msgs`, firmware PX4 và cấu hình `dds_topics.yaml`. `px4_state_bridge` cho phép đổi tên topic qua tham số.

## Tham số tập trung

Các tham số runtime của hệ thống bay được khai báo trong [System_Params.yaml](System_Params.yaml). Khi chạy bằng `vdt_bringup`, file này được cài vào package và nạp cho mọi node. Giá trị trong launch arguments có thể ghi đè `debug_enabled`, `start_hardware` và `start_servo`.

| Node | Nhóm tham số |
|---|---|
| `input_cache_node` | `ekf_timeout_sec=0.5`, `odom_timeout_sec=0.5`, `vision_timeout_sec=0.5`, `alt_timeout_sec=0.5`, `planner_timeout_sec=1.0`, `world_frame=world`, `debug_enabled=false` |
| `fsm_node` | `land_entry_height=0.5`, `align_threshold=0.3`, `land_entry_cycles=5`, `enter_follow_cycles=10`, `yaw_settle_rate=0.1`, `follow_lost_timeout=2.5`, `approach_lost_timeout=1.5`, `approach_descent_rate=0.3`, `land_descent_rate=0.4`, `land_requires_rearm=true`, `ignore_planner_timeout=false`, `debug_enabled=false` |
| `gimbal_node` | `kp=1.0`, `ki=0.0`, `kd=0.1`, `out_min_deg=-90`, `out_max_deg=90`, `max_slew_rate_deg_s=60`, `land_entry_height=0.5`, `debug_enabled=false` |
| `offboard_node` | `required_engage_cycles=10`, `mode_confirm_timeout_cycles=20`, `health_confirm_timeout_cycles=100`, `data_freshness_timeout_sec=1.0`, `max_horizontal_velocity=2.0`, `max_vertical_velocity=1.0`, `watchdog_timeout_sec=0.5`, `planner_timeout_sec=1.0`, `min_engage_altitude_m=2.0`, `yaw_search_rate=0.3`, `land_descent_rate=0.4`, `debug_enabled=false` |
| `px4_state_bridge` | `odometry_topic=/fmu/out/vehicle_odometry`, `land_detected_topic=/fmu/out/vehicle_land_detected`, `gimbal_angle_topic=/gimbal/target_angle_deg`, `world_frame=world`, `base_frame=base_link`, `gimbal_frame=gimbal_link`, `camera_frame=camera_optical_frame`, `gimbal_pivot_xyz=[0,0,0]`, `camera_in_gimbal_xyz=[0,0,0]`, `use_ground_contact=false`, `odom_timeout_sec=0.5` |
| `vision_interface_bridge` | `detected_topic=/hpad/detected`, `bbox_topic=/hpad/bbox`, `camera_info_topic=/camera_info`, `marker_topic=vision/marker`, `phase_topic=/mission/phase`, `image_width=640`, `image_height=480`, `fsm_state_timeout_sec=1.0` |
| `safety_monitor_node` | `battery_warning_frac=0.3`, `battery_critical_frac=0.15`, `offboard_hold_timeout=1.0`, `offboard_rtl_timeout=5.0`, `data_freshness_timeout_sec=1.0`, `force_land_latched=true`, `debug_enabled=false` |
| `rc_node` | `serial_device=/dev/ttyUSB0`, `baudrate=100000`, `land_channel=4`, `kill_channel=5`, `low_threshold=1200`, `high_threshold=1800`, `frame_timeout=0.5`, `debug_enabled=false` |
| `kill_switch_node` | `kill_channel=5`, `low_threshold=1200`, `high_threshold=1800`, `debounce_threshold=3`, `debug_enabled=false` |
| `servo_node` | `servo_function=33.0`, `command_timeout_sec=2.5`, `min_send_interval_sec=0.1`, `min_send_delta=0.01`, `keepalive_sec=1.5`, `pwm_min_us=1000`, `pwm_max_us=2000`, `angle_min_deg=0`, `angle_max_deg=180`, `home_angle_deg=90`, `input_timeout_sec=1.0`, `debug_enabled=false` |
| `xrce_bridge_node` | `serial_port=/dev/ttyAMA0`, `baudrate=921600`, `connection_timeout_sec=2.0`, `debug_enabled=false` |

Tham số của module vision và planner không nằm trong `System_Params.yaml` (danh sách đầy đủ xem guide của từng package):

| Node | Nguồn tham số | Tham số chính |
|---|---|---|
| `aruco_node` | Tham số node (`--ros-args -p`) | `marker_id=42`, `marker_size_m=0.15` (cạnh ngoài của phần đen), `dictionary=DICT_6X6_50`, `min_detection_distance_m=0.0`, `min_z_m=0.0`, `image_topic=/camera`, `camera_info_topic=/camera_info`, `camera_frame_id=camera_optical_frame`, `fallback_horizontal_fov_rad=1.52`, `require_camera_info=true` |
| `ekf_node` | Tham số node và launch arguments của `ekf.launch.py` | `process_accel_variance=[1.0, 1.0, 0.5]`, `gate_threshold=16.27`, `max_target_speed=2.5`, `max_target_vz=1.5`, `target_frame=world`, `child_frame=hpad`, `tf_timeout_s=0.03`, `output_rate_hz=50.0`, `position_topic=/hpad/position_camera`, `phase_topic=/mission/phase`, `state_topic=/ekf/target_state`, `mode_topic=/ekf/tracking_mode` |
| `ibvs_controller` | `ibvs/config/ibvs_params.yaml` | `K_pitch=0.8`, `K_yaw=0.5`, `focal_x=focal_y=466.0`, `u0=320.0`, `v0=240.0`, `pitch_rate_limit=1.5`, `pitch_ema_alpha=0.25`, `pitch_pixel_trim_gain=0.5`, `pitch_trim_limit_deg=20.0`, `yaw_rate_limit=0.5` |
| `apf_planner` | `apf_planner/config/apf_params.yaml` | `planner_type=iapf` (node mặc định `apf`), `obstacle_source=pointcloud`, `target_topic=/ekf/target_state`, `allowed_tracking_modes=[TRACKING, PREDICTING]`, `data_timeout_sec=0.5`, `rate_hz=30`, `d0=2.0`, `v_max=1.2`, `d_slow=1.5`, `k_att=10`, `k_rep=250`, `k_rep_approach=125`, `goal_threshold=0.20`, `follow_distance=3.5`, `hold_follow_altitude=true`, `target_altitude=3.0`, `k_z=0.6`, `vz_max=0.5`, `max_cloud_points=8`, `iapf_f_enter=0.10`, `iapf_f_exit=0.30`, `iapf_n_tangent=12`, `iapf_n_pred=3` |
| `planner_merge` | `apf_planner/config/apf_params.yaml` | `rate_hz=20`, `yaw_source=ibvs_apf`, `upstream_timeout_sec=0.3`, `stale_hover_sec=0.5`, `yaw_timeout_sec=0.3`, `phase_timeout_sec=1.0`, `output_topic=planner/velocity_setpoint` |
| `apf_pointcloud_generator` | `apf_planner/config/apf_params.yaml` | `topic=/map_generator/global_cloud`, `frame_id=world`, `rate_hz=1.0`, `num_obs=35`, `map_size=25.0`, `height=4.0`, `resolution=0.15`, `clear_radius=2.0`, `seed=-1`, `publish_static_tf=true` |
| `takeoff` | Tham số dòng lệnh | `--alt=3.0` (phải khớp `MIS_TAKEOFF_ALT` của PX4), `--climb-timeout=20.0` |

Launch arguments:

| Launch file | Arguments |
|---|---|
| `ekf.launch.py` | `use_sim_time=false`, `publish_odom_tf=true`, `publish_camera_tf=true`, `odom_topic=/odom`, `world_frame=world`, `base_frame=base_link`, `camera_frame=camera_optical_frame`, `cam_x/y/z`, `cam_roll/pitch/yaw` |
| `ibvs.launch.py` | `params_file=config/ibvs_params.yaml`, `use_sim_time=false` |
| `apf_planner.launch.py` | `params_file`, `generator=false`, `planner_type=iapf` |
| `vdt_system.launch.py` (`vdt_bringup`) | `start_hardware=true` (bật `rc_node` và `kill_switch_node`; chỉ đặt `false` trong bench/SITL có cơ chế safety khác), `start_servo=false` (bật `servo_node`), `debug=false` (ghi đè `debug_enabled`) |

## Sơ đồ launch

```text
xrce_bridge_node
        |
        v
 px4_state_bridge
        |
        v
 vision_interface_bridge
        |
        v
  input_cache_node
        |
   +----+-----------------+
   |                      |
   v                      v
 rc_node          offboard_safety_monitor
   |                      |
   v                      |
kill_switch_node          |
   |                      |
   +----------+-----------+
              |
              v
          fsm_node
              |
              v
         gimbal_node
              |
              v
   servo_node (optional)

input_cache_node + offboard_safety_monitor + fsm_node
              |
              v
         offboard_node
              |
              v
              PX4

offboard_safety_monitor ------------> PX4
kill_switch_node --------------------> PX4
servo_node --------------------------> PX4
```

Launch sequence theo thời gian là `XRCE -> PX4 state bridge -> vision bridge -> input cache -> RC/kill -> safety -> FSM/gimbal -> Offboard -> servo`. Đây là thứ tự khởi tạo process; readiness thật vẫn do các node kiểm tra freshness, timeout, health và mode PX4.

Module vision và `apf_planner` (`planner_node`, `planner_merge_node`) được khởi động riêng, xem [Chạy hệ thống](#chạy-hệ-thống).


`takeoff` cũng không nằm trong launch: chạy tay sau khi hệ thống đã lên, xem [Chạy hệ thống](#chạy-hệ-thống).

## Luồng hoạt động

1. `xrce_bridge_manager` kết nối ROS 2 với PX4 qua Micro XRCE-DDS.
2. Cất cánh bằng `takeoff` (arm và PX4 takeoff, chạy tay trên Pi). `offboard_manager` chỉ engage Offboard khi UAV đã armed và đạt `min_engage_altitude_m`.
3. `px4_state_bridge` đổi trạng thái PX4 sang `/odom`, chuỗi TF tới camera và `alt_estimator/state`.
4. `aruco_node` phát hiện H-Pad trên ảnh IR1 và publish `/hpad/detected`, `/hpad/bbox`, `/hpad/position_camera`. `ekf_node` đổi vị trí sang `world` bằng TF tại stamp ảnh và ước lượng trạng thái H-Pad (`/ekf/target_state`, `/ekf/tracking_mode`).
5. `vision_interface_bridge` tạo `vision/marker` từ kết quả phát hiện và `/mission/phase` từ `fsm/state`.
6. `ibvs_controller` dùng `/mission/phase`: ở SEARCH giữ `yaw_cmd` theo yaw thực (việc quét yaw do `offboard_manager` điều khiển), ở FOLLOW/APPROACH hiệu chỉnh pitch gimbal (`/ibvs/pitch_trim_deg`) và tính `/ibvs/yaw_cmd`.
7. `planner_node` (khi phase là FOLLOW/APPROACH và EKF ở `TRACKING`/`PREDICTING`) tính vận tốc tránh vật cản `/apf/velocity_cmd` và `/apf/yaw_cmd` từ `/odom`, `/ekf/target_state` và vật cản.
8. `planner_merge_node` ghép vận tốc APF với yaw (ưu tiên IBVS) thành `planner/velocity_setpoint` và publish đều ở mọi phase.
9. `input_state_cache` tập hợp dữ liệu, kiểm tra độ tươi và tính hợp lệ, tính hình học tương đối giữa UAV và H-Pad rồi phát `InputSnapshot` đồng bộ.
10. FSM nhận `InputSnapshot`, cập nhật bộ đếm và phát lệnh cho planner, gimbal và Offboard manager.
11. `gimbal_control` tính góc nền theo state, cộng hiệu chỉnh từ IBVS rồi phát `gimbal/target_angle_deg`. `servo_control` đổi góc này thành lệnh `ACTUATOR_TEST` gửi PX4 qua `/fmu/in/vehicle_command`.
12. `offboard_manager` đổi `planner/velocity_setpoint` (ENU) sang NED và gửi PX4 qua `/fmu/in/*`; planner stale thì UAV hover.
13. `offboard_safety_monitor` và `kill_switch` có thể inhibit Offboard, yêu cầu HOLD/RTL/force-land hoặc force-disarm độc lập với perception.

## Cài đặt và build

### Phụ thuộc

- ROS 2 với `ament_cmake` và `ament_python`.
- `rclcpp`, `rclpy`, `std_msgs`, `nav_msgs`, `geometry_msgs`, `sensor_msgs`, `vision_msgs`, `visualization_msgs`, `tf2_ros`, `px4_msgs`.
- `rosidl_default_generators`, `rosidl_default_runtime`.
- `vdt_msgs` (build trước các package còn lại).
- `MicroXRCEAgent`.
- `aruco_detector`: `numpy`, `opencv-contrib-python` (cần `cv2.aruco`; từ OpenCV 4.7 dùng `ArucoDetector`), `ros-<distro>-vision-msgs`.
- `ekf_adapter`: `numpy`, `ros-<distro>-tf2-ros`, `nav-msgs`, `geometry-msgs`, `launch-ros`.
- `ibvs`: `ros-<distro>-vision-msgs`.
- `apf_planner`: `python3-numpy`, `tf2_ros_py`, `visualization_msgs`, `vdt_msgs`, `launch_ros`.
- `servo_control`: `px4_msgs`.
- `takeoff`: `rclpy`, `std_msgs`, `px4_msgs`.

Không cài song song `opencv-python` và `opencv-contrib-python` trong cùng môi trường. Nếu hệ thống đã có `python3-opencv` qua apt và có `cv2.aruco` thì không cần cài thêm.

### Build ROS 2

Thực hiện trên Linux có ROS 2 đã cài:

```bash
cd ros2_ws
source /opt/ros/<ros_distro>/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --packages-select vdt_msgs
colcon build
source install/setup.bash
```

Launch file `vdt_bringup` nạp [System_Params.yaml](System_Params.yaml) và sắp xếp thứ tự khởi động, còn readiness thực tế được kiểm tra trong từng node.

### Build module vision và planner

Cài phụ thuộc Python của vision:

```bash
python3 -m pip install numpy opencv-contrib-python
sudo apt install ros-<ros_distro>-vision-msgs
```

Build các package vision và planner (`apf_planner` cần `vdt_msgs` đã build):

```bash
cd ros2_ws
colcon build --packages-select aruco_detector ekf_adapter ibvs apf_planner
source install/setup.bash
```

### Build diagnostics

```bash
cd landing_diagnostics
python3 -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt
```

`requirements.txt` gồm `pandas` và `numpy`. Logger còn cần môi trường ROS 2, `px4_msgs` và `vdt_msgs`.

## Chạy hệ thống

Khuyến nghị dùng launch orchestration:

```bash
source /opt/ros/<ros_distro>/setup.bash
source ros2_ws/install/setup.bash
ros2 launch vdt_bringup vdt_system.launch.py
```

Các tùy chọn launch:

```bash
ros2 launch vdt_bringup vdt_system.launch.py start_hardware:=false
ros2 launch vdt_bringup vdt_system.launch.py start_servo:=true
ros2 launch vdt_bringup vdt_system.launch.py debug:=true
```

Cất cánh (chạy tay ở terminal khác sau khi hệ thống đã lên, không đưa vào launch):

```bash
ros2 run takeoff takeoff --alt 3.0
```

`--alt` phải khớp tham số PX4 `MIS_TAKEOFF_ALT` (đặt một lần trước khi bay). Xem [Takeoff_Guide.md](ros2_ws/src/takeoff/Takeoff_Guide.md).

Chạy riêng `input_state_cache` (cần remap để nhận trạng thái H-Pad từ EKF):

```bash
ros2 run input_state_cache input_cache_node --ros-args \
  -r hpad/state_filtered:=/ekf/target_state
```

Module vision được khởi động riêng, cần driver camera đang chạy. Phát hiện ArUco (`marker_size_m` phải đúng cạnh ngoài của phần đen trên H-Pad thật; `image_topic` và `camera_info_topic` phải cùng camera IR1):

```bash
ros2 run aruco_detector aruco_node --ros-args \
  -p marker_id:=42 \
  -p marker_size_m:=0.15 \
  -p image_topic:=/camera/infra1/image_rect_raw \
  -p camera_info_topic:=/camera/infra1/camera_info
```

Che vùng marker trên depth:

```bash
ros2 run aruco_detector depth_to_image_node
```

EKF dùng TF của `px4_state_bridge` nên tắt hai nguồn TF của launch (nếu không sẽ trùng/xung đột TF). Tham số `state_topic` của `ekf_node` đặt là `/ekf/target_state`:

```bash
ros2 launch ekf_adapter ekf.launch.py \
  publish_odom_tf:=false publish_camera_tf:=false
```

IBVS đọc `/ekf/target_state` trực tiếp, không cần remap:

```bash
ros2 launch ibvs ibvs.launch.py
```

Planner tránh vật cản và node ghép (`target_topic` trong `apf_params.yaml` đặt là `/ekf/target_state`):

```bash
ros2 launch apf_planner apf_planner.launch.py planner_type:=iapf
```

Thử với point cloud vật cản giả lập trong RViz2:

```bash
ros2 launch apf_planner apf_planner.launch.py generator:=true planner_type:=iapf
```