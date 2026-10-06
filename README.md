# VDT Project

Workspace prototype cho hệ thống UAV PX4 + ROS 2 thực hiện phát hiện, bám và hạ cánh lên H-Pad bằng vision. Hệ thống gồm bộ máy trạng thái bay, điều khiển Offboard, điều khiển gimbal/servo, nhận RC, các lớp giám sát an toàn, các node cầu nối giữa PX4 và module vision, các package vision (`aruco_detector`, `ekf_adapter`, `ibvs`), planner tránh vật cản `apf_planner` (APF / I-APF) kèm node ghép lệnh cho Offboard, và công cụ phân tích log sau chuyến bay.

> **Trạng thái:** skeleton tích hợp/prototype, cần kiểm thử trên PX4 SITL trước khi dùng cho phương tiện thật.

## Mục lục

- [Kiến trúc tổng quan](#kiến-trúc-tổng-quan)
- [Cấu trúc repository](#cấu-trúc-repository)
- [Quy ước frame và đơn vị](#quy-ước-frame-và-đơn-vị)
- [ROS 2 packages](#ros-2-packages)
- [Module vision](#module-vision)
- [Messages](#messages)
- [Topics chính](#topics-chính)
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
        |        /hpad/detected, /hpad/bbox, /hpad/state_filtered,
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

ibvs_controller ──► /ibvs/pitch_trim_deg ──► gimbal_control (cộng vào góc mục tiêu)
```

## Cấu trúc repository

| Đường dẫn | Nội dung |
|---|---|
| `ros2_ws/src/` | Mười sáu ROS 2 packages của hệ thống bay, module vision, planner, cầu nối vision và bringup |
| `ros2_ws/src/vdt_bringup/` | Launch orchestration và startup ordering |
| `ros2_ws/src/px4_state_bridge/` | Cầu nối trạng thái PX4 sang `/odom`, TF và `alt_estimator/state` |
| `ros2_ws/src/vision_interface_bridge/` | Cầu nối topic vision sang `vision/marker` và `/mission/phase` |
| `ros2_ws/src/aruco_detector/` | Phát hiện ArUco H-Pad, pose trong frame camera, mask depth |
| `ros2_ws/src/ekf_adapter/` | EKF ước lượng trạng thái H-Pad trong frame `world` |
| `ros2_ws/src/ibvs/` | Visual servoing: hiệu chỉnh pitch gimbal và lệnh yaw |
| `ros2_ws/src/apf_planner/` | Planner APF / I-APF tránh vật cản, node ghép `planner/velocity_setpoint`, bộ sinh point cloud thử nghiệm |
| `landing_diagnostics/` | Ghi log, tính metric và đề xuất tuning từ CSV |
| `PX4_Control/` | Hướng dẫn và template patch cho PX4 |
| `Embedded_Plan.md` | Kế hoạch thiết kế embedded tổng thể |
| `PX4_Architecture.md` | Kiến trúc giao tiếp PX4/ROS 2 |

## Quy ước frame và đơn vị

| Hạng mục | Quy ước |
|---|---|
| Frame gốc `world` | ENU (x Đông, y Bắc, z lên), trùng local frame của PX4 sau khi đổi sang ENU |
| Thân UAV `base_link` | FLU (x trước, y trái, z lên) |
| Camera `camera_optical_frame` | x phải, y xuống, z phía trước |
| `/odom`, `/hpad/state_filtered` | Frame `world`, vị trí tuyệt đối |
| `/hpad/pose`, `/hpad/position_camera` | Frame `camera_optical_frame`; quaternion thứ tự x, y, z, w của ROS |
| `/hpad/bbox`, `/hpad/mask_polygon` | Pixel của ảnh IR1 |
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

### `fsm_state_machine` (C++)

Điều khiển chuỗi trạng thái:

```text
SEARCH -> FOLLOW -> APPROACH -> LAND -> COMPLETE
```

- Timer 10 Hz.
- Nhận `InputSnapshot` (từ `input_cache/snapshot`), RC (`rc/fsm_input`), yêu cầu `safety/force_land` và cờ ngắt `system/killed` (QoS transient-local). Khi `killed = true`, `update()` thoát ngay: FSM không cập nhật bộ đếm, không chuyển state và không publish topic nào (kể cả `fsm/state`).
- Watchdog: quá 200 ms không nhận snapshot (hoặc chưa từng nhận) thì `valid = false`, `marker_detected = false` và coi như `planner_timeout = true`.
- `sensor_validate`: `align_error` hoặc `altitude` là NaN thì `valid = false`. `geometry_valid = valid && isfinite(delta_h) && isfinite(d_horiz)`. FSM node hiện truyền cờ timeout EKF/vision mặc định (đều `false`) nên chưa phân biệt mode EKF.
- Bộ đếm:
  - `marker_stable_count` chỉ tăng khi thấy marker, `geometry_valid` và `abs(yaw_rate) < yaw_settle_rate`; sai một chu kỳ thì về 0.
  - `marker_lost_time` tăng khi không thấy marker và không bị reset khi đổi state.
  - `land_ok_count` chỉ đếm ở APPROACH, tăng khi `geometry_valid && align_error < align_threshold && delta_h < land_entry_height`; sai một chu kỳ thì về 0.
- Thứ tự ưu tiên khi chuyển state: `safety/force_land`, rồi `planner_timeout`, rồi bảng chuyển state. Điều kiện RC dùng `land_switch` hiệu lực: `rc.land_switch || safety/force_land`.
- `SEARCH -> FOLLOW` khi `marker_stable_count >= enter_follow_cycles`.
- `FOLLOW -> SEARCH` khi mất marker quá `follow_lost_timeout` (2.5 s).
- `FOLLOW -> APPROACH` khi land switch bật, marker còn nhìn thấy và không bị `land_inhibit`.
- `APPROACH -> FOLLOW` khi tắt land switch hoặc mất marker quá `approach_lost_timeout` (1.5 s).
- `APPROACH -> LAND` khi `land_ok_count >= land_entry_cycles`. Khi không có mục tiêu hợp lệ, `delta_h`/`align_error` là NaN nên điều kiện không thể đúng.
- `SEARCH/FOLLOW/APPROACH -> LAND` khi `safety/force_land = true` (không áp dụng khi đã ở LAND hoặc COMPLETE).
- `FOLLOW/APPROACH -> SEARCH` khi `planner_timeout = true`, gồm cả mất snapshot quá 200 ms (bỏ qua nếu `ignore_planner_timeout = true`).
- `LAND -> COMPLETE` khi có touchdown flag. COMPLETE là trạng thái cuối.
- `land_inhibit`: đặt khi rời APPROACH (trừ sang LAND) mà công tắc hạ cánh vẫn bật, xóa khi công tắc tắt. Chỉ có tác dụng khi `land_requires_rearm = true`, nhằm tránh công tắc "sống lại" sau khi hủy APPROACH.
- Đầu ra theo state: `planner/mode` và `gimbal/state_request` (giá trị `uint8` của enum `State`), `planner/apf_gain` (FOLLOW 1.0, APPROACH 0.5, LAND 0.0, SEARCH không publish), `gimbal/align_error_cmd` (chỉ APPROACH), `cmd/vertical_descent_rate` (SEARCH/FOLLOW 0; APPROACH bằng `approach_descent_rate` nếu `marker_detected && geometry_valid && delta_h >= land_entry_height`, ngược lại 0; LAND bằng `land_descent_rate`), `cmd/disarm_request = true` ở COMPLETE, và `fsm/state`.
- FSM không publish `cmd/yaw_rate`. Quét yaw ở SEARCH và phanh yaw khi thấy marker do `ibvs_controller` đảm nhiệm (khi `planner/mode = SEARCH`); FSM chỉ kiểm tra yaw đã ổn định bằng `yaw_settle_rate`.
- `fsm/state` còn được `vision_interface_bridge` đọc để tạo `/mission/phase` cho module vision.

Tham số chính: `land_entry_height=0.5`, `align_threshold=0.3`, `land_entry_cycles=5`, `enter_follow_cycles=10`, `yaw_settle_rate=0.1`, `follow_lost_timeout=2.5`, `approach_lost_timeout=1.5`, `approach_descent_rate=0.3`, `land_descent_rate=0.4`, `land_requires_rearm=true`, `ignore_planner_timeout=false`. Xem [FSM_Guide.md](ros2_ws/src/fsm_state_machine/FSM_Guide.md).

### `gimbal_control` (C++)

Tính góc mục tiêu theo state và hình học tương đối giữa UAV với H-Pad. Góc đi qua PID phần mềm và slew-rate limiter trước khi phát trên `gimbal/target_angle_deg`.

- Timer 20 Hz.
- SEARCH: `0` độ.
- FOLLOW/APPROACH: `-atan2(delta_h, d_horiz)` đổi sang độ.
- LAND: nội suy từ `-60` đến `-90` độ.
- Mặc định PID: `kp=1.0`, `ki=0.0`, `kd=0.1`.
- Điều khiển open-loop ở cấp servo; `current_angle_` là góc lệnh trước đó, không phải feedback vật lý. PID có anti-windup, finite-value checks và bảo vệ `land_entry_height`/slew rate khỏi dữ liệu không hợp lệ.
- `gimbal/target_angle_deg` cũng được `px4_state_bridge` đọc để dựng TF của camera, nên góc lệnh đi vào vị trí H-Pad mà EKF tính.
- Giao ước với `ibvs`: `gimbal_control` quyết định góc pitch nền theo phase và hình học 3D (kể cả giới hạn góc theo phase), còn IBVS chỉ gửi lượng hiệu chỉnh nhỏ theo pixel qua `/ibvs/pitch_trim_deg`. Phía gimbal đăng ký topic này, chỉ cộng vào góc mục tiêu ở FOLLOW và APPROACH, coi như 0 nếu giá trị không hữu hạn hoặc cũ hơn khoảng 0.3 s, và kẹp tổng góc trong `[out_min_deg, out_max_deg]` trước khi đưa vào PID và slew limiter. Xem [IBVS_Guide.md](ros2_ws/src/ibvs/IBVS_Guide.md).

### `servo_control` (Python)

Chuyển `gimbal/target_angle_deg` thành lệnh servo:

```text
gimbal/target_angle_deg
  -> cộng home_angle_deg
  -> clamp [angle_min_deg, angle_max_deg]
  -> AngularServo.angle
```

Mặc định góc servo `0..180`, home `90` độ, timeout input `1.0 s`. Khi mất lệnh gimbal quá timeout, servo trở về home; giá trị NaN/vô hạn bị bỏ qua. Xem [Servo_Guide.md](ros2_ws/src/servo_control/Servo_Guide.md).

Dự kiến chuyển sang điều khiển servo bằng PWM của PX4; ranh giới giao tiếp cần giữ nguyên là `gimbal/target_angle_deg` (độ).

### `input_state_cache` (C++)

Bộ tập hợp trạng thái trung tâm: lưu payload của các nguồn, kiểm tra độ tươi và tính hợp lệ, tính hình học tương đối giữa UAV và H-Pad rồi phát `InputSnapshot` đồng bộ và `TimeoutFlags`.

- Timer 20 Hz.
- Nguồn đầu vào: `hpad/state_filtered` (Odometry của H-Pad do `ekf_node` phát), `ekf/tracking_mode`, `odom` (Odometry của UAV), `vision/marker`, `alt_estimator/state`, `planner/velocity_setpoint` (chỉ theo dõi thời điểm nhận).
- Timeout mặc định: EKF `0.5 s`, odom `0.5 s`, vision `0.5 s`, altitude `0.5 s`, planner `1.0 s`.
- `hpad/state_filtered` và `odom` có `frame_id` khác `world_frame` bị loại bỏ.
- EKF chỉ hợp lệ khi `ekf/tracking_mode` thuộc `TRACKING`, `PREDICTING`, `PREDICTING_DEGRADED`; `EXPIRED` hoặc chuỗi lạ bị coi là không hợp lệ.
- Hình học tương đối (chỉ khi EKF và odom cùng hợp lệ): `d_horiz = hypot(x_ekf - x_odom, y_ekf - y_odom)`, `delta_h = z_odom - z_ekf`, `align_error = d_horiz`. Ngược lại là `NaN`.
- `valid = odom_valid && alt_valid`, chỉ mô tả trạng thái UAV, không phụ thuộc việc có mục tiêu hay không.
- `marker_detected` yêu cầu `vision/marker` còn tươi; `touchdown` yêu cầu `alt_estimator/state` còn tươi.
- `InputSnapshot.yaw_rate` (rad/s, quanh trục yaw) phải được điền. Nếu là NaN thì FSM không bao giờ vào FOLLOW.
- Phải publish `input_cache/snapshot` đều đặn: khoảng cách giữa hai bản tin quá 200 ms thì FSM coi là mất snapshot.
- `planner/velocity_setpoint` phải được publish đều đặn ở mọi phase (xem `planner_merge_node`), nếu không `planner_timeout` bật ngay khi vào FOLLOW.

Xem [ISC_Guide.md](ros2_ws/src/input_state_cache/ISC_Guide.md).

### `offboard_manager` (C++)

Chuyển state và planner output thành lệnh PX4:

- Timer 20 Hz.
- Phát `OffboardControlMode` (position=false, velocity=true), `TrajectorySetpoint` và `VehicleCommand`.
- `PlannerOutput` ở frame ENU; node đổi sang NED trước khi gửi PX4 (xem [Quy ước frame và đơn vị](#quy-ước-frame-và-đơn-vị)). `yaw = NaN` giữ nguyên yaw hiện tại.
- Chỉ bắt đầu chuỗi engage khi UAV đã bay: PX4 `ARMED` và độ cao `-z >= min_engage_altitude_m` (mặc định `2.0 m`). Cất cánh được thực hiện bằng PX4 takeoff.
- Gửi 10 chu kỳ setpoint trước khi yêu cầu chuyển Offboard.
- Chờ `VehicleStatus.nav_state == OFFBOARD` từ message còn fresh trước khi tiếp tục.
- Chỉ arm khi `VehicleStatus` và `VehicleLocalPosition` đã nhận, còn fresh, EKF có `xy_valid/z_valid` và PX4 không ở failsafe.
- SEARCH dừng vị trí và quay theo `yaw_search_rate`.
- FOLLOW/APPROACH dùng velocity từ `planner/velocity_setpoint`; khi planner stale (từ `TimeoutFlags` hoặc không nhận planner quá `planner_timeout_sec`), UAV hover và giữ yaw.
- LAND dừng vận tốc ngang, dùng `land_descent_rate` (dương là xuống) và giữ yaw hiện tại.
- Planner stale chỉ ảnh hưởng FOLLOW/APPROACH, không chặn chuỗi engage, SEARCH hoặc LAND.

Tham số chính: `required_engage_cycles=10`, `mode_confirm_timeout_cycles=20`, `health_confirm_timeout_cycles=100`, `data_freshness_timeout_sec=1.0`, `max_horizontal_velocity=2.0`, `max_vertical_velocity=1.0`, `watchdog_timeout_sec=0.5`, `planner_timeout_sec=1.0`, `min_engage_altitude_m=2.0`, `yaw_search_rate=0.3`, `land_descent_rate=0.4`. Vận tốc planner non-finite bị thay bằng hover (vận tốc 0, giữ yaw), vận tốc hợp lệ bị clamp theo các giới hạn này.

Xem [Offboard_Guide.md](ros2_ws/src/offboard_manager/Offboard_Guide.md).

### `px4_state_bridge` (C++)

Cầu nối trạng thái PX4 sang dạng mà module vision và `input_state_cache` cần.

- Đọc `/fmu/out/vehicle_odometry` (NED/FRD), `/fmu/out/vehicle_land_detected` và `gimbal/target_angle_deg`.
- Publish `/odom` (`nav_msgs/Odometry`, frame `world`, ENU, orientation FLU, vận tốc tuyến tính trong frame `world`).
- Publish chuỗi TF `world -> base_link -> gimbal_link -> camera_optical_frame`; góc gimbal là góc lệnh. Vị trí lắp (`gimbal_pivot_xyz`, `camera_in_gimbal_xyz`) là tham số. `ekf_node` dùng chuỗi TF này để đổi vị trí marker sang `world`.
- Publish `alt_estimator/state` (`AltEstimate`) ở 20 Hz khi odometry còn tươi: `altitude` là độ cao ENU so với gốc local frame của PX4, `touchdown_flag` lấy từ `landed` (tùy chọn `ground_contact`).
- Bỏ bản tin có `pose_frame` không phải NED, giá trị `NaN`/`Inf` hoặc quaternion suy biến.

Xem [PX4_Bridge_Guide.md](ros2_ws/src/px4_state_bridge/PX4_Bridge_Guide.md).

### `vision_interface_bridge` (C++)

Cầu nối topic của module vision sang giao diện của embedded.

- `vision/marker` (`VisionMarker`): publish `marker_visible = true` kèm `pixel_align_error` chuẩn hóa khi nhận `/hpad/bbox`; publish `marker_visible = false` khi nhận `/hpad/detected` với `false`.
- `/mission/phase` (`String`): đổi `fsm/state` thành `SEARCH`, `FOLLOW`, `APPROACH`, `LAND`; COMPLETE, giá trị lạ hoặc `fsm/state` cũ hơn `fsm_state_timeout_sec` là `IDLE`. Publish ở 10 Hz.
- Kích thước ảnh lấy từ `camera_info`, dự phòng `image_width`/`image_height`.

Xem [Vision_Bridge_Guide.md](ros2_ws/src/vision_interface_bridge/Vision_Bridge_Guide.md).

### `aruco_detector` (Python)

Phát hiện ArUco bằng `cv2.aruco` trên ảnh IR1 (mono8), giải PnP bằng `SOLVEPNP_IPPE_SQUARE`. Gồm hai node: `aruco_node` và `depth_to_image_node`.

- `aruco_node` nhận ảnh và `CameraInfo` (QoS `BEST_EFFORT`, depth 3). Chưa có `CameraInfo` thì dùng camera matrix dự phòng theo `fallback_horizontal_fov_rad`; nếu `require_camera_info = true` thì chặn publish pose cho đến khi nhận `CameraInfo`.
- Chọn marker đúng `marker_id`, loại detection có khoảng cách nhỏ hơn `min_detection_distance_m` hoặc `z` nhỏ hơn `min_z_m`.
- Publish `/hpad/detected` ở mọi frame; khi thấy marker publish thêm `/hpad/pose`, `/hpad/position_camera`, `/hpad/bbox`, `/hpad/mask_polygon` và `/hpad/annotated`.
- Pose nằm trong `camera_frame_id` (mặc định `camera_optical_frame`). Node không tự chuyển pose sang body hay local NED.
- `depth_to_image_node` giữ polygon mới nhất từ `/hpad/mask_polygon`; khi nhận depth, nếu stamp lệch không quá 0.2 s thì vùng polygon được gán NaN trước khi chuẩn hóa về mono8, publish `/depth_camera/image_mono`. Polygon tính theo pixel IR1 nên chỉ khớp khi depth được align với IR1 và cùng độ phân giải.
- Tham số chính: `marker_id=42`, `marker_size_m=0.15` (cạnh ngoài của phần đen), `dictionary=DICT_6X6_50`, `image_topic=/camera`, `camera_info_topic=/camera_info`, `camera_frame_id=camera_optical_frame`, `require_camera_info=true`, `mask_margin_percent=0.15`.

Xem [Aruco_Guide.md](ros2_ws/src/aruco_detector/Aruco_Guide.md).

### `ekf_adapter` (Python)

Ước lượng trạng thái H-Pad trong frame `world` từ `/hpad/position_camera`. Gồm `ekf_node`, `odom_tf_node` và `ekf.launch.py`.

- Trạng thái lọc `[x, y, z, vx, vy, vz]` trong `world`, mô hình vận tốc không đổi, nhiễu gia tốc trắng.
- Mỗi measurement được đổi sang `world` bằng TF tra đúng tại stamp ảnh; thiếu stamp/`frame_id` hoặc lỗi TF thì bỏ measurement và tăng `tf_rejects`. Ma trận nhiễu R tăng theo độ sâu, cộng sai số attitude và sai số vị trí xe.
- `TargetTracker` gồm gate động theo khoảng cách ngang, NIS gate trong lõi lọc, tái bắt track sau `reacquire_frames` frame nhất quán, giảm vận tốc khi mất measurement lâu.
- Timer 50 Hz ngoại suy một bản sao filter tới thời điểm hiện tại và publish liên tục `/hpad/state_filtered` (Odometry, frame `world`, có covariance vị trí và vận tốc) cùng `/ekf/tracking_mode`, kể cả khi mode là `EXPIRED`; consumer phải lọc theo mode, không dựa vào độ tươi của timestamp. Orientation không được ước lượng; vận tốc nằm trong hệ `world`, không phải `child_frame_id`.
- Nhận `/mission/phase`: `APPROACH` hoặc `FOLLOW`, giá trị khác coi như `FOLLOW`.
- Mode tracking do `classify_tracking_mode` (`tracking_policy.py`) quyết định, tên mode khớp với danh sách mà `input_state_cache` chấp nhận. `age` là tuổi của measurement hợp lệ gần nhất; `limit` là 1.0 s ở APPROACH và 2.0 s ở FOLLOW:

| Mode | Điều kiện |
|---|---|
| `TRACKING` | Đang detect và `age <= 0.25 s` |
| `PREDICTING` | `age <= 0.5 * limit` (APPROACH 0.5 s, FOLLOW 1.0 s) |
| `PREDICTING_DEGRADED` | `age <= limit` (APPROACH 1.0 s, FOLLOW 2.0 s) |
| `EXPIRED` | Quá `limit` hoặc chưa từng có measurement |

- `odom_tf_node` publish TF `world -> base_link` từ `/odom`, và `ekf.launch.py` có thể publish static TF `base_link -> camera_optical_frame` từ tham số `cam_*`. Trong hệ thống này `px4_state_bridge` đã publish toàn bộ chuỗi TF (gồm cả góc gimbal), nên chạy launch với `publish_odom_tf:=false publish_camera_tf:=false`; khi đó các tham số `cam_*` không có tác dụng.
- Tham số nhiễu measurement (`NoiseConfig`), gate và tái bắt (`TrackerConfig`) được khai báo từ dataclass, xem guide.

Xem [EKF_Guide.md](ros2_ws/src/ekf_adapter/EKF_Guide.md).

### `ibvs` (Python)

`ibvs_controller` điều khiển theo pixel, timer 30 Hz và tính theo sự kiện khi có `/hpad/bbox` mới. Không ra góc pitch tuyệt đối, chỉ gửi hiệu chỉnh nhỏ để bù sai số EKF và sai lệch góc servo.

| Phase (`/mission/phase`) | Hiệu chỉnh pitch | Yaw |
|---|---|---|
| IDLE (mặc định khi chưa nhận phase) | 0 | `yaw_cmd` bằng yaw thực, không điều khiển |
| SEARCH | 0 | Quét với `search_yaw_rate`; thấy marker thì giữ yaw cố định, quá 0.75 s không thấy thì quét tiếp từ yaw thực |
| FOLLOW, APPROACH | Theo pixel, kẹp ±`pitch_trim_limit_deg` | Bám pixel kèm feedforward tiếp tuyến khi có EKF |
| LAND | 0 | Chỉnh nhẹ theo pixel, chỉ khi có bbox mới; mất marker thì `yaw_cmd` giữ giá trị cuối |

- Đầu ra: `/ibvs/pitch_trim_deg` (`Float32`, độ) và `/ibvs/yaw_cmd` (`Float64`, rad, heading ENU tuyệt đối). `/ibvs/yaw_cmd` được `planner_merge_node` tiêu thụ.
- Đầu vào: `/hpad/bbox`, `/hpad/detected`, `/camera_info` (ghi đè `focal_x/focal_y/u0/v0` khi nhận `K[0] > 0`), `/odom`, `/ekf/target_state`, `/ekf/tracking_mode`, `/mission/phase`, `/gimbal/target_angle_deg` (chưa nhận thì giả định -30 độ). Thiếu `/ekf/target_state` và `/ekf/tracking_mode` thì chạy chế độ chỉ dùng pixel.
- Tên topic cố định trong code, đổi bằng remap (xem [Chạy hệ thống](#chạy-hệ-thống)).
- Tham số trong `config/ibvs_params.yaml`: `K_pitch=0.8`, `K_yaw=0.5`, `focal_x=focal_y=466.0`, `u0=320.0`, `v0=240.0`, `search_yaw_rate=0.2`, `pitch_rate_limit=1.5`, `pitch_ema_alpha=0.25`, `pitch_pixel_trim_gain=0.5`, `pitch_trim_limit_deg=20.0`, `yaw_rate_limit=0.5`.

Xem [IBVS_Guide.md](ros2_ws/src/ibvs/IBVS_Guide.md).

### `apf_planner` (Python)

Planner tránh vật cản cho FOLLOW/APPROACH, gồm ba node và hai lõi thuật toán thuần Python (không phụ thuộc ROS): `apf_core.py` (APF 2D) và `iapf_core.py` (Improved APF 3D).

**`planner_node`** (node `apf_planner`, timer 30 Hz)

- Đầu vào: `/odom` (BEST_EFFORT), `target_topic` (mặc định `/hpad/state_filtered`), `/ekf/tracking_mode`, `/mission/phase`, `/map_generator/global_cloud` (khi dùng point cloud).
- Đầu ra: `/apf/velocity_cmd` (`Twist`, ENU), `/apf/yaw_cmd` (`Float64`, heading ENU), `/apf/force_markers` (`MarkerArray` cho RViz: lực hút, lực đẩy, tổng, tiếp tuyến, UAV, goal, vật cản).
- Chỉ tính lệnh khi đồng thời: phase là `FOLLOW` hoặc `APPROACH`; `tracking_mode` thuộc `allowed_tracking_modes` (mặc định `TRACKING`, `PREDICTING`); odom và target đều tươi (`data_timeout_sec = 0.5 s`). Ngược lại publish `Twist` bằng 0 và không publish yaw. Message sai `world_frame` hoặc có giá trị non-finite bị loại; kết quả non-finite được thay bằng vận tốc 0.
- Goal:
  - APPROACH: vị trí H-Pad (3D).
  - FOLLOW: điểm stand-off (`make_follow_goal`) cách H-Pad `follow_distance` (khoảng cách 3D), giữ độ cao `target_altitude` nếu `hold_follow_altitude`. Vị trí H-Pad được cộng lead theo vận tốc target (`target_lead_time`, kẹp `max_target_lead`, chỉ khi tốc độ >= 0.18 m/s); vận tốc target lọc EMA (`target_velocity_alpha`) kèm deadband.
  - FOLLOW còn cộng feedforward vận tốc ngang của target khi target di chuyển >= 0.20 m/s và chưa tới goal, rồi kẹp theo `v_max`.
- Lõi `planner_type`:
  - `apf`: lực hút `2·k_att·(goal - pos)` trên mặt phẳng ngang; lực đẩy kèm thành phần tiếp tuyến 2D trong bán kính `d0`; giảm tốc tuyến tính từ `d_slow` đến `goal_threshold`; `vz` điều khiển P theo độ cao goal (`k_z`, deadband 8 cm, kẹp `vz_max`).
  - `iapf`: lực đẩy 3D cho GNRON (`F_rep1 + F_rep2`, trọng số sigmoid theo khoảng cách tới goal); phát hiện cực tiểu cục bộ có hysteresis (`iapf_f_enter`, `iapf_f_exit`); thoát bằng lực tiếp tuyến 3D chọn trong `iapf_n_tangent` ứng viên, chấm điểm theo hướng goal, khoảng trống phía trước và liên tục với tiếp tuyến trước; giảm dao động theo góc đổi hướng (ngưỡng 30°/60°); giảm tốc theo khoảng cách goal và vật cản, có tốc độ tối thiểu khi thoát cực tiểu; `vz` kẹp `vz_max`.
- APPROACH dùng `k_rep_approach` thay `k_rep`. Lõi được reset khi đổi phase.
- Yaw: hướng ENU từ UAV tới H-Pad (khóa camera vào mục tiêu) khi cách xa hơn 0.1 m; ngược lại theo hướng goal (`apf`) hoặc hướng vận tốc (`iapf`).
- Vật cản (`obstacle_source`):
  - `pointcloud` (mặc định): voxel-downsample (`cloud_voxel_size`), lấy các điểm trong bán kính `cloud_query_radius` (mặc định `2·d0`), mỗi cụm một điểm gần nhất (`cloud_cluster_radius`), tối đa `max_cloud_points` điểm mỗi chu kỳ.
  - `sdf`: cylinder đọc từ world SDF (`world_sdf`, model tên `cyl_*` hoặc `cylinder_obs_*`), dùng điểm gần nhất trên mặt trụ.
  - `both`: dùng cả hai.
  - `k_rep` cần hiệu chỉnh lại khi chuyển giữa cylinder và point cloud.

**`planner_merge_node`** (node `planner_merge`, timer 20 Hz)

Ghép đầu ra planner và IBVS thành `planner/velocity_setpoint` (`PlannerOutput`) cho `offboard_manager` và `input_state_cache`.

- Đầu vào: `/apf/velocity_cmd`, `/apf/yaw_cmd`, `/ibvs/yaw_cmd`, `/mission/phase` (quá `phase_timeout_sec` coi là `IDLE`).
- FOLLOW/APPROACH: chuyển tiếp vận tốc của APF; yaw theo `yaw_source` (`ibvs_apf`: ưu tiên IBVS, dự phòng APF, cuối cùng NaN; hoặc `ibvs`, `apf`, `hold`), mỗi nguồn yaw phải tươi trong `yaw_timeout_sec`.
- Phase khác: publish vận tốc 0 và `yaw = NaN` để `planner/velocity_setpoint` luôn có nhịp, tránh `planner_timeout` giả ở `input_state_cache` khi vào FOLLOW.
- Mất `/apf/velocity_cmd` quá `upstream_timeout_sec` trong FOLLOW/APPROACH: hover (vận tốc 0, yaw NaN) thêm `stale_hover_sec`, sau đó ngừng publish để `planner_timeout` lan tới FSM và Offboard.
- Vận tốc non-finite được thay bằng hover.

**`pointcloud_generator`** (node `apf_pointcloud_generator`)

Sinh "rừng" trụ ngẫu nhiên để thử APF trong RViz2: `num_obs=35`, `map_size=25 m`, cao 2 đến 4 m, `resolution=0.15`, chừa vùng `clear_radius=2 m` quanh gốc. Publish `/map_generator/global_cloud` (`PointCloud2`, frame `world`) ở 1 Hz; `seed >= 0` để tái lập; `publish_static_tf` phát TF tĩnh `world -> map`.

Launch và cấu hình: `ros2 launch apf_planner apf_planner.launch.py generator:=true planner_type:=iapf`, tham số trong `config/apf_params.yaml`.

### `offboard_safety_monitor` (C++)

Đánh giá RC override, EKF, pin và tuổi heartbeat Offboard. Các mức lỗi gồm `NONE`, `RC_OVERRIDE`, `EKF_UNHEALTHY`, `BATTERY_WARNING`, `BATTERY_CRITICAL`, `OFFBOARD_LOST_SHORT` và `OFFBOARD_LOST_LONG`.

- Timer 5 Hz.
- Pin warning dưới `30%`, critical dưới `15%`.
- Mất Offboard 1 giây: yêu cầu HOLD.
- Mất Offboard 5 giây: yêu cầu RTL.
- Phát lệnh PX4, `safety/inhibit_offboard` và `safety/force_land`.
- `force_land_requested` mặc định latch đến khi node restart; dùng `force_land_latched=false` chỉ cho bench/test với battery fresh và hợp lệ.
- Xem [Safety_Guide.md](ros2_ws/src/offboard_safety_monitor/Safety_Guide.md) để biết policy reset.

### `rc_parser` (C++)

Đọc SBUS, giải mã 16 channel 11-bit, đổi thành microseconds và phát dữ liệu RC (`rc/channels_raw`, `rc/fsm_input`).

- Land switch channel mặc định `4`, kill switch channel `5` (zero-based).
- Ngưỡng thấp/cao `1200/1800 us`.
- Frame timeout `0.5 s`; khi mất frame hoặc lỗi đọc, node vẫn publish `valid=false`, `failsafe=true`.

### `kill_switch` (C++)

Debounce kill channel trong ba chu kỳ, sau đó latch trạng thái killed và gửi `VEHICLE_CMD_COMPONENT_ARM_DISARM` với force-disarm (`param2=21196`). Phát thêm `system/killed` với QoS transient-local.

Kill switch không tự kích hoạt khi mất RC; tuy nhiên khi đã publish `system/killed`, `fsm_state_machine` dừng update (không publish gì nữa) và `offboard_manager` reset engage, dừng heartbeat/setpoint. `rc/fsm_input.kill_switch` chỉ được FSM lưu lại, không dùng trực tiếp để dừng.

### `xrce_bridge_manager` (Python)

Kiểm tra và khởi động `MicroXRCEAgent` bằng lệnh:

```bash
MicroXRCEAgent serial --dev <serial_port> -b <baudrate>
```

Timeout kết nối mặc định `2.0 s`. Trạng thái connected yêu cầu Agent còn sống và `/fmu/out/vehicle_status` còn fresh; process chết hoặc status stale đều kích hoạt reconnect. Xem [XRCE_Guide.md](ros2_ws/src/xrce_bridge_manager/XRCE_Guide.md).

### `vdt_bringup`

Launch orchestration cho các node của workspace và nạp [System_Params.yaml](System_Params.yaml). Không khởi động module vision và `apf_planner`.

## Module vision

Module vision gồm ba package Python trong `ros2_ws/src/` cùng planner `apf_planner`. Module nhận ảnh từ camera và cung cấp phát hiện H-Pad, ước lượng trạng thái mục tiêu, lệnh yaw/pitch theo pixel và lệnh dẫn đường tránh vật cản.

| Package | Node | Vai trò |
|---|---|---|
| `aruco_detector` | `aruco_node` | Phát hiện ArUco, publish `/hpad/detected`, `/hpad/bbox`, `/hpad/pose`, `/hpad/position_camera`, `/hpad/mask_polygon`, `/hpad/annotated` |
| `aruco_detector` | `depth_to_image_node` | Che vùng marker trên depth và chuẩn hóa mono8, publish `/depth_camera/image_mono` |
| `ekf_adapter` | `ekf_node` | EKF trạng thái H-Pad trong frame `world`, publish `/hpad/state_filtered` và `/ekf/tracking_mode` |
| `ekf_adapter` | `odom_tf_node` | TF `world -> base_link` từ `/odom` (tắt khi dùng `px4_state_bridge`) |
| `ibvs` | `ibvs_controller` | Visual servoing, publish `/ibvs/pitch_trim_deg` và `/ibvs/yaw_cmd` |
| `apf_planner` | `planner_node` | APF / I-APF tránh vật cản, publish `/apf/velocity_cmd` (ENU) và `/apf/yaw_cmd` |
| `apf_planner` | `planner_merge_node` | Ghép APF + IBVS thành `planner/velocity_setpoint` |
| `apf_planner` | `pointcloud_generator` | Sinh point cloud vật cản thử nghiệm |

Giao diện giữa embedded và vision:

| Topic | Kiểu | Từ | Đến |
|---|---|---|---|
| `/odom` | `nav_msgs/Odometry` (`world`, ENU) | `px4_state_bridge` | `planner_node`, `ibvs_controller`, `input_state_cache` |
| TF `world` -> `base_link` -> `gimbal_link` -> `camera_optical_frame` | TF | `px4_state_bridge` | `ekf_node` |
| `/mission/phase` | `std_msgs/String` | `vision_interface_bridge` | `ekf_node`, `planner_node`, `planner_merge_node`, `ibvs_controller` |
| `/gimbal/target_angle_deg` | `std_msgs/Float32` | `gimbal_control` | `ibvs_controller`, `px4_state_bridge`, `servo_control` |
| `/hpad/detected`, `/hpad/bbox` | `Bool`, `BoundingBox2D` | `aruco_node` | `vision_interface_bridge`, `ibvs_controller` |
| `/hpad/position_camera` | `PointStamped` (camera frame) | `aruco_node` | `ekf_node` |
| `/hpad/mask_polygon` | `PolygonStamped` (pixel IR1) | `aruco_node` | `depth_to_image_node` |
| `/hpad/state_filtered` | `nav_msgs/Odometry` (`world`) | `ekf_node` | `input_state_cache`, `planner_node`, `ibvs_controller` (`ibvs_controller` cần remap, xem [Chạy hệ thống](#chạy-hệ-thống)) |
| `/ekf/tracking_mode` | `std_msgs/String` | `ekf_node` | `input_state_cache`, `planner_node`, `ibvs_controller` |
| `/ibvs/pitch_trim_deg` | `std_msgs/Float32` | `ibvs_controller` | `gimbal_control` |
| `/ibvs/yaw_cmd` | `std_msgs/Float64` | `ibvs_controller` | `planner_merge_node` |
| `/apf/velocity_cmd`, `/apf/yaw_cmd` | `Twist`, `Float64` | `planner_node` | `planner_merge_node` |
| `/map_generator/global_cloud` | `sensor_msgs/PointCloud2` (`world`) | `pointcloud_generator` | `planner_node` |
| `planner/velocity_setpoint` | `offboard_manager/PlannerOutput` (ENU) | `planner_merge_node` | `offboard_manager`, `input_state_cache` |
| `vision/marker` | `fsm_state_machine/VisionMarker` | `vision_interface_bridge` | `input_state_cache` |
| `alt_estimator/state` | `fsm_state_machine/AltEstimate` | `px4_state_bridge` | `input_state_cache` |

Yêu cầu với node phát hiện: publish `/hpad/detected` ở mọi khung hình và chỉ publish `/hpad/bbox` khi phát hiện thành công. `/camera_info` cấp cho `aruco_node` và `ibvs_controller` phải cùng camera với ảnh phát hiện marker (IR1).

## Messages

| Package | Message | Trường |
|---|---|---|
| `vdt_msgs` | `InputSnapshot` | `valid`, `marker_detected`, `align_error`, `altitude`, `delta_h`, `d_horiz`, `yaw_rate`, `touchdown`, `planner_timeout` |
| `vdt_msgs` | `TimeoutFlags` | `ekf_timeout`, `vision_timeout`, `alt_timeout`, `planner_timeout` |
| `vdt_msgs` | `VisionMarker` | `marker_visible`, `pixel_align_error` |
| `vdt_msgs` | `AltEstimate` | `altitude`, `touchdown_flag` |
| `vdt_msgs` | `RcFsmInput` | `land_switch`, `kill_switch` |
| `vdt_msgs` | `RcChannelsRaw` | `int16[16] ch`, `valid`, `failsafe` |
| `vdt_msgs` | `PlannerOutput` | `vx`, `vy`, `vz` (ENU, m/s), `yaw` (heading ENU, rad, NaN là giữ yaw) |
| `vdt_msgs` | `OffboardStatus` | `offboard_active`, `heartbeat_age_sec` |

Ghi chú:

- `InputSnapshot.delta_h`, `d_horiz`, `align_error` là `NaN` khi không có mục tiêu hợp lệ; `altitude` là `NaN` khi `alt_estimator/state` không hợp lệ.
- `InputSnapshot.yaw_rate` là rad/s quanh trục yaw; `NaN` làm FSM không vào được FOLLOW. `align_error` hoặc `altitude` là `NaN` thì FSM đặt `valid = false`.
- `VisionMarker.pixel_align_error` là sai số chuẩn hóa theo nửa kích thước ảnh; là `NaN` khi `marker_visible = false`.

Repository hiện không định nghĩa service hoặc action interface. Các topic vision dùng message chuẩn (`geometry_msgs`, `nav_msgs`, `sensor_msgs`, `std_msgs`, `vision_msgs`, `visualization_msgs`).

## Topics chính

### Input của FSM

| Topic | Type |
|---|---|
| `input_cache/snapshot` | `input_state_cache/msg/InputSnapshot` |
| `rc/fsm_input` | `rc_parser/msg/RcFsmInput` |
| `safety/force_land` | `std_msgs/msg/Bool` |
| `system/killed` | `std_msgs/msg/Bool` |

### Input của `input_state_cache`

| Topic | Type |
|---|---|
| `hpad/state_filtered` | `nav_msgs/msg/Odometry` |
| `ekf/tracking_mode` | `std_msgs/msg/String` |
| `odom` | `nav_msgs/msg/Odometry` |
| `vision/marker` | `fsm_state_machine/msg/VisionMarker` |
| `alt_estimator/state` | `fsm_state_machine/msg/AltEstimate` |
| `planner/velocity_setpoint` | `offboard_manager/msg/PlannerOutput` |

### Input của `offboard_manager`

| Topic | Type |
|---|---|
| `fsm/state` | `std_msgs/msg/UInt8` |
| `planner/velocity_setpoint` | `offboard_manager/msg/PlannerOutput` |
| `input_cache/timeout_flags` | `input_state_cache/msg/TimeoutFlags` |
| `system/killed`, `safety/inhibit_offboard` | `std_msgs/msg/Bool` |

### Output nội bộ

`input_cache/snapshot`, `input_cache/timeout_flags`, `fsm/state`, `gimbal/state_request`, `planner/mode`, `planner/apf_gain`, `gimbal/align_error_cmd`, `cmd/vertical_descent_rate`, `cmd/disarm_request`, `gimbal/target_angle_deg`, `rc/fsm_input`, `rc/channels_raw`, `offboard/status`, `safety/inhibit_offboard`, `safety/force_land`, `system/killed`, `/odom`, `alt_estimator/state`, `vision/marker`, `/mission/phase`, `planner/velocity_setpoint`, TF `world -> base_link -> gimbal_link -> camera_optical_frame`.

### Output của module vision

`/hpad/detected`, `/hpad/bbox`, `/hpad/pose`, `/hpad/position_camera`, `/hpad/mask_polygon`, `/hpad/annotated`, `/depth_camera/image_mono`, `/hpad/state_filtered`, `/ekf/tracking_mode`, `/ibvs/pitch_trim_deg`, `/ibvs/yaw_cmd`, `/apf/velocity_cmd`, `/apf/yaw_cmd`, `/apf/force_markers`, `/map_generator/global_cloud`.

### Giao tiếp PX4

- Input từ PX4: `/fmu/out/vehicle_status`, `/fmu/out/vehicle_local_position`, `/fmu/out/battery_status`, `/fmu/out/vehicle_odometry`, `/fmu/out/vehicle_land_detected`.
- Output đến PX4: `/fmu/in/offboard_control_mode`, `/fmu/in/trajectory_setpoint`, `/fmu/in/vehicle_command`.

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
| `rc_node` | `land_channel=4`, `kill_channel=5`, `low_threshold=1200`, `high_threshold=1800`, `frame_timeout=0.5`, `debug_enabled=false` |
| `kill_switch_node` | `kill_channel=5`, `low_threshold=1200`, `high_threshold=1800`, `debounce_threshold=3`, `debug_enabled=false` |
| `servo_node` | `angle_min_deg=0`, `angle_max_deg=180`, `home_angle_deg=90`, `input_timeout_sec=1.0`, `debug_enabled=false` |
| `xrce_bridge_node` | `connection_timeout_sec=2.0`, `debug_enabled=false` |

Tham số `max_yaw` của `offboard_node` và `yaw_search_rate` của `fsm_node` đã bị loại bỏ; các dòng này trong `System_Params.yaml` (nếu còn) có thể xóa.

Tham số của module vision và planner không nằm trong `System_Params.yaml`:

| Node | Nguồn tham số | Tham số chính |
|---|---|---|
| `aruco_node` | Tham số node (`--ros-args -p`) | `marker_id=42`, `marker_size_m=0.15`, `dictionary=DICT_6X6_50`, `min_detection_distance_m=0.0`, `min_z_m=0.0`, `image_topic=/camera`, `camera_info_topic=/camera_info`, `camera_frame_id=camera_optical_frame`, `fallback_horizontal_fov_rad=1.52`, `require_camera_info=true`, `mask_margin_percent=0.15` |
| `ekf_node` | Tham số node và launch arguments của `ekf.launch.py` | `process_accel_variance=[1.0, 1.0, 0.5]`, `gate_threshold=16.27`, `max_target_speed=2.5`, `max_target_vz=1.5`, `target_frame=world`, `child_frame=hpad`, `tf_timeout_s=0.03`, `output_rate_hz=50.0`, `position_topic=/hpad/position_camera`, `phase_topic=/mission/phase`, `state_topic=/hpad/state_filtered`, `mode_topic=/ekf/tracking_mode` |
| `ibvs_controller` | `ibvs/config/ibvs_params.yaml` | `K_pitch=0.8`, `K_yaw=0.5`, `search_yaw_rate=0.2`, `pitch_rate_limit=1.5`, `pitch_ema_alpha=0.25`, `pitch_pixel_trim_gain=0.5`, `pitch_trim_limit_deg=20.0`, `yaw_rate_limit=0.5` |
| `apf_planner` | `apf_planner/config/apf_params.yaml` | `planner_type=iapf` (node mặc định `apf`), `obstacle_source=pointcloud`, `target_topic=/hpad/state_filtered`, `allowed_tracking_modes=[TRACKING, PREDICTING]`, `data_timeout_sec=0.5`, `rate_hz=30`, `d0=2.0`, `v_max=1.2`, `d_slow=1.5`, `k_att=10`, `k_rep=250`, `k_rep_approach=125`, `goal_threshold=0.20`, `follow_distance=3.5`, `hold_follow_altitude=true`, `target_altitude=3.0`, `k_z=0.6`, `vz_max=0.5`, `target_lead_time=0.25`, `max_target_lead=0.75`, `target_velocity_alpha=0.25`, `target_velocity_deadband=0.08`, `cloud_voxel_size=0.3`, `cloud_query_radius=0` (tức `2·d0`), `cloud_cluster_radius=1.2`, `max_cloud_points=8`, `iapf_f_enter=0.10`, `iapf_f_exit=0.30`, `iapf_k_tan=1.0`, `iapf_n_tangent=12`, `iapf_n_pred=3`, `iapf_w_goal=1.0`, `iapf_w_clear=1.0`, `iapf_w_prev=0.60` |
| `planner_merge` | `apf_planner/config/apf_params.yaml` | `rate_hz=20`, `yaw_source=ibvs_apf`, `upstream_timeout_sec=0.3`, `stale_hover_sec=0.5`, `yaw_timeout_sec=0.3`, `phase_timeout_sec=1.0`, `output_topic=planner/velocity_setpoint` |
| `apf_pointcloud_generator` | `apf_planner/config/apf_params.yaml` | `topic=/map_generator/global_cloud`, `frame_id=world`, `rate_hz=1.0`, `num_obs=35`, `map_size=25.0`, `height=4.0`, `resolution=0.15`, `clear_radius=2.0`, `seed=-1`, `publish_static_tf=true` |

Launch arguments của `ekf.launch.py`: `use_sim_time=false`, `publish_odom_tf=true`, `publish_camera_tf=true`, `odom_topic=/odom`, `world_frame=world`, `base_frame=base_link`, `camera_frame=camera_optical_frame`, `cam_x/y/z`, `cam_roll/pitch/yaw`. Danh sách đầy đủ tham số gate, tái bắt và nhiễu measurement nằm trong [EKF_Guide.md](ros2_ws/src/ekf_adapter/EKF_Guide.md).

Launch arguments của `apf_planner.launch.py`: `params_file`, `generator=false`, `planner_type=iapf`.

Launch arguments của `vdt_bringup`:

| Argument | Mặc định | Ý nghĩa |
|---|---:|---|
| `start_hardware` | `true` | Bật `rc_node` và `kill_switch_node` |
| `start_servo` | `false` | Bật `servo_node` |
| `debug` | `false` | Ghi đè `debug_enabled` cho các node |

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
```

Launch sequence theo thời gian là `XRCE -> PX4 state bridge -> vision bridge -> input cache -> RC/kill -> safety -> FSM/gimbal -> Offboard -> servo`. Đây là thứ tự khởi tạo process; readiness thật vẫn do các node kiểm tra freshness, timeout, health và mode PX4.

Module vision và `apf_planner` (`planner_node`, `planner_merge_node`) được khởi động riêng, xem [Chạy hệ thống](#chạy-hệ-thống).

## Luồng hoạt động

1. `xrce_bridge_manager` kết nối ROS 2 với PX4 qua Micro XRCE-DDS.
2. Cất cánh bằng PX4 takeoff. `offboard_manager` chỉ engage Offboard khi UAV đã armed và đạt `min_engage_altitude_m`.
3. `px4_state_bridge` đổi trạng thái PX4 sang `/odom`, chuỗi TF tới camera và `alt_estimator/state`.
4. `aruco_node` phát hiện H-Pad trên ảnh IR1 và publish `/hpad/detected`, `/hpad/bbox`, `/hpad/position_camera`. `ekf_node` đổi vị trí sang `world` bằng TF tại stamp ảnh và ước lượng trạng thái H-Pad (`/hpad/state_filtered`, `/ekf/tracking_mode`).
5. `vision_interface_bridge` tạo `vision/marker` từ kết quả phát hiện và `/mission/phase` từ `fsm/state`.
6. `ibvs_controller` dùng `/mission/phase` để quét yaw ở SEARCH, hiệu chỉnh pitch gimbal (`/ibvs/pitch_trim_deg`) và tính `/ibvs/yaw_cmd` ở FOLLOW/APPROACH.
7. `planner_node` (khi phase là FOLLOW/APPROACH và EKF ở `TRACKING`/`PREDICTING`) tính vận tốc tránh vật cản `/apf/velocity_cmd` và `/apf/yaw_cmd` từ `/odom`, `/hpad/state_filtered` và vật cản.
8. `planner_merge_node` ghép vận tốc APF với yaw (ưu tiên IBVS) thành `planner/velocity_setpoint` và publish đều ở mọi phase.
9. `input_state_cache` tập hợp dữ liệu, kiểm tra độ tươi và tính hợp lệ, tính hình học tương đối giữa UAV và H-Pad rồi phát `InputSnapshot` đồng bộ.
10. FSM nhận `InputSnapshot`, cập nhật bộ đếm và phát lệnh cho planner, gimbal và Offboard manager.
11. `gimbal_control` tính góc nền theo state, cộng hiệu chỉnh từ IBVS rồi phát `gimbal/target_angle_deg`.
12. `offboard_manager` đổi `planner/velocity_setpoint` (ENU) sang NED và gửi PX4 qua `/fmu/in/*`; planner stale thì UAV hover.
13. `offboard_safety_monitor` và `kill_switch` có thể inhibit Offboard, yêu cầu HOLD/RTL/force-land hoặc force-disarm độc lập với perception.

## Cài đặt và build

### Phụ thuộc

- ROS 2 với `ament_cmake` và `ament_python`.
- `rclcpp`, `rclpy`, `std_msgs`, `nav_msgs`, `geometry_msgs`, `sensor_msgs`, `vision_msgs`, `visualization_msgs`, `tf2_ros`, `px4_msgs`.
- `rosidl_default_generators`, `rosidl_default_runtime`.
- `MicroXRCEAgent`.
- Driver camera phát ảnh IR1, `CameraInfo` của IR1 và depth align với IR1.
- `aruco_detector`: `numpy`, `opencv-contrib-python` (cần `cv2.aruco`; từ OpenCV 4.7 dùng `ArucoDetector`), `ros-<distro>-vision-msgs`.
- `ekf_adapter`: `numpy`, `ros-<distro>-tf2-ros`, `nav-msgs`, `geometry-msgs`, `launch-ros`.
- `ibvs`: `ros-<distro>-vision-msgs`.
- `apf_planner`: `python3-numpy`, `tf2_ros_py`, `visualization_msgs`, `offboard_manager` (message `PlannerOutput`), `launch_ros`.

Không cài song song `opencv-python` và `opencv-contrib-python` trong cùng môi trường. Nếu hệ thống đã có `python3-opencv` qua apt và có `cv2.aruco` thì không cần cài thêm.

### Build ROS 2

Thực hiện trên Linux có ROS 2 đã cài:

```bash
cd ros2_ws
source /opt/ros/<ros_distro>/setup.bash
rosdep install --from-paths src --ignore-src -r -y
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

Build các package vision và planner (`apf_planner` cần `offboard_manager` đã build):

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

`requirements.txt` gồm `pandas` và `numpy`. Logger còn cần môi trường ROS 2, `px4_msgs` và `fsm_state_machine`.

## Chạy hệ thống

Khuyến nghị dùng launch orchestration:

```bash
source /opt/ros/<ros_distro>/setup.bash
source ros2_ws/install/setup.bash
ros2 launch vdt_bringup vdt_system.launch.py
```

Các tùy chọn bench:

```bash
ros2 launch vdt_bringup vdt_system.launch.py start_hardware:=false
ros2 launch vdt_bringup vdt_system.launch.py start_servo:=true
ros2 launch vdt_bringup vdt_system.launch.py debug:=true
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

EKF dùng TF của `px4_state_bridge` nên tắt hai nguồn TF của launch (nếu không sẽ trùng/xung đột TF):

```bash
ros2 launch ekf_adapter ekf.launch.py \
  publish_odom_tf:=false publish_camera_tf:=false
```

IBVS đọc `/ekf/target_state` cố định trong code, trong khi `ekf_node` publish `/hpad/state_filtered`, nên cần remap:

```bash
ros2 run ibvs ibvs_controller --ros-args \
  -r /ekf/target_state:=/hpad/state_filtered
```

Planner tránh vật cản và node ghép (`planner_node` đọc `/hpad/state_filtered` mặc định, không cần remap):

```bash
ros2 launch apf_planner apf_planner.launch.py planner_type:=iapf
```

Thử với point cloud vật cản giả lập trong RViz2:

```bash
ros2 launch apf_planner apf_planner.launch.py generator:=true planner_type:=iapf
```