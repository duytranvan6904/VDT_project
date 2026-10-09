# Landing Guidance — Guide

## 1. Package cần cài đặt ngoài

- Package chuẩn ROS 2: `rclpy`, `std_msgs`, `geometry_msgs`, `nav_msgs`, `diagnostic_msgs`.
- `px4_msgs` (lấy `VehicleLandDetected`), `python3-numpy`.

## 2. Nguyên lý hoạt động

```text
odom, ekf/target_state, ekf/tracking_mode
        └─► covariance_gate_node ─► /landing/safe_to_land
                                    /landing/uncertainty_radius
                                    /landing/rswitch_adaptive
                                    /landing/sliding_weight
                                          │
mission/phase, odom, ekf/target_state     ▼
        └─────────────────────► smc_guidance_node ─► /landing/velocity_cmd
                                                     /landing/active
                                                     /landing/sub_phase

mission/phase, odom, hpad/position_camera,
cmd/vertical_descent_rate, fmu/out/vehicle_land_detected
        └─► touchdown_detector_node ─► /landing/touchdown
```

Phần toán nằm trong `*_logic.py` (không import `rclpy`), phần ROS nằm trong `*_node.py`.

### 2.1 `covariance_gate_node`

1. Lấy covariance từ `/odom` (UAV) và `/ekf/target_state` (H-Pad). Covariance không hợp lệ thì dùng mặc định (UAV: `default_drone_eph_m`/`default_drone_epv_m`, mục tiêu: 1.0 m).
2. Với `use_marker_relative`, sai số tương đối = covariance mục tiêu + sai số tư thế (`độ cao × attitude_sigma_rad`) + sai số độ cao PX4. Sai số GPS ngang bị loại.
3. `r_uncertainty` là 2σ theo trục lớn của elip ngang.
4. `safe_to_land = true` khi: `r_uncertainty` trong phễu `max(pad_radius_m, 0.12 × Δh)` và dưới ngưỡng glide, EKF đang `TRACKING`, khoảng cách ngang không quá `max_horizontal_distance_m`.
5. `rswitch_adaptive` nằm trong `[r_switch_min_m, r_switch_max_m]`, covariance càng xấu thì càng nhỏ. `sliding_weight` giảm khi covariance lớn.

### 2.2 `smc_guidance_node`

Chỉ chạy khi `/mission/phase` là `APPROACH` hoặc `LAND`. Ngoài hai pha này, hoặc khi dữ liệu không tươi, `safe_to_land = false`, EKF không `TRACKING`/`PREDICTING`: phát vận tốc 0 và `/landing/active = false`.

| Điều kiện | Sub-phase | Lệnh |
|---|---|---|
| `r_xy > rswitch` | `GLIDE_SLOPE` | Bay ngang về pad, giữ độ cao |
| `r_xy <= rswitch`, cao hơn `final_descent_alt_m` | `GLIDE_SLOPE` | Vừa tiến vừa hạ |
| `r_xy <= min(rswitch, final_descent_rxy_m)` và `r_z <= final_descent_alt_m` | `FINAL_DESCENT` | Căn tâm tối đa `v_final_xy_max`, hạ `v_descend_fast` (`v_descend_touch` khi `r_z <= 0.4`) |
| `FINAL_DESCENT`, `r_uncertainty > final_uncertainty_max_2sigma_m`, `r_z > 0.4` | `FINAL_GATE_WAIT` | Giữ độ cao, vẫn căn tâm |

- `/landing/active = true` khi `r_xy <= rswitch`. `planner_merge_node` dùng cờ này để chỉ nhận SMC ở gần pad, xa hơn vẫn dùng APF.
- Vận tốc đầu ra là điều khiển tỉ lệ theo vị trí cộng feedforward vận tốc pad. Các mặt trượt S1, S2, S3 chỉ ảnh hưởng `yaw_rate`, và `angular.z` không được phát ra.

### 2.3 `touchdown_detector_node`

Chỉ chạy ở `LAND`, hoặc `APPROACH` khi độ cao `<= altitude_ceiling_m`. Chốt `touchdown = true` (latched) khi một điều kiện sau giữ liên tục `confirmation_duration_s`:

| Điều kiện | Chi tiết |
|---|---|
| `PX4_FIRMWARE_LANDED` | `vehicle_land_detected.landed` và `seen_descending` |
| `KINEMATIC_STOPPED` | Hạ liên tục đã chứng minh, vận tốc đứng yên, và `optical_z <= optical_height_threshold_m` |
| `IMPACT_JERK_STOPPED` | Như trên cộng jerk lớn (chưa nối `accel_z_body` nên nhánh này không chạy) |

"Hạ liên tục đã chứng minh" (chống touchdown giả lúc mới vào LAND):

- Lệnh hạ (`-cmd/vertical_descent_rate <= descent_cmd_threshold_mps`) giữ liên tục ít nhất `min_descent_time_s`. Lệnh hạ ngừng thì bộ đếm về 0.
- `seen_descending` bật khi vận tốc thực `vz <= seen_descending_vz_mps` liên tục ít nhất `seen_descending_hold_s`, giữ đến khi detector reset.
- Mất marker (không có `optical_z`) thì không có đường `KINEMATIC_STOPPED`, chỉ còn `PX4_FIRMWARE_LANDED`.

## 3. Cách chạy

```bash
ros2 launch landing_guidance landing.launch.py
```

Dùng file tham số khác:

```bash
ros2 launch landing_guidance landing.launch.py params_file:=/path/to/System_Params.yaml
```

Chạy riêng từng node:

```bash
ros2 run landing_guidance covariance_gate_node
ros2 run landing_guidance smc_guidance_node
ros2 run landing_guidance touchdown_detector_node
```

Tham số nằm trong `System_Params.yaml`, mục `/covariance_gate_node`, `/smc_guidance_node`, `/touchdown_detector_node`.

`covariance_gate_node`:

| Tên | Mặc định | Ý nghĩa |
|---|---|---|
| `pad_radius_m` | 0.25 | Bán kính H-Pad, giới hạn phễu tại mặt pad |
| `confidence_sigma` | 2.0 | Hệ số σ của bán kính bất định |
| `max_measurement_age_s` | 0.5 | Tuổi tối đa của đo đạc vision |
| `max_horizontal_distance_m` | 5.0 | Khoảng cách ngang tối đa để cho phép hạ |
| `r_switch_min_m` / `r_switch_max_m` | 3.0 / 15.0 | Khoảng của bán kính chuyển pha thích nghi |
| `sigma_ideal_m` / `sigma_bad_m` | 0.05 / 0.30 | σ ngang tốt / xấu để nội suy `rswitch` |
| `sigma_max_continue_glide_m` | 0.15 | σ tối đa để tiếp tục glide (nhân `confidence_sigma`) |
| `sliding_beta` | 2.0 | Độ nhạy của `sliding_weight` |
| `default_drone_eph_m` / `default_drone_epv_m` | 0.10 / 0.15 | Sai số UAV khi `/odom` không có covariance |
| `use_marker_relative` | true | Dùng sai số tương đối theo marker, loại sai số GPS ngang |
| `attitude_sigma_rad` | 0.0087 | Sai số tư thế dùng cho sai số theo độ cao |
| `eval_rate_hz` | 20.0 | Tần số đánh giá |

`smc_guidance_node`:

| Tên | Mặc định | Ý nghĩa |
|---|---|---|
| `v_max` | 1.50 | Vận tốc tối đa khi tiếp cận (m/s) |
| `v_final_xy_max` | 0.35 | Vận tốc căn tâm tối đa ở pha cuối |
| `v_descend_fast` / `v_descend_touch` | 0.35 / 0.15 | Tốc độ hạ khi cao / gần đất |
| `rswitch_default` | 0.80 | Bán kính chuyển pha khi chưa nhận `rswitch_adaptive` |
| `final_descent_rxy_m` | 1.00 | Bán kính ngang tối đa để vào `FINAL_DESCENT` |
| `final_descent_alt_m` | 0.70 | Độ cao tối đa để vào `FINAL_DESCENT` |
| `final_uncertainty_max_2sigma_m` | 0.35 | Quá ngưỡng này thì giữ độ cao |
| `max_state_age_s` / `max_covariance_age_s` | 0.30 / 0.30 | Tuổi tối đa của odom/target và của tin gate |
| `control_rate_hz` | 20.0 | Tần số điều khiển |

`touchdown_detector_node`:

| Tên | Mặc định | Ý nghĩa |
|---|---|---|
| `optical_height_threshold_m` | 0.40 | `optical_z` nhỏ hơn mức này coi là sát mặt pad |
| `descent_cmd_threshold_mps` | -0.10 | Lệnh hạ tối thiểu để tính là đang hạ |
| `stoppage_vz_max_mps` / `stoppage_dz_dt_max_mps` | 0.08 / 0.05 | Ngưỡng đứng yên theo phương đứng |
| `jerk_impact_threshold` | 3.50 | Ngưỡng jerk va chạm (m/s³) |
| `confirmation_duration_s` | 0.35 | Thời gian giữ điều kiện trước khi chốt |
| `altitude_ceiling_m` | 0.80 | Ở `APPROACH`, chỉ đánh giá khi dưới độ cao này |
| `min_descent_time_s` | 1.0 | Thời gian hạ liên tục tối thiểu |
| `seen_descending_vz_mps` | -0.10 | Vận tốc thực để tính là đã hạ |
| `seen_descending_hold_s` | 0.30 | Thời gian giữ vận tốc đó để bật `seen_descending` |
| `descent_rate_topic` | `cmd/vertical_descent_rate` | Topic lệnh hạ của FSM (`std_msgs/Float64`) |
| `eval_rate_hz` | 20.0 | Tần số đánh giá |

Topic đầu vào:

| Topic | Kiểu | QoS | Node dùng | Nguồn |
|---|---|---|---|---|
| `/odom` | `nav_msgs/Odometry` | best effort | cả 3 | `px4_state_bridge` |
| `/ekf/target_state` | `nav_msgs/Odometry` | reliable | gate, SMC | EKF của vision |
| `/ekf/tracking_mode` | `std_msgs/String` | reliable | gate, SMC | EKF của vision |
| `/mission/phase` | `std_msgs/String` | reliable | SMC, touchdown | FSM |
| `/landing/safe_to_land`, `uncertainty_radius`, `rswitch_adaptive`, `sliding_weight` | `Bool`/`Float64` | reliable | SMC | gate |
| `/hpad/position_camera` | `geometry_msgs/PointStamped` | reliable | touchdown | vision |
| `cmd/vertical_descent_rate` | `std_msgs/Float64` | reliable | touchdown | FSM |
| `/fmu/out/vehicle_land_detected` | `px4_msgs/VehicleLandDetected` | best effort | touchdown | PX4 |

Topic đầu ra:

| Topic | Kiểu | Node |
|---|---|---|
| `/landing/safe_to_land` | `std_msgs/Bool` | gate |
| `/landing/uncertainty_radius`, `rswitch_adaptive`, `sliding_weight` | `std_msgs/Float64` | gate |
| `/landing/covariance_status` | `diagnostic_msgs/DiagnosticStatus` | gate |
| `/landing/velocity_cmd` | `geometry_msgs/Twist` (ENU, chỉ dùng `linear`) | SMC |
| `/landing/active` | `std_msgs/Bool` | SMC |
| `/landing/sub_phase` | `std_msgs/String` | SMC |
| `/landing/smc_status` | `diagnostic_msgs/DiagnosticStatus` | SMC |
| `/landing/touchdown` | `std_msgs/Bool` | touchdown |
| `/landing/touchdown_status` | `diagnostic_msgs/DiagnosticStatus` | touchdown |

## 4. Cách debug

```bash
ros2 topic echo /landing/covariance_status
ros2 topic echo /landing/smc_status
ros2 topic echo /landing/touchdown_status
ros2 topic echo /landing/safe_to_land
ros2 topic echo /landing/active
ros2 topic echo /landing/velocity_cmd
ros2 topic echo /landing/touchdown
```

Kiểm tra đầu vào:

```bash
ros2 topic echo /mission/phase
ros2 topic echo /ekf/tracking_mode
ros2 topic echo /odom --field pose.covariance
ros2 topic echo /fmu/out/vehicle_land_detected --qos-reliability best_effort
ros2 topic info -v cmd/vertical_descent_rate
```

Gate và SMC in log mỗi 2 s:

```text
[covariance_gate_node]: [GATE_HOLD] r_unc=0.412m DistXY=2.31m Uncertainty radius 0.412m > allowed funnel 0.280m
[smc_guidance_node]: [GLIDE_SLOPE] Rxy=3.20 Rz=2.10 cmd=(0.85,-0.12,0.00)
```

## 5. Test

Chưa có test tự động. Dự kiến `test/` gồm `test_covariance_logic.py`, `test_smc_logic.py`, `test_touchdown_logic.py` (pytest, không cần ROS 2), sau đó SITL trong `ros2_ws/scripts/SITL`.

Build lên UAV:

```bash
colcon build --packages-select landing_guidance
```

## 6. Xử lý sự cố thường gặp

1. **`safe_to_land = false` liên tục:**
   - Đọc trường `message` của `/landing/covariance_status` để biết lý do (bất định lớn, mất vision, quá xa).
   - `/ekf/tracking_mode` phải là `TRACKING`; mode `PREDICTING` làm gate chặn.
2. **`/landing/velocity_cmd` luôn 0:**
   - `/mission/phase` phải là `APPROACH` hoặc `LAND`.
   - Kiểm tra `safe_to_land`, tuổi dữ liệu (`max_state_age_s`, `max_covariance_age_s`) và `/ekf/tracking_mode`.
3. **`/landing/active` luôn `false`:**
   - `r_xy` còn lớn hơn `rswitch_adaptive` (APF vẫn điều khiển), hoặc SMC đang ở trạng thái giữ.
4. **Sai số UAV luôn là mặc định (0.10/0.15 m):**
   - `/odom` chưa điền covariance. Kiểm tra `ros2 topic echo /odom --field pose.covariance` và `px4_state_bridge`.
5. **`touchdown` không bao giờ lên `true`:**
   - Xem `/landing/touchdown_status`: `seen_descending`, `descent_time_s`, `optical_near`.
   - Kiểm tra `cmd/vertical_descent_rate` có được publish và đúng kiểu `std_msgs/Float64`.
   - Không có `/hpad/position_camera` thì chỉ còn đường `PX4_FIRMWARE_LANDED`.
6. **Không nhận `/fmu/out/vehicle_land_detected`:**
   - Subscriber phải dùng QoS best effort; reliable sẽ không nhận được.
7. **Node không chạy trong launch:**
   - `ros2 pkg executables landing_guidance` phải có 3 node.
   - `System_Params.yaml` trong `install/bringup` phải có đủ 3 khối tham số (build lại `bringup` sau khi sửa).