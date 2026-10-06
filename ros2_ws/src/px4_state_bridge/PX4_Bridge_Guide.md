# PX4 State Bridge — Guide

## 1. Package cần cài đặt ngoài

- Package chuẩn ROS 2: `rclcpp`, `nav_msgs`, `geometry_msgs`, `std_msgs`, `tf2_ros`.
- `px4_msgs`: phải clone và build từ nguồn vào workspace, phiên bản phải khớp với firmware PX4 đang chạy.
- Phụ thuộc `vdt_msgs` để lấy định nghĩa `AltEstimate`.
- Micro XRCE-DDS Agent đang chạy để PX4 publish các topic `/fmu/out/...`.

## 2. Nguyên lý hoạt động

```text
/fmu/out/vehicle_odometry       --\
/fmu/out/vehicle_land_detected  --> px4_state_bridge
gimbal/target_angle_deg         --/        │
                                           ├─► /odom (Odometry, frame world, ENU)
                                           │
                                           ├─► TF: world ─► base_link ─► gimbal_link ─► camera_optical_frame
                                           │
                                           └─► alt_estimator/state (AltEstimate, 20Hz)
```

Node chuyển trạng thái PX4 (NED/FRD) sang dạng mà các node vision và embedded cần (ENU/FLU, frame `world`).

### Quy ước đổi frame

| Dữ liệu | PX4 | Sau khi đổi |
|---|---|---|
| Vị trí | NED `(n, e, d)` | ENU `(e, n, -d)` |
| Orientation | `q` FRD sang NED | `q` FLU sang ENU: `q_enu_ned * q_ned_frd * q_frd_flu` |
| Vận tốc tuyến tính | NED | ENU, nằm trong frame `world` (không theo quy ước child frame của ROS) |
| Vận tốc góc | FRD `(x, y, z)` | FLU `(x, -y, -z)` |

Kiểm tra nhanh: UAV hướng Bắc có yaw ENU = 90 độ, UAV hướng Đông có yaw ENU = 0.

### Kiểm tra dữ liệu đầu vào

Một bản tin `vehicle_odometry` bị bỏ qua nếu:

- `pose_frame` không phải NED (có log cảnh báo).
- Vị trí hoặc quaternion có `NaN`/`Inf`.
- Độ dài quaternion nhỏ hơn 0.5.

Vận tốc tuyến tính chỉ được đưa vào `/odom` khi `velocity_frame` là NED và hữu hạn. Vận tốc góc chỉ được đưa vào khi hữu hạn.

### `/odom`

- `header.frame_id = world_frame`, `child_frame_id = base_frame`.
- `header.stamp` là thời điểm ROS nhận bản tin (không dùng timestamp của PX4).
- Publish reliable, độ sâu 10, tương thích với subscriber best-effort của vision.

### Chuỗi TF

Mỗi lần nhận odometry, node publish 3 transform cùng timestamp:

| Transform | Nội dung |
|---|---|
| `world_frame` sang `base_frame` | Pose của UAV (ENU, FLU) |
| `base_frame` sang `gimbal_frame` | Tịnh tiến `gimbal_pivot_xyz`; quay quanh trục y một góc `-gimbal_angle` (góc gimbal âm là camera chúi xuống) |
| `gimbal_frame` sang `camera_frame` | Tịnh tiến `camera_in_gimbal_xyz`; quay từ frame link sang optical (x phải, y xuống, z phía trước) |

Góc gimbal lấy từ `gimbal/target_angle_deg` là **góc lệnh** (đã qua bộ làm mượt và giới hạn tốc độ), không phải góc thực của servo, nên có sai lệch open-loop. Trước khi nhận được bản tin gimbal đầu tiên, góc là 0.

### `alt_estimator/state`

- Publish ở 20 Hz, chỉ khi `/fmu/out/vehicle_odometry` còn tươi (trong `odom_timeout_sec`). Nếu odometry cũ, node không publish để `input_state_cache` timeout đúng.
- `altitude` là độ cao ENU so với gốc local frame của PX4 (không phải độ cao so với mặt đất hay H-Pad).
- `touchdown_flag = true` khi `vehicle_land_detected` còn tươi (dưới 1 giây) và `landed = true`. Nếu bật `use_ground_contact` thì `ground_contact = true` cũng được tính.

## 3. Cách chạy

```bash
ros2 launch px4_state_bridge px4_state_bridge.launch.py
```

Hoặc chạy trực tiếp với tham số:

```bash
ros2 run px4_state_bridge px4_state_bridge_node --ros-args \
  -p odometry_topic:=/fmu/out/vehicle_odometry \
  -p land_detected_topic:=/fmu/out/vehicle_land_detected \
  -p gimbal_angle_topic:=/gimbal/target_angle_deg \
  -p camera_frame:=camera_optical_frame \
  -p gimbal_pivot_xyz:="[0.10, 0.0, -0.05]" \
  -p camera_in_gimbal_xyz:="[0.03, 0.0, 0.0]"
```

Danh sách tham số:

| Tên | Mặc định | Ý nghĩa |
|---|---|---|
| `odometry_topic` | `/fmu/out/vehicle_odometry` | Topic odometry của PX4 |
| `land_detected_topic` | `/fmu/out/vehicle_land_detected` | Topic land detector của PX4 |
| `gimbal_angle_topic` | `gimbal/target_angle_deg` | Góc gimbal lệnh (`std_msgs/Float32`, độ) |
| `world_frame` | `world` | Tên frame gốc, trùng frame `world` của vision |
| `base_frame` | `base_link` | Tên frame thân UAV |
| `gimbal_frame` | `gimbal_link` | Tên frame trục gimbal |
| `camera_frame` | `camera_optical_frame` | Tên frame optical của camera; phải trùng `frame_id` của node phát hiện và `source_frame` của EKF |
| `gimbal_pivot_xyz` | `[0, 0, 0]` | Vị trí trục gimbal trong `base_link` (m, x trước, y trái, z lên) |
| `camera_in_gimbal_xyz` | `[0, 0, 0]` | Vị trí camera trong `gimbal_link` (m) |
| `use_ground_contact` | false | Tính cả `ground_contact` vào `touchdown_flag` |
| `odom_timeout_sec` | 0.5 | Odometry cũ hơn mức này thì ngừng publish `alt_estimator/state` |

Topic tên PX4 phụ thuộc phiên bản `px4_msgs`. Nếu `ros2 topic list` cho thấy tên khác (ví dụ có hậu tố phiên bản), đổi qua `odometry_topic` và `land_detected_topic`.

## 4. Hiệu chuẩn

Điền chính xác trước khi bay thật:

- `gimbal_pivot_xyz`: đo từ tâm thân UAV tới trục xoay gimbal.
- `camera_in_gimbal_xyz`: đo từ trục xoay gimbal tới tâm quang học camera.
- `camera_frame`: đặt thành tên mà node phát hiện đang dùng làm `frame_id`.

Sai số các giá trị này đi thẳng vào vị trí H-Pad mà EKF tính trong frame `world`.

## 5. Cách debug

Kiểm tra `/odom`:

```bash
ros2 topic echo /odom --field pose.pose.position
ros2 topic hz /odom
```

Kiểm tra chuỗi TF tới camera:

```bash
ros2 run tf2_ros tf2_echo world camera_optical_frame
```

Xem toàn bộ cây TF:

```bash
ros2 run tf2_tools view_frames
```

Kiểm tra `alt_estimator/state`:

```bash
ros2 topic echo /alt_estimator/state
```

Kiểm tra hướng nhìn của camera theo góc gimbal (UAV nằm ngang):

```bash
ros2 topic pub /gimbal/target_angle_deg std_msgs/msg/Float32 "{data: -90.0}" -r 20
```

Ở `-90.0`, trục z của `camera_optical_frame` (phía trước camera) phải chĩa xuống đất. Ở `0.0` nó chĩa về phía trước mũi UAV.

### Chạy unit test phần toán đổi frame:

```bash
cd ros2_ws
colcon test --packages-select px4_state_bridge --ctest-args -R frame_conversion_test
```

## 6. Xử lý sự cố thường gặp

1. **Không có `/odom`:**
   - Kiểm tra Micro XRCE-DDS Agent đang chạy và `ros2 topic list` có `/fmu/out/vehicle_odometry`.
   - Kiểm tra tên topic có khớp với `odometry_topic` (phụ thuộc phiên bản `px4_msgs`).
   - Kiểm tra log cảnh báo `pose_frame ... is not NED`.
2. **EKF của vision báo lỗi lookup TF hoặc extrapolation:**
   - Kiểm tra `ros2 run tf2_ros tf2_echo world camera_optical_frame` có trả kết quả liên tục.
   - Kiểm tra `use_sim_time` là `false` ở tất cả các node trên hardware.
   - Kiểm tra camera và máy chạy bridge dùng chung đồng hồ hệ thống.
3. **Camera nhìn lên thay vì xuống:**
   - Góc gimbal lệnh phải là số âm khi nhìn xuống. Kiểm tra dấu của `gimbal/target_angle_deg`.
4. **Hai nguồn cùng publish một frame TF:**
   - Nếu driver RealSense publish TF trùng tên với `camera_frame`, đổi `camera_frame` sang tên khác hoặc tắt `publish_tf` của driver.
   - Không đặt tên frame của bridge là `odom` hoặc `map`, vì `apf_planner` đã publish static TF `world` sang `map` và `world` sang `odom`.
5. **`alt_estimator/state` không được publish:**
   - Odometry đã cũ hơn `odom_timeout_sec`. Kiểm tra `ros2 topic hz /odom`.
6. **`touchdown_flag` không bao giờ lên `true`:**
   - Kiểm tra topic `land_detected_topic` có nhận được dữ liệu.
   - `landed` chỉ lên sau khi PX4 xác nhận đã hạ cánh. Bật `use_ground_contact` nếu cần tín hiệu sớm hơn.
7. **Vị trí H-Pad trong `world` bị lệch hoặc trôi:**
   - Kiểm tra `gimbal_pivot_xyz`, `camera_in_gimbal_xyz` và sai lệch giữa góc lệnh và góc thực của servo.