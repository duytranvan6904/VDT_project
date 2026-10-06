# EKF Adapter — Guide

## 1. Package cần cài đặt ngoài

Không có binary nào phải build riêng. Cần các package ROS2 sau, thường đã có sẵn trong bản cài ROS2 desktop:

```bash
sudo apt install ros-$ROS_DISTRO-tf2-ros ros-$ROS_DISTRO-nav-msgs \
  ros-$ROS_DISTRO-geometry-msgs ros-$ROS_DISTRO-launch-ros
python3 -m pip install numpy
```

Điều kiện đầu vào:
- `aruco_node` đang chạy và publish `/hpad/position_camera` (có `header.stamp` hợp lệ và `frame_id`).
- Có TF `world -> base_link` (từ `odom_tf_node` hoặc nguồn khác) và TF `base_link -> camera_optical_frame` (static).
- Stamp của ảnh, odom và TF cùng một đồng hồ (cùng chế độ thời gian thật hoặc sim time).

## 2. Nguyên lý hoạt động

```
/odom (Odometry)
   |
   v
odom_tf_node --> TF world -> base_link
                                          static TF base_link -> camera_optical_frame
                                                     |
/hpad/position_camera (PointStamped, camera frame)   |
   |                                                 |
   v                                                 v
ekf_node: lookup TF tại stamp ảnh --> điểm trong world + R theo từng measurement
   |
   v
TargetTracker (gate, tái bắt, decay) --> TargetStateEKF (6 trạng thái, CV)
   |
   v  timer 50 Hz, ngoại suy tới "now" (không sửa filter)
/hpad/state_filtered (Odometry, world)
/ekf/tracking_mode (String)
```

Trạng thái lọc: `[x, y, z, vx, vy, vz]` trong hệ world, mô hình vận tốc không đổi, nhiễu gia tốc trắng.

Mỗi measurement (`position_callback`):
1. Loại nếu thiếu stamp, thiếu `frame_id`, giá trị không hữu hạn hoặc `z <= 0`.
2. Tra TF `world <- frame_id` đúng tại stamp ảnh (không dùng TF mới nhất). Lỗi TF thì bỏ measurement và tăng `tf_rejects`.
3. Đổi điểm sang world, tính ma trận nhiễu R:
   - Hệ camera: sai số ngang `max(floor, k1*z)`, sai số theo trục sâu `max(floor, k2*z^2)`.
   - Xoay sang world bằng rotation của TF, cộng nhiễu đẳng hướng do sai attitude (`attitude_std_rad * khoảng cách`) và sai vị trí xe.
4. `TargetTracker.process_measurement`:
   - Frame đầu: khởi tạo track (`initialized`).
   - Stamp cũ hơn thời gian filter: loại (`rejected_order`).
   - Đưa filter tới stamp (có giảm vận tốc nếu mất measurement lâu), rồi kiểm tra gate động theo khoảng cách ngang. Gate nở ra theo thời gian từ lần measurement hợp lệ cuối, kẹp trong `[gate_min_m, gate_max_m]`.
   - Qua gate thì gọi `update()` (có thêm NIS gate trong lõi). Chỉ khi `update()` chấp nhận mới tính là hợp lệ (`accepted`) và reset tuổi.
   - Ngoài gate hoặc bị NIS loại: lưu làm candidate. Đủ `reacquire_frames` frame liên tiếp nhất quán trong `candidate_window_s` thì khởi tạo lại track (`reacquired`). Vận tốc ban đầu chỉ lấy từ chênh lệch vị trí nếu khoảng thời gian đủ `velocity_baseline_s`, ngược lại đặt 0.

Mỗi chu kỳ timer (`output_rate_hz`):
1. Tính tuổi measurement hợp lệ cuối, phân loại mode bằng `classify_tracking_mode`, publish `/ekf/tracking_mode`.
2. Nếu track đã khởi tạo, ngoại suy một bản sao filter tới thời gian hiện tại và publish Odometry. Filter gốc không bị đổi, nên frame ảnh trễ vẫn xử lý đúng thứ tự.

Chế độ tracking:

| Mode | Điều kiện |
|---|---|
| TRACKING | Có measurement hợp lệ trong 0.25 s gần nhất |
| COASTING | Mất measurement nhưng tuổi chưa quá giới hạn (APPROACH: 1.0 s, FOLLOW: 2.0 s) |
| LOST | Quá giới hạn hoặc chưa từng có measurement |

Lưu ý:
- Mất measurement quá `decay_after_s`, vận tốc giảm theo `exp(-dt/decay_tau_s)` để không trôi vô hạn khi mục tiêu đã dừng ngoài tầm nhìn.
- Odometry chỉ có vị trí và vận tốc. Orientation không được ước lượng (w = 1, phương sai 1e6). Vận tốc nằm trong hệ world, không phải `child_frame_id`.
- Pose camera đến từ `aruco_node` đã lọc theo `min_detection_distance_m`, adapter không lọc khoảng cách thêm lần nữa.

## 3. Cách chạy

Chạy bằng launch (khuyến nghị):

```bash
colcon build --packages-select ekf_adapter
source install/setup.bash
ros2 launch ekf_adapter ekf.launch.py \
  cam_x:=0.0 cam_y:=0.0 cam_z:=-0.05 \
  cam_roll:=-1.5708 cam_pitch:=0.0 cam_yaw:=-1.5708
```

Giá trị mặc định của `cam_*` là camera nhìn thẳng về phía trước. Phải sửa theo vị trí lắp thật. Sai extrinsic thì vị trí world sai mà không có cảnh báo.

Tham số launch:

| Tên | Mặc định | Ý nghĩa |
|---|---|---|
| `use_sim_time` | false | Dùng thời gian mô phỏng hoặc rosbag |
| `publish_odom_tf` | true | Chạy `odom_tf_node`. Đặt false nếu đã có TF `world -> base_link` |
| `publish_camera_tf` | true | Publish static TF của camera. Đặt false nếu đã có |
| `odom_topic` | /odom | Topic `nav_msgs/Odometry` của xe |
| `world_frame` | world | Frame đích |
| `base_frame` | base_link | Frame thân xe |
| `camera_frame` | camera_optical_frame | Phải trùng `camera_frame_id` của `aruco_node` |
| `cam_x/y/z`, `cam_roll/pitch/yaw` | 0, 0, 0, -1.5708, 0, -1.5708 | Extrinsic camera so với `base_frame` (m, rad) |

Chạy riêng từng node:

```bash
ros2 run ekf_adapter odom_tf_node
ros2 run ekf_adapter ekf_node --ros-args -p max_target_speed:=2.0
```

Tham số chính của `ekf_node`:

| Tên | Mặc định | Ý nghĩa |
|---|---|---|
| `process_accel_variance` | [1.0, 1.0, 0.5] | Mật độ nhiễu gia tốc theo x, y, z |
| `gate_threshold` | 16.27 | Ngưỡng NIS (chi-square 3 bậc tự do, 99.9%) |
| `max_target_speed` | 2.5 | Giới hạn vận tốc ngang (m/s) |
| `max_target_vz` | 1.5 | Giới hạn vận tốc đứng (m/s) |
| `target_frame` | world | Frame của trạng thái lọc |
| `child_frame` | hpad | `child_frame_id` của Odometry |
| `tf_timeout_s` | 0.03 | Thời gian chờ TF theo stamp ảnh |
| `output_rate_hz` | 50.0 | Tần số publish trạng thái và mode |
| `position_topic` | /hpad/position_camera | Nguồn measurement |
| `phase_topic` | /mission/phase | Nhận APPROACH hoặc FOLLOW, giá trị khác coi như FOLLOW |
| `state_topic` | /hpad/state_filtered | Đầu ra Odometry |
| `mode_topic` | /ekf/tracking_mode | Đầu ra mode |

Tham số gate và tái bắt (`TrackerConfig`):

| Tên | Mặc định | Ý nghĩa |
|---|---|---|
| `gate_accel_mps2` | 2.0 | Gia tốc dùng tính ngân sách gate |
| `gate_speed_fraction` | 0.3 | Hệ số vận tốc trong ngân sách gate |
| `gate_margin_m` | 0.4 | Biên cố định cộng vào gate |
| `gate_min_m` / `gate_max_m` | 0.6 / 2.0 | Cận dưới và cận trên của gate (m) |
| `gate_min_gap_s` / `gate_max_gap_s` | 0.02 / 1.5 | Kẹp khoảng thời gian từ measurement hợp lệ cuối |
| `reacquire_frames` | 2 | Số frame nhất quán để tái bắt (tối thiểu 2) |
| `candidate_window_s` | 1.0 | Cửa sổ thời gian giữa các frame candidate |
| `candidate_step_min_m` / `candidate_step_margin_m` | 0.5 / 0.3 | Bước nhảy tối đa giữa hai candidate |
| `velocity_baseline_s` | 0.3 | Khoảng thời gian tối thiểu để ước lượng vận tốc khi tái bắt |
| `decay_after_s` | 1.0 | Bắt đầu giảm vận tốc sau khi mất measurement bấy lâu |
| `decay_tau_s` | 1.3 | Hằng số thời gian giảm vận tốc |
| `out_of_order_tol_s` | 0.001 | Dung sai stamp lệch thứ tự |
| `detected_timeout_s` | 0.25 | Ngưỡng tuổi để coi là đang thấy marker |

Tham số nhiễu measurement (`NoiseConfig`), cần hiệu chỉnh bằng dữ liệu thật:

| Tên | Mặc định | Ý nghĩa |
|---|---|---|
| `lateral_floor_m` / `lateral_per_m` | 0.02 / 0.003 | Sai số ngang: `max(floor, per_m * z)` |
| `depth_floor_m` / `depth_per_m2` | 0.02 / 0.01 | Sai số trục sâu: `max(floor, per_m2 * z^2)` |
| `attitude_std_rad` | 0.02 | Sai số attitude của xe |
| `vehicle_position_std_m` | 0.05 | Sai số vị trí xe |

Topic đầu vào và đầu ra:

| Topic | Kiểu | Hướng |
|---|---|---|
| `/hpad/position_camera` | `PointStamped` | vào |
| `/mission/phase` | `String` | vào |
| `/odom` | `Odometry` | vào (`odom_tf_node`) |
| `/hpad/state_filtered` | `Odometry` | ra, có covariance vị trí và vận tốc |
| `/ekf/tracking_mode` | `String` | ra |

## 4. Cách debug

Log thống kê mỗi 2 giây:

```
[INFO] [ekf_node]: mode=TRACKING age=0.03s pos=(10.69, 0.00, 3.00) vel=(0.50, 0.00, 0.00) tf_rejects=0 {'initialized': 1, 'accepted': 39}
```

Các bộ đếm: `accepted` (đã fuse), `rejected_gate` (ngoài gate hoặc bị NIS loại), `rejected_order` (stamp lệch thứ tự), `reacquired` (khởi tạo lại track).

Kiểm tra đầu ra:

```bash
ros2 topic echo /ekf/tracking_mode
ros2 topic echo /hpad/state_filtered --field pose.pose.position
ros2 topic hz /hpad/state_filtered
```

Kiểm tra cây TF và phép đổi camera sang world:

```bash
ros2 run tf2_ros tf2_echo world camera_optical_frame
ros2 run tf2_tools view_frames
```

Kiểm tra stamp của measurement và odom có cùng đồng hồ không:

```bash
ros2 topic echo /hpad/position_camera --field header.stamp --once
ros2 topic echo /odom --field header.stamp --once
```

Nếu `tf_rejects` tăng đều:
- Lỗi `ExtrapolationException`: odom đến muộn hơn stamp ảnh hoặc stamp hai nguồn lệch đồng hồ. Tăng `tf_timeout_s` (ví dụ 0.05) hoặc kiểm tra `use_sim_time`.
- Lỗi `LookupException`: thiếu frame. Kiểm tra bằng `view_frames`, và `camera_frame` phải trùng `frame_id` trong message của `aruco_node`.

Nếu log báo `measurement without stamp or frame_id rejected`: `aruco_node` đang publish stamp bằng 0 hoặc `camera_frame_id` rỗng.

Nếu `rejected_gate` tăng liên tục và track nhảy qua lại: thường do extrinsic camera sai, hoặc `/odom` thiếu chuẩn (sai hệ NED hay ENU). Quan sát `world` trong log khi xe đứng yên và marker đứng yên, giá trị phải ổn định. Nếu chỉ lệch khi xe xoay, nghi ngờ rotation của extrinsic.

Nếu mode luôn `LOST` dù `/hpad/detected` bằng true: không có measurement nào qua được TF, gate hoặc kiểm tra z. Xem bộ đếm `accepted` và `tf_rejects` trong log thống kê.

Nếu mode nhảy `TRACKING` rồi `COASTING` liên tục: tần số `/hpad/position_camera` thấp hơn 4 Hz (tuổi vượt 0.25 s giữa hai frame), hoặc nhiều frame bị loại. Kiểm tra bằng `ros2 topic hz`.

Nếu vị trí lọc trễ so với marker khi mục tiêu chạy nhanh: tăng `process_accel_variance`, hoặc kiểm tra `max_target_speed` có nhỏ hơn vận tốc thật không (lõi kẹp vận tốc ngang theo giá trị này).

Nếu vị trí lọc rung hơn mong đợi: giảm `process_accel_variance` hoặc tăng nhiễu measurement (`lateral_per_m`, `depth_per_m2`). Nếu ngược lại, lọc quá chậm theo, giảm các hệ số nhiễu đó.

Nếu cần phát lại rosbag: đặt `use_sim_time:=true` ở launch và chạy `ros2 bag play --clock`. Bag phải chứa `/hpad/position_camera` và `/odom`, hoặc TF tương ứng.

## Unit test

Test chạy trên dữ liệu tổng hợp, không cần ROS2 hay phần cứng:

```bash
python3 -m pip install pytest
cd ros2_ws/src/ekf_adapter
python3 -m pytest test/test_ekf_logic.py
```

Nếu import lỗi khi chạy từ thư mục khác:

```bash
PYTHONPATH=ros2_ws/src/ekf_adapter python3 -m pytest ros2_ws/src/ekf_adapter/test/test_ekf_logic.py
```

Nội dung được kiểm tra:
- Lõi: `decay_velocity`, `vz_max`, kiểm tra tham số không hợp lệ.
- Hình học: quaternion sang ma trận, đổi điểm, ma trận R (đối xứng, xác định dương, tăng theo độ sâu, xoay theo TF).
- Chính sách mode theo tuổi measurement và phase.
- Tracker: khởi tạo, hội tụ vận tốc, loại stamp lệch thứ tự, loại outlier đơn, tái bắt sau 2 frame nhất quán, hết hạn candidate, NIS loại không reset tuổi, decay vận tốc, ngoại suy không làm đổi filter, phục hồi sau mất marker.
- Tích hợp với `target_state_simulation.py`: trên 5 quỹ đạo, sai số lọc nhỏ hơn sai số đo thô, outlier phần lớn bị loại, track phục hồi sau mất marker.

Hai node ROS (`ekf_node.py`, `odom_tf_node.py`) không nằm trong unit test. Kiểm tra chúng bằng cách chạy với rosbag hoặc mô phỏng và quan sát các bộ đếm trong log thống kê.