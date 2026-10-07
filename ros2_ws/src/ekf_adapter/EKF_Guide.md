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

Chế độ tracking (tên phải khớp danh sách mà `input_state_cache` chấp nhận):

| Mode | Điều kiện |
|---|---|
| TRACKING | Có measurement hợp lệ trong 0.25 s gần nhất |
| PREDICTING | Mất measurement, tuổi không quá một nửa giới hạn (APPROACH: 0.5 s, FOLLOW: 1.0 s) |
| PREDICTING_DEGRADED | Tuổi chưa quá giới hạn (APPROACH: 1.0 s, FOLLOW: 2.0 s) |
| EXPIRED | Quá giới hạn hoặc chưa từng có measurement |

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

Nếu mode luôn `EXPIRED` dù `/hpad/detected` bằng true: không có measurement nào qua được TF, gate hoặc kiểm tra z. Xem bộ đếm `accepted` và `tf_rejects` trong log thống kê.

Nếu mode nhảy `TRACKING` rồi `PREDICTING` liên tục: tần số `/hpad/position_camera` thấp hơn 4 Hz (tuổi vượt 0.25 s giữa hai frame), hoặc nhiều frame bị loại. Kiểm tra bằng `ros2 topic hz`.

Nếu vị trí lọc trễ so với marker khi mục tiêu chạy nhanh: tăng `process_accel_variance`, hoặc kiểm tra `max_target_speed` có nhỏ hơn vận tốc thật không (lõi kẹp vận tốc ngang theo giá trị này).

Nếu vị trí lọc rung hơn mong đợi: giảm `process_accel_variance` hoặc tăng nhiễu measurement (`lateral_per_m`, `depth_per_m2`). Nếu ngược lại, lọc quá chậm theo, giảm các hệ số nhiễu đó.

Nếu cần phát lại rosbag: đặt `use_sim_time:=true` ở launch và chạy `ros2 bag play --clock`. Bag phải chứa `/hpad/position_camera` và `/odom`, hoặc TF tương ứng.

## 5. Test

Test chia 3 tầng, nằm trong `test/` của package. Tầng 1 không cần ROS 2 chạy; tầng 2 và 3 cần source ROS 2 và workspace đã build.

Cài đặt:

```bash
python3 -m pip install pytest
sudo apt install ros-$ROS_DISTRO-ros2bag ros-$ROS_DISTRO-rosbag2-storage-default-plugins
```

`test/conftest.py` thêm thư mục gốc của package vào `sys.path` để `from ekf_adapter.ekf_logic import ...` hoạt động khi chạy `pytest` từ bất kỳ đâu. Tầng 3 gọi `ros2 run ekf_adapter odom_tf_node` và `ros2 run ekf_adapter ekf_node` nên package phải đã build và `source install/setup.bash`.

### Tầng 1: logic thuần

Không tạo node, chạy trong vài giây.

| File | Kiểm tra |
|---|---|
| `test_ekf_logic.py` | `TargetStateEKF` (tham số không hợp lệ, khởi tạo, kẹp vận tốc `v_max`/`vz_max`, `decay_velocity`, ma trận chuyển trạng thái và `process_covariance`, `predict` sai thứ tự, `update` chấp nhận và loại outlier, covariance luôn đối xứng xác định dương), hình học (quaternion sang ma trận, đổi điểm, ma trận R đối xứng, xác định dương, tăng theo độ sâu, dùng floor ở gần, xoay theo TF), `classify_tracking_mode` theo tuổi và phase, `TrackerConfig` kiểm tra tham số, `TargetTracker` (khởi tạo, hội tụ vận tốc, loại stamp lệch thứ tự, gate nở theo khoảng cách thời gian, loại outlier đơn, tái bắt sau 2 frame nhất quán và theo `reacquire_frames`, ước lượng vận tốc khi tái bắt, hết hạn candidate, NIS loại không reset tuổi, decay vận tốc theo `exp(-dt/tau)`, ngoại suy không làm đổi filter, phục hồi sau mất marker) |
| `test_ekf_simulation.py` | Chạy tracker trên 5 quỹ đạo của `target_state_simulation.py`: sai số lọc nhỏ hơn sai số đo thô, phần lớn outlier bị loại, track phục hồi sau mất marker, sai số khi mất marker không vượt 1 m |

```bash
cd ros2_ws/src/ekf_adapter
python3 -m pytest test/test_ekf_logic.py test/test_ekf_simulation.py -v
```

### Tầng 2: node ROS 2 với publisher giả

Tạo `EkfNode` hoặc `OdomTfNode` thật cùng một node helper trong cùng process. Helper phát TF (`world -> base_link` đúng stamp ảnh, static `base_link -> camera_optical_frame`), publish `/hpad/position_camera`, `/mission/phase`, `/odom` rồi kiểm tra topic đầu ra. `rclpy` khởi tạo một lần cho cả file, mỗi test tạo node mới.

| File | Kiểm tra |
|---|---|
| `test_ekf_node.py` | Tham số mặc định khớp dataclass, chưa có state và mode `EXPIRED` trước measurement đầu, measurement hợp lệ ra Odometry đúng `frame_id`/`child_frame_id` và mode `TRACKING`, dùng TF tại stamp ảnh (tịnh tiến xe cộng vào vị trí world), covariance và orientation của Odometry, loại vị trí NaN/Inf/`z <= 0`, loại thiếu stamp hoặc `frame_id`, `frame_id` lạ tăng `tf_reject_count`, `/mission/phase` đổi phase, mode hết hạn sau giới hạn APPROACH, tiếp tục publish state khi mất measurement; `odom_tf_node` (TF được chuẩn hóa quaternion, dùng đồng hồ node khi stamp bằng 0, bỏ qua quaternion zero-norm, nhận odom QoS BEST_EFFORT) |

```bash
python3 -m pytest test/test_ekf_node.py -v
```

### Tầng 3: rosbag

`test_ekf_rosbag.py` tự sinh một bag tổng hợp bằng `rosbag2_py`, chạy `static_transform_publisher` (identity `base_link -> camera_optical_frame`), `odom_tf_node` và `ekf_node` với `use_sim_time`, phát lại bag bằng `ros2 bag play --clock` rồi kiểm tra `/hpad/state_filtered` và `/ekf/tracking_mode`. Xe đứng yên tại gốc (odom 50 Hz), H-Pad bắt đầu tại (1.0, 0.5, 3.0) và đi theo +x với 0.5 m/s trong 8 s, measurement 15 Hz với nhiễu 0.02 m, mất measurement từ 3.0 s đến 6.0 s.

| Đoạn | Thời gian | Kỳ vọng |
|---|---|---|
| Có measurement | 0 - 3 s | Mode `TRACKING`, sai số vị trí dưới 0.15 m (từ 2 s), vận tốc x trung bình 0.5 m/s (sai số 0.2) |
| Mất measurement | 3 - 6 s | `PREDICTING_DEGRADED` trong khoảng 4.1 - 4.8 s, `EXPIRED` trong khoảng 5.2 - 6.0 s |
| Tái bắt | 6 - 8 s | Sai số vị trí dưới 0.15 m (từ 7 s), mode cuối là `TRACKING` |

Tổng số message state và mode phải trên 100, mọi mode thuộc 4 giá trị hợp lệ. Test chạy khoảng 15 s theo thời gian thật. Test tự bỏ qua nếu thiếu `rosbag2_py` hoặc lệnh `ros2`.

```bash
python3 -m pytest test/test_ekf_rosbag.py -v
```

Để thay bằng bag thật, ghi bag rồi sửa fixture `replay` (bỏ `static_transform_publisher` nếu bag đã có `/tf_static`, đổi đường dẫn bag) và chỉnh lại các giá trị kỳ vọng theo kịch bản đã ghi:

```bash
ros2 bag record -o sample_ekf /hpad/position_camera /odom /tf_static
```

### Chạy toàn bộ

```bash
colcon build --packages-select ekf_adapter
source install/setup.bash
colcon test --packages-select ekf_adapter --event-handlers console_direct+
colcon test-result --verbose
```

### Lưu ý khi chạy test

- Test tầng 2 và 3 dùng thời gian thật nên có thể chập chờn trên máy chậm; chạy lại một lần trước khi kết luận lỗi.
- Không chạy song song nhiều test ROS trong cùng `ROS_DOMAIN_ID`, vì các node dùng chung tên topic. Nếu cần chạy cạnh hệ thống đang chạy, đặt domain riêng: `ROS_DOMAIN_ID=77 python3 -m pytest ...`.
- Test rosbag chỉ chạy `odom_tf_node` và `ekf_node`. `aruco_node` không được chạy, measurement đến từ bag.
- Tên mode trong test lấy theo `tracking_policy.py` (`TRACKING`, `PREDICTING`, `PREDICTING_DEGRADED`, `EXPIRED`).
- Cú pháp `TopicMetadata` của `rosbag2_py` và tham số của `static_transform_publisher` có thể khác chút giữa các bản ROS 2 (Humble, Jazzy). Nếu lỗi, chỉnh `write_bag()` hoặc dòng `spawn(...)` tương ứng.
- `package.xml` cần `test_depend` cho `python3-pytest`, `ros2bag`, `rosbag2_py`, `rosbag2_storage_default_plugins`.

Nếu test tầng 2 báo không nhận được message: kiểm tra `ros2 topic list` trong cùng domain có node khác đang publish cùng topic không, và nhớ rằng `/odom` dùng QoS BEST_EFFORT nên publisher giả cũng phải là BEST_EFFORT hoặc RELIABLE.

Nếu test rosbag báo thiếu message: tăng thời gian khởi động (`spin_for(collector, 3.0)` trong fixture `replay`, node cần thời gian chạy trước khi bag phát), hoặc kiểm tra `colcon build` đã cài entry point `odom_tf_node` và `ekf_node` để `ros2 run ekf_adapter ...` chạy được.