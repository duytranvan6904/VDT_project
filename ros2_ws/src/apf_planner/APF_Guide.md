# APF Planner — Guide

## 1. Package cần cài đặt ngoài

Chỉ cần `numpy`, không có binary ngoài nào phải build riêng.

```bash
python3 -m pip install numpy
```

Package ROS2 cần có sẵn: `rclpy`, `geometry_msgs`, `nav_msgs`, `sensor_msgs`, `std_msgs`, `visualization_msgs`, `tf2_ros_py` (chỉ `pointcloud_generator` dùng), `launch_ros` và `vdt_msgs` (message `PlannerOutput`, phải build trước).

## 2. Nguyên lý hoạt động

```
/odom, /hpad/state_filtered, /ekf/tracking_mode, /mission/phase
/map_generator/global_cloud (pointcloud_generator)
        |
        v
   planner_node  --> /apf/velocity_cmd, /apf/yaw_cmd, /apf/force_markers
        |
        v
 planner_merge_node <-- /ibvs/yaw_cmd, /mission/phase
        |
        v
 planner/velocity_setpoint --> offboard_manager, input_state_cache
```

Lúc khởi động `planner_node`:
1. Khai báo tham số, tạo lõi theo `planner_type`: `apf` (`APFCore`, lực 2D ngang, độ cao điều khiển P riêng) hoặc `iapf` (`IAPFCore`, 3D). Giá trị khác `apf`/`iapf` rơi về `apf`. Hai lõi là Python thuần, không phụ thuộc ROS.
2. Nếu `obstacle_source` là `sdf` hoặc `both`, đọc cylinder từ `world_sdf` (chỉ nhận model có tên bắt đầu bằng `cyl_` hoặc `cylinder_obs_`).
3. Tạo subscriber, publisher và timer `rate_hz` (mặc định 30 Hz). `/odom` dùng QoS BEST_EFFORT; các topic còn lại dùng QoS mặc định. Subscriber point cloud chỉ có khi `obstacle_source` là `pointcloud` hoặc `both`.

Mỗi chu kỳ (`tick`):
1. Chỉ tính lệnh khi phase là `FOLLOW`/`APPROACH`, `tracking_mode` thuộc `allowed_tracking_modes` (`TRACKING`, `PREDICTING`), và odom, target cùng tươi trong `data_timeout_sec` (tuổi tính từ lúc node nhận message, không dùng stamp trong header). Ngược lại publish `Twist` bằng 0.
2. Tính goal: APPROACH là vị trí H-Pad (gồm cả z); FOLLOW là điểm stand-off cách H-Pad `follow_distance` (khoảng cách chéo 3D), giữ độ cao `target_altitude`. Ở FOLLOW, vận tốc target được lọc thông thấp (`target_velocity_alpha`, deadband `target_velocity_deadband`); khi tốc độ ngang >= 0.18 m/s thì goal được lead thêm `vận tốc × target_lead_time`, giới hạn bởi `max_target_lead`.
3. Chọn vật cản: mỗi cylinder SDF đóng góp một điểm gần nhất (không bị giới hạn bởi `max_cloud_points`). Point cloud được voxel downsample khi nhận message; mỗi chu kỳ lấy điểm trong bán kính `cloud_query_radius` (mặc định `2·d0` khi tham số bằng 0), mỗi cụm một điểm gần nhất, tối đa `max_cloud_points`.
4. Gọi lõi với `target_pos` là vị trí H-Pad để khóa yaw (APPROACH dùng `k_rep_approach`). Ở FOLLOW, nếu chưa tới goal và target di chuyển ngang >= 0.20 m/s thì cộng feedforward vận tốc ngang của target, kẹp theo `v_max`.
5. Publish `/apf/velocity_cmd`, `/apf/yaw_cmd` (hướng ENU từ UAV tới H-Pad) và marker RViz.

Khi phase đổi, `planner_node` gọi `reset()` của lõi (I-APF xóa trạng thái tiếp tuyến và hướng trước đó).

Hai lõi:
- `apf`: chỉ tính lực ngang, z của vật cản bị bỏ qua. Bán kính ảnh hưởng hiệu dụng là `min(d0, max(0.4, khoảng cách ngang tới goal))`. Mỗi vật cản thêm lực tiếp tuyến bằng 0.8 lần lực đẩy. Tốc độ giảm tuyến tính từ `d_slow` xuống `goal_threshold`. `vz = k_z × sai số độ cao` với deadband 8 cm, kẹp `vz_max`.
- `iapf`: lực 3D, lực đẩy kiểu GNRON có trọng số sigmoid theo khoảng cách tới goal, thoát cực tiểu cục bộ bằng lực tiếp tuyến chọn theo điểm số (hướng goal, khoảng trống phía trước, liên tục với hướng trước), làm mượt hướng khi gần vật cản. Tốc độ = `v_max` × hệ số goal (`d_goal/d0`, sàn 0.15) × hệ số vật cản (sàn 0.25), tối thiểu `0.35·v_max` khi đang thoát kẹt. `vz` kẹp `vz_max`.

`planner_merge_node` (20 Hz):
- FOLLOW/APPROACH: chuyển tiếp vận tốc APF; yaw ưu tiên IBVS, dự phòng APF, cuối cùng NaN (giữ yaw), tùy `yaw_source`. Vận tốc non-finite thì publish 0 và yaw NaN.
- Phase khác, hoặc phase quá cũ (quá `phase_timeout_sec` không nhận `/mission/phase`, coi như `IDLE`): publish vận tốc 0, yaw NaN.
- Mất `/apf/velocity_cmd` (hoặc chưa từng nhận) quá `upstream_timeout_sec`: publish vận tốc 0 trong thêm `stale_hover_sec`, sau đó ngừng publish để FSM thấy `planner_timeout`. Cơ chế này chỉ áp dụng ở FOLLOW/APPROACH.

Lưu ý:
- `ekf_node` publish liên tục kể cả khi `EXPIRED`, nên cổng theo mode là bắt buộc, không thể chỉ dựa vào độ tươi của timestamp.
- `planner_merge_node` phải publish đều ở mọi phase. Nếu chỉ publish trong FOLLOW/APPROACH, `input_state_cache` sẽ báo `planner_timeout` ngay khi vào FOLLOW và FSM quay về SEARCH.
- Message có `frame_id` khác `world_frame` hoặc giá trị non-finite bị bỏ (`frame_id` rỗng vẫn được chấp nhận); kết quả có NaN thì publish vận tốc 0 (không publish yaw và marker chu kỳ đó).
- `/apf/yaw_cmd` bằng 0 rad (không phải NaN) khi H-Pad cách UAV dưới 0.1 m theo phương ngang.
- `apf` đơn giản và dễ kẹt cực tiểu cục bộ; `iapf` thêm lực đẩy GNRON, lực tiếp tuyến 3D để thoát kẹt (hysteresis `iapf_f_enter`/`iapf_f_exit`) và chống dao động hướng.
- Lõi I-APF chạy vòng lặp Python, chi phí tăng theo `max_cloud_points × iapf_n_tangent × iapf_n_pred`.
- Khi dùng point cloud, `k_rep` cần hiệu chỉnh lại so với cylinder (SDF).
- `apf_pointcloud_generator` sinh point cloud một lần lúc khởi động rồi publish lại mỗi `1/rate_hz` giây với stamp mới. Các cột bị bỏ nếu nằm trong `clear_radius` quanh gốc tọa độ, nên số cột thực tế có thể ít hơn `num_obs`. Nếu `publish_static_tf` bật, node publish thêm TF tĩnh `frame_id` -> `map` (đồng nhất).

## 3. Cách chạy

```bash
colcon build --packages-select apf_planner
source install/setup.bash
ros2 launch apf_planner apf_planner.launch.py planner_type:=iapf
```

Thử với point cloud vật cản giả lập:

```bash
ros2 launch apf_planner apf_planner.launch.py generator:=true planner_type:=iapf
```

Launch arguments: `params_file`, `generator` (mặc định `false`), `planner_type` (mặc định `iapf`). Lưu ý giá trị mặc định của tham số `planner_type` khi chạy `planner_node` trực tiếp (không qua launch) là `apf`.

Tham số chính của `planner_node` (đầy đủ trong `config/apf_params.yaml`):

| Tên | Mặc định | Ý nghĩa |
|---|---|---|
| `planner_type` | `apf` (launch: `iapf`) | `apf` hoặc `iapf` |
| `obstacle_source` | `pointcloud` | `pointcloud`, `sdf` hoặc `both` |
| `world_sdf` | rỗng | Đường dẫn SDF khi dùng `sdf`/`both` |
| `world_frame` | `world` | Frame bắt buộc của odom, target, cloud |
| `rate_hz` | 30 | Tần số tick |
| `target_topic` | `/hpad/state_filtered` | Trạng thái H-Pad từ EKF |
| `allowed_tracking_modes` | `[TRACKING, PREDICTING]` | Mode cho phép tính lệnh |
| `data_timeout_sec` | 0.5 | Tuổi tối đa của odom/target |
| `d0` | 2.0 | Bán kính ảnh hưởng vật cản (m) |
| `v_max` | 1.2 | Tốc độ tối đa (m/s) |
| `d_slow` | 1.5 | Khoảng cách bắt đầu giảm tốc tới goal (m), chỉ `apf` |
| `k_att` / `k_rep` / `k_rep_approach` | 10 / 250 / 125 | Gain lực hút, lực đẩy, lực đẩy ở APPROACH |
| `goal_threshold` | 0.20 | Ngưỡng tới goal (m) |
| `follow_distance` | 3.5 | Khoảng cách chéo 3D tới H-Pad ở FOLLOW (m) |
| `hold_follow_altitude` / `target_altitude` | true / 3.0 | Giữ độ cao FOLLOW (m, ENU) |
| `k_z` | 0.6 | Gain P độ cao (1/s), chỉ `apf` |
| `vz_max` | 0.5 | Vận tốc đứng tối đa (m/s) |
| `target_lead_time` / `max_target_lead` | 0.25 / 0.75 | Thời gian lead và giới hạn lead của goal (s / m) |
| `target_velocity_alpha` | 0.25 | Hệ số lọc vận tốc target |
| `target_velocity_deadband` | 0.08 | Bỏ vận tốc target nhỏ hơn giá trị này (m/s) |
| `cloud_voxel_size` | 0.3 | Voxel downsample (m) |
| `cloud_query_radius` | 0.0 | Bán kính lấy điểm vật cản (m), 0 nghĩa là `2·d0` |
| `cloud_cluster_radius` | 1.2 | Bán kính loại điểm trùng cụm (m) |
| `max_cloud_points` | 8 | Số điểm vật cản tối đa mỗi chu kỳ |
| `iapf_f_enter` / `iapf_f_exit` | 0.10 / 0.30 | Ngưỡng vào/ra chế độ tiếp tuyến |
| `iapf_k_tan` | 1.0 | Độ lớn lực tiếp tuyến |
| `iapf_n_tangent` / `iapf_n_pred` | 12 / 3 | Số hướng tiếp tuyến thử / số bước dự đoán |
| `iapf_w_goal` / `iapf_w_clear` / `iapf_w_prev` | 1.0 / 1.0 / 0.60 | Trọng số điểm số hướng tiếp tuyến |

Tham số của `planner_merge`:

| Tên | Mặc định | Ý nghĩa |
|---|---|---|
| `rate_hz` | 20 | Tần số publish |
| `yaw_source` | `ibvs_apf` | `ibvs_apf`, `ibvs`, `apf` hoặc `hold` (giá trị lạ rơi về `ibvs_apf`) |
| `upstream_timeout_sec` | 0.3 | Tuổi tối đa của vận tốc APF |
| `stale_hover_sec` | 0.5 | Thời gian hover trước khi ngừng publish |
| `yaw_timeout_sec` | 0.3 | Tuổi tối đa của nguồn yaw |
| `phase_timeout_sec` | 1.0 | Quá thời gian này không nhận phase thì coi là `IDLE` |
| `velocity_topic` / `apf_yaw_topic` / `ibvs_yaw_topic` / `phase_topic` | `/apf/velocity_cmd` / `/apf/yaw_cmd` / `/ibvs/yaw_cmd` / `/mission/phase` | Topic đầu vào |
| `output_topic` | `planner/velocity_setpoint` | Topic đầu ra |

Tham số của `apf_pointcloud_generator`: `topic=/map_generator/global_cloud`, `frame_id=world`, `num_obs=35`, `map_size=25.0`, `height=4.0`, `resolution=0.15`, `clear_radius=2.0`, `seed=-1` (>= 0 để tái lập), `rate_hz=1.0`, `publish_static_tf=true`.

Topic đầu vào của `planner_node`:

| Topic | Kiểu | Nội dung |
|---|---|---|
| `/odom` | `nav_msgs/Odometry` | UAV, frame `world`, BEST_EFFORT |
| `/hpad/state_filtered` | `nav_msgs/Odometry` | H-Pad (vị trí và `twist.linear`), frame `world` |
| `/ekf/tracking_mode` | `std_msgs/String` | Mode tracking của EKF |
| `/mission/phase` | `std_msgs/String` | Phase hiện tại |
| `/map_generator/global_cloud` | `sensor_msgs/PointCloud2` | Vật cản, frame `world`, chỉ khi dùng `pointcloud`/`both` |

Topic đầu ra:

| Topic | Kiểu | Nội dung |
|---|---|---|
| `/apf/velocity_cmd` | `Twist` | Vận tốc ENU (m/s), publish mỗi tick (bằng 0 khi không tính lệnh) |
| `/apf/yaw_cmd` | `Float64` | Heading ENU (rad), chỉ publish khi đang tính lệnh |
| `/apf/force_markers` | `MarkerArray` | Debug RViz, publish mỗi tick |
| `planner/velocity_setpoint` | `vdt_msgs/PlannerOutput` | Lệnh đã ghép (`vx`, `vy`, `vz`, `yaw`), ENU, yaw NaN nghĩa là giữ yaw (từ `planner_merge_node`) |

## 4. Cách debug

Kiểm tra nhịp topic:

```bash
ros2 topic hz /apf/velocity_cmd
ros2 topic hz planner/velocity_setpoint
ros2 topic echo /apf/velocity_cmd
```

`planner/velocity_setpoint` phải ổn định khoảng 20 Hz ở mọi phase. Nếu không, FSM sẽ thấy `planner_timeout`.

Xem marker trong RViz2: Fixed Frame `world`, thêm `MarkerArray` `/apf/force_markers` và `PointCloud2` `/map_generator/global_cloud`. Mũi tên xanh lá là lực hút, đỏ là lực đẩy, xanh dương là vận tốc lệnh, tím là lực tiếp tuyến (chỉ khi I-APF đang thoát cực tiểu), độ dài mũi tên tối đa 2 m; cầu cyan là UAV, cầu vàng là goal, chấm đỏ là điểm vật cản đang dùng, hình trụ cam là cylinder từ SDF.

Test nhanh không cần PX4 (UAV đứng yên nên chỉ kiểm tra lệnh và marker):

```bash
ros2 topic pub -r 20 /odom nav_msgs/msg/Odometry "{header: {frame_id: world}, pose: {pose: {position: {x: -8.0, y: -8.0, z: 3.0}}}}"
ros2 topic pub -r 20 /hpad/state_filtered nav_msgs/msg/Odometry "{header: {frame_id: world}, pose: {pose: {position: {x: 8.0, y: 8.0, z: 0.0}}}}"
ros2 topic pub -r 10 /ekf/tracking_mode std_msgs/msg/String "{data: TRACKING}"
ros2 topic pub -r 10 /mission/phase std_msgs/msg/String "{data: FOLLOW}"
```

Nếu `/apf/velocity_cmd` luôn bằng 0: kiểm tra lần lượt phase có là `FOLLOW`/`APPROACH`, `tracking_mode` có thuộc `allowed_tracking_modes` (`PREDICTING_DEGRADED` và `EXPIRED` bị chặn), odom và target có đang đến trong `data_timeout_sec`, và `frame_id` có đúng `world` (sai frame sẽ có cảnh báo `Ignoring frame`). UAV đã tới goal (cách goal dưới `goal_threshold`) cũng cho vận tốc bằng 0.

Nếu `/apf/yaw_cmd` không có message: planner chỉ publish yaw khi đang tính lệnh; ngoài FOLLOW/APPROACH thì không có. Nếu yaw bằng 0 rad khi UAV ở sát H-Pad: đó là hành vi mặc định khi khoảng cách ngang dưới 0.1 m.

Nếu UAV không né vật cản: kiểm tra `obstacle_source` khớp nguồn đang có, `/map_generator/global_cloud` có message và frame là `world`, và `world_sdf` đúng đường dẫn khi dùng `sdf` (log khởi động in `Loaded N SDF cylinders`; nếu N bằng 0 thì kiểm tra tên model có bắt đầu bằng `cyl_` hoặc `cylinder_obs_` và có `<cylinder>` và `<pose>`). Chấm đỏ trên RViz cho biết điểm vật cản đang được dùng.

Nếu lực đẩy quá mạnh khi dùng point cloud: giảm `max_cloud_points` hoặc `k_rep`, hoặc tăng `cloud_cluster_radius`.

Nếu UAV kẹt giữa vật cản (I-APF): tăng `iapf_f_enter` hoặc `iapf_k_tan`. Nếu đổi hướng liên tục thì tăng `iapf_w_prev`.

Nếu rung quanh goal: tăng `goal_threshold` (hoặc `d_slow` khi dùng `apf`).

Nếu `planner/velocity_setpoint` im lặng trong FOLLOW/APPROACH: `planner_merge_node` đã ngừng publish vì mất `/apf/velocity_cmd` (hoặc chưa từng nhận) quá `upstream_timeout_sec + stale_hover_sec`; kiểm tra `planner_node` còn chạy không.

Nếu `planner/velocity_setpoint` luôn bằng 0 dù đang FOLLOW: kiểm tra `/mission/phase` có được publish đều (quá `phase_timeout_sec` không nhận thì merge coi là `IDLE`).

Nếu yaw luôn là NaN (UAV không xoay): kiểm tra `/ibvs/yaw_cmd` và `/apf/yaw_cmd` có đến trong `yaw_timeout_sec`, và `yaw_source` có phải `hold` không.

## 5. Test

Test chia 3 tầng, nằm trong `test/` của package. Tầng 1 không cần ROS 2 chạy; tầng 2 và 3 cần source ROS 2 và workspace đã build.

Cài đặt:

```bash
python3 -m pip install pytest
sudo apt install ros-$ROS_DISTRO-launch-pytest ros-$ROS_DISTRO-ros2bag ros-$ROS_DISTRO-rosbag2-storage-default-plugins
```

`test/` không có `__init__.py` để `from ros_utils import ...` hoạt động. `setup.py` cần có `data_files` cài thư mục `launch/` vì test rosbag lấy `launch/apf_planner.launch.py` qua `get_package_share_directory`.

### Tầng 1: logic thuần

Không tạo node, chạy trong vài giây.

| File | Kiểm tra |
|---|---|
| `test_apf_core.py` | `make_follow_goal` (stand-off 3D/2D, hold altitude, drone ngay trên target, sai shape), `APFCore` (tốc độ tối đa, đường giảm tốc, né vật cản, lực tiếp tuyến, bán kính ảnh hưởng co lại gần goal, `k_rep_override`, điều khiển độ cao có deadband, khóa yaw theo target) |
| `test_iapf_core.py` | Dừng và reset khi tới goal, lịch tốc độ theo goal và vật cản, kẹp `vz`, vào/giữ/thoát chế độ tiếp tuyến (hysteresis), tốc độ thoát kẹt tối thiểu, tiếp tuyến vuông góc pháp tuyến, làm mượt hướng gần vật cản, yaw, kết quả hữu hạn khi vật cản trùng vị trí UAV |
| `test_obstacles.py` | Đọc cylinder từ SDF (tiền tố tên, giá trị mặc định, model thiếu pose), điểm gần nhất trên cylinder, `cloud_to_xyz` (offset field, big-endian, NaN, buffer ngắn), voxel downsample, chọn điểm vật cản (bán kính, cụm, thứ tự, giới hạn số điểm) |
| `test_markers.py` | Số lượng và `ns`/`id` marker, mũi tên tiếp tuyến chỉ khi I-APF đang thoát kẹt, độ dài mũi tên tối đa 2 m, cylinder và điểm vật cản |
| `test_pointcloud_forest.py` | `generate_forest` tái lập theo seed, kiểu dữ liệu, giới hạn không gian, vùng `clear_radius`, mật độ theo `resolution` |

```bash
cd ros2_ws/src/apf_planner
python3 -m pytest test/test_apf_core.py test/test_iapf_core.py test/test_obstacles.py test/test_markers.py test/test_pointcloud_forest.py -v
```

### Tầng 2: node ROS 2 với publisher giả

Tạo node thật cùng một node probe trong cùng process, publish dữ liệu giả vào topic đầu vào rồi kiểm tra topic đầu ra. Tham số được truyền qua `--ros-args -p` nên mỗi test khởi tạo `rclpy` riêng (fixture `make_rig` trong `conftest.py`).

| File | Kiểm tra |
|---|---|
| `test_planner_node_ros.py` | Vận tốc bằng 0 khi phase không phải FOLLOW/APPROACH, chặn `EXPIRED`/`PREDICTING_DEGRADED`/`LOST`, cho phép `TRACKING`/`PREDICTING`, odom quá hạn, sai `frame_id`, vị trí NaN, hướng chuyển động và yaw ở FOLLOW, hạ độ cao ở APPROACH, giữ độ cao FOLLOW, né vật cản từ point cloud và từ SDF, feedforward vận tốc target, marker, bản `iapf`, tần số publish khi không hoạt động |
| `test_planner_merge_ros.py` | Publish đều ở mọi phase, chuyển tiếp vận tốc ở FOLLOW/APPROACH, ưu tiên yaw IBVS rồi APF rồi NaN, các giá trị `yaw_source`, yaw quá hạn, vận tốc NaN, phase quá hạn coi là `IDLE`, hover rồi im lặng khi mất upstream, phục hồi khi upstream trở lại, tần số khoảng 20 Hz |
| `test_pointcloud_generator_ros.py` | Publish định kỳ, layout `PointCloud2`, nội dung tĩnh với stamp tăng dần, `num_obs=0`, `frame_id` tùy chỉnh, bật `publish_static_tf` |

Dùng chung `ros_utils.py` (tạo `Rig`, odom và cloud giả) và `conftest.py`.

```bash
python3 -m pytest test/test_planner_node_ros.py test/test_planner_merge_ros.py test/test_pointcloud_generator_ros.py -v
```

### Tầng 3: rosbag

`test_apf_rosbag.py` tự sinh một bag tổng hợp, launch `apf_planner.launch.py planner_type:=apf` (gồm `planner_node` và `planner_merge_node`), phát lại bag rồi kiểm tra `planner/velocity_setpoint`. UAV đứng yên tại (-8, -8, 3), H-Pad tại (8, 8, 0), odom và target 20 Hz, phase và mode 10 Hz.

| Đoạn | Phase | Mode | Thời lượng | Kỳ vọng |
|---|---|---|---|---|
| 1 | IDLE | LOST | 2.0 s | Vận tốc bằng 0 |
| 2 | FOLLOW | TRACKING | 3.0 s | Vận tốc theo hướng H-Pad (`vx`, `vy` dương), yaw gần π/4 |
| 3 | FOLLOW | EXPIRED | 1.5 s | Vận tốc bằng 0 |
| 4 | APPROACH | TRACKING | 2.0 s | `vx`, `vy` dương, `vz` âm |
| 5 | IDLE | LOST | 1.5 s | Vận tốc bằng 0 |

Mỗi đoạn được kiểm tra theo tỉ lệ message (từ 70% đến 90%), vì phase và mode của message trễ một chút so với vận tốc ở thời điểm chuyển đoạn. Tổng số message `planner/velocity_setpoint` phải từ 100 trở lên.

```bash
python3 -m pytest test/test_apf_rosbag.py -v
```

Để thay bằng bag thật, ghi bag rồi đổi `bag_dir` trong `generate_test_description()` và chỉnh lại các giá trị kỳ vọng theo kịch bản đã ghi:

```bash
ros2 bag record -o sample_apf /odom /hpad/state_filtered /ekf/tracking_mode /mission/phase /ibvs/yaw_cmd
```

### Chạy toàn bộ

```bash
colcon build --packages-select apf_planner
source install/setup.bash
colcon test --packages-select apf_planner --event-handlers console_direct+
colcon test-result --verbose
```

### Lưu ý khi chạy test

- Test tầng 2 và 3 dùng thời gian thật nên có thể chập chờn trên máy chậm; chạy lại một lần trước khi kết luận lỗi.
- Không chạy song song nhiều test ROS trong cùng `ROS_DOMAIN_ID`, vì các node dùng chung tên topic. Nếu cần chạy cạnh hệ thống đang chạy, đặt domain riêng: `ROS_DOMAIN_ID=77 python3 -m pytest ...`.
- `PlannerOutput` được import từ `vdt_msgs`, nếu không có thì dùng `offboard_manager`. `planner_merge_node.py` cần import từ cùng nơi với test.
- Test rosbag chỉ kiểm tra chuỗi `planner_node` và `planner_merge_node`. IBVS, EKF và FSM không được chạy trong bag này.
- Test rosbag cần `ros2 bag play` trong `PATH` (package `ros2bag`) và cần plugin lưu trữ `sqlite3`.

Nếu test tầng 2 báo không nhận được message: kiểm tra `ros2 topic list` trong cùng domain có node khác đang publish cùng topic không, và nhớ rằng `/odom` dùng QoS BEST_EFFORT nên publisher giả cũng phải là BEST_EFFORT hoặc RELIABLE.

Nếu test rosbag báo thiếu message: tăng `BAG_DELAY` trong `test_apf_rosbag.py` (node cần thời gian khởi động trước khi bag phát), hoặc kiểm tra `colcon build` đã cài thư mục `launch/` vào `share/apf_planner`.