# APF Planner — Guide

## 1. Package cần cài đặt ngoài

Chỉ cần `numpy`, không có binary ngoài nào phải build riêng.

```bash
python3 -m pip install numpy
```

Package ROS2 cần có sẵn: `rclpy`, `geometry_msgs`, `nav_msgs`, `sensor_msgs`, `std_msgs`, `visualization_msgs`, `tf2_ros_py`, `launch_ros` và `vdt_msgs` (message `PlannerOutput`, phải build trước).

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
1. Khai báo tham số, tạo lõi theo `planner_type`: `apf` (`APFCore`, 2D ngang) hoặc `iapf` (`IAPFCore`, 3D). Hai lõi là Python thuần, không phụ thuộc ROS.
2. Nếu `obstacle_source` là `sdf` hoặc `both`, đọc cylinder từ `world_sdf`.
3. Tạo subscriber, publisher và timer 30 Hz.

Mỗi chu kỳ (`tick`):
1. Chỉ tính lệnh khi phase là `FOLLOW`/`APPROACH`, `tracking_mode` thuộc `allowed_tracking_modes` (`TRACKING`, `PREDICTING`), và odom, target cùng tươi trong `data_timeout_sec`. Ngược lại publish `Twist` bằng 0.
2. Tính goal: APPROACH là vị trí H-Pad; FOLLOW là điểm stand-off cách H-Pad `follow_distance` (khoảng cách chéo 3D), giữ độ cao `target_altitude`, có lead theo vận tốc target.
3. Chọn vật cản: point cloud được voxel downsample khi nhận message; mỗi chu kỳ lấy điểm trong bán kính `2·d0`, mỗi cụm một điểm gần nhất, tối đa `max_cloud_points`.
4. Gọi lõi (APPROACH dùng `k_rep_approach`), ở FOLLOW cộng feedforward vận tốc ngang của target khi target di chuyển >= 0.20 m/s, kẹp theo `v_max`.
5. Publish `/apf/velocity_cmd`, `/apf/yaw_cmd` (hướng ENU từ UAV tới H-Pad) và marker RViz.

`planner_merge_node` (20 Hz):
- FOLLOW/APPROACH: chuyển tiếp vận tốc APF; yaw ưu tiên IBVS, dự phòng APF, cuối cùng NaN (giữ yaw).
- Phase khác: publish vận tốc 0, yaw NaN.
- Mất `/apf/velocity_cmd` quá `upstream_timeout_sec`: hover thêm `stale_hover_sec` rồi ngừng publish để FSM thấy `planner_timeout`.

Lưu ý:
- `ekf_node` publish liên tục kể cả khi `EXPIRED`, nên cổng theo mode là bắt buộc, không thể chỉ dựa vào độ tươi của timestamp.
- `planner_merge_node` phải publish đều ở mọi phase. Nếu chỉ publish trong FOLLOW/APPROACH, `input_state_cache` sẽ báo `planner_timeout` ngay khi vào FOLLOW và FSM quay về SEARCH.
- Message có `frame_id` khác `world_frame` hoặc giá trị non-finite bị bỏ; kết quả có NaN thì publish vận tốc 0.
- `apf` đơn giản và dễ kẹt cực tiểu cục bộ; `iapf` thêm lực đẩy GNRON, lực tiếp tuyến 3D để thoát kẹt (hysteresis `iapf_f_enter`/`iapf_f_exit`) và chống dao động hướng.
- Lõi I-APF chạy vòng lặp Python, chi phí tăng theo `max_cloud_points × iapf_n_tangent × iapf_n_pred`.
- Khi dùng point cloud, `k_rep` cần hiệu chỉnh lại so với cylinder (SDF).

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

Launch arguments: `params_file`, `generator` (mặc định `false`), `planner_type` (mặc định `iapf`).

Tham số chính của `planner_node` (đầy đủ trong `config/apf_params.yaml`):

| Tên | Mặc định | Ý nghĩa |
|---|---|---|
| `planner_type` | `iapf` | `apf` hoặc `iapf` |
| `obstacle_source` | `pointcloud` | `pointcloud`, `sdf` hoặc `both` |
| `target_topic` | `/hpad/state_filtered` | Trạng thái H-Pad từ EKF |
| `allowed_tracking_modes` | `[TRACKING, PREDICTING]` | Mode cho phép tính lệnh |
| `data_timeout_sec` | 0.5 | Tuổi tối đa của odom/target |
| `d0` | 2.0 | Bán kính ảnh hưởng vật cản (m) |
| `v_max` | 1.2 | Tốc độ tối đa (m/s) |
| `k_att` / `k_rep` / `k_rep_approach` | 10 / 250 / 125 | Gain lực hút, lực đẩy, lực đẩy ở APPROACH |
| `goal_threshold` | 0.20 | Ngưỡng tới goal (m) |
| `follow_distance` | 3.5 | Khoảng cách chéo 3D tới H-Pad ở FOLLOW (m) |
| `hold_follow_altitude` / `target_altitude` | true / 3.0 | Giữ độ cao FOLLOW (m, ENU) |
| `vz_max` | 0.5 | Vận tốc đứng tối đa (m/s) |
| `cloud_voxel_size` | 0.3 | Voxel downsample (m) |
| `cloud_cluster_radius` | 1.2 | Bán kính loại điểm trùng cụm (m) |
| `max_cloud_points` | 8 | Số điểm vật cản tối đa mỗi chu kỳ |
| `iapf_f_enter` / `iapf_f_exit` | 0.10 / 0.30 | Ngưỡng vào/ra chế độ tiếp tuyến |
| `iapf_k_tan` | 1.0 | Độ lớn lực tiếp tuyến |

Tham số của `planner_merge`:

| Tên | Mặc định | Ý nghĩa |
|---|---|---|
| `rate_hz` | 20 | Tần số publish |
| `yaw_source` | `ibvs_apf` | `ibvs_apf`, `ibvs`, `apf` hoặc `hold` |
| `upstream_timeout_sec` | 0.3 | Tuổi tối đa của vận tốc APF |
| `stale_hover_sec` | 0.5 | Thời gian hover trước khi ngừng publish |
| `yaw_timeout_sec` | 0.3 | Tuổi tối đa của nguồn yaw |

Tham số của `apf_pointcloud_generator`: `num_obs=35`, `map_size=25.0`, `height=4.0`, `resolution=0.15`, `clear_radius=2.0`, `seed=-1` (>= 0 để tái lập), `rate_hz=1.0`, `publish_static_tf=true`.

Topic đầu vào của `planner_node`:

| Topic | Kiểu | Nội dung |
|---|---|---|
| `/odom` | `nav_msgs/Odometry` | UAV, frame `world`, BEST_EFFORT |
| `/hpad/state_filtered` | `nav_msgs/Odometry` | H-Pad, frame `world` |
| `/ekf/tracking_mode` | `std_msgs/String` | Mode tracking của EKF |
| `/mission/phase` | `std_msgs/String` | Phase hiện tại |
| `/map_generator/global_cloud` | `sensor_msgs/PointCloud2` | Vật cản, frame `world` |

Topic đầu ra:

| Topic | Kiểu | Nội dung |
|---|---|---|
| `/apf/velocity_cmd` | `Twist` | Vận tốc ENU (m/s) |
| `/apf/yaw_cmd` | `Float64` | Heading ENU (rad), chỉ publish khi đang tính lệnh |
| `/apf/force_markers` | `MarkerArray` | Debug RViz |
| `planner/velocity_setpoint` | `offboard_manager/PlannerOutput` | Lệnh đã ghép, ENU (từ `planner_merge_node`) |

## 4. Cách debug

Kiểm tra nhịp topic:

```bash
ros2 topic hz /apf/velocity_cmd
ros2 topic hz planner/velocity_setpoint
ros2 topic echo /apf/velocity_cmd
```

`planner/velocity_setpoint` phải ổn định khoảng 20 Hz ở mọi phase. Nếu không, FSM sẽ thấy `planner_timeout`.

Xem marker trong RViz2: Fixed Frame `world`, thêm `MarkerArray` `/apf/force_markers` và `PointCloud2` `/map_generator/global_cloud`. Mũi tên xanh lá là lực hút, đỏ là lực đẩy, xanh dương là vận tốc lệnh, tím là lực tiếp tuyến (chỉ khi I-APF đang thoát cực tiểu); cầu cyan là UAV, cầu vàng là goal, chấm đỏ là điểm vật cản đang dùng.

Test nhanh không cần PX4 (UAV đứng yên nên chỉ kiểm tra lệnh và marker):

```bash
ros2 topic pub -r 20 /odom nav_msgs/msg/Odometry "{header: {frame_id: world}, pose: {pose: {position: {x: -8.0, y: -8.0, z: 3.0}}}}"
ros2 topic pub -r 20 /hpad/state_filtered nav_msgs/msg/Odometry "{header: {frame_id: world}, pose: {pose: {position: {x: 8.0, y: 8.0, z: 0.0}}}}"
ros2 topic pub -r 10 /ekf/tracking_mode std_msgs/msg/String "{data: TRACKING}"
ros2 topic pub -r 10 /mission/phase std_msgs/msg/String "{data: FOLLOW}"
```

Nếu `/apf/velocity_cmd` luôn bằng 0: kiểm tra lần lượt phase có là `FOLLOW`/`APPROACH`, `tracking_mode` có thuộc `allowed_tracking_modes` (`PREDICTING_DEGRADED` và `EXPIRED` bị chặn), odom và target có đang đến trong `data_timeout_sec`, và `frame_id` có đúng `world` (sai frame sẽ có cảnh báo `Ignoring frame`).

Nếu `/apf/yaw_cmd` không có message: planner chỉ publish yaw khi đang tính lệnh; ngoài FOLLOW/APPROACH thì không có.

Nếu UAV không né vật cản: kiểm tra `obstacle_source` khớp nguồn đang có, `/map_generator/global_cloud` có message và frame là `world`, và `world_sdf` đúng đường dẫn khi dùng `sdf`. Chấm đỏ trên RViz cho biết điểm vật cản đang được dùng.

Nếu lực đẩy quá mạnh khi dùng point cloud: giảm `max_cloud_points` hoặc `k_rep`, hoặc tăng `cloud_cluster_radius`.

Nếu UAV kẹt giữa vật cản (I-APF): tăng `iapf_f_enter` hoặc `iapf_k_tan`. Nếu đổi hướng liên tục thì tăng `iapf_w_prev`.

Nếu rung quanh goal: tăng `goal_threshold` (hoặc `d_slow` khi dùng `apf`).

Nếu `planner/velocity_setpoint` im lặng trong FOLLOW/APPROACH: `planner_merge_node` đã ngừng publish vì mất `/apf/velocity_cmd` quá `upstream_timeout_sec + stale_hover_sec`; kiểm tra `planner_node` còn chạy không.

Nếu yaw luôn là NaN (UAV không xoay): kiểm tra `/ibvs/yaw_cmd` và `/apf/yaw_cmd` có đến trong `yaw_timeout_sec`, và `yaw_source` có phải `hold` không.