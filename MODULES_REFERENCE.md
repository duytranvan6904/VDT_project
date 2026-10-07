# 📖 MODULES_REFERENCE — Tài liệu kỹ thuật chi tiết

> Tài liệu này mô tả chi tiết chức năng, ROS 2 interface, tham số cấu hình và cách test độc lập từng module trong hệ thống.  
> Đọc [`README.md`](README.md) trước để nắm pipeline tổng thể.

---

## 📑 Mục lục

1. [Module Map & Dependencies](#1-module-map--dependencies)
2. [simulation/core — Node điều phối](#2-simulationcore--node-điều-phối)
3. [simulation/perception — Nhận thức mục tiêu](#3-simulationperception--nhận-thức-mục-tiêu)
4. [simulation/control — Thuật toán điều khiển](#4-simulationcontrol--thuật-toán-điều-khiển)
5. [simulation/utils — Công cụ giám sát](#5-simulationutils--công-cụ-giám-sát)
6. [vision/ — Hardware Vision Stack](#6-vision--hardware-vision-stack)
7. [hardware/ — Entry Point Phần cứng](#7-hardware--entry-point-phần-cứng)
8. [ROS 2 Topics & TF Tree đầy đủ](#8-ros-2-topics--tf-tree-đầy-đủ)
9. [Tham số cấu hình đầy đủ](#9-tham-số-cấu-hình-đầy-đủ)
10. [Hướng dẫn test từng module độc lập](#10-hướng-dẫn-test-từng-module-độc-lập)

---

## 1. Module Map & Dependencies

```
simulation/launch/launch_simulation.py   ← ĐIỂM VÀO CHÍNH
         │ spawn subprocess
         ├──► simulation/perception/ekf_ros_adapter.py
         │         ▲ subscribes
         │    simulation/perception/aruco_sim_node.py (Nested Dual-Scale Board)
         │
         ├──► simulation/control/apf_planner.py
         │         ▲ /ekf/target_state, /odom
         │
         ├──► simulation/control/ibvs_controller.py
         │         ▲ /hpad/bbox, /hpad/position_camera (Gimbal Pitch + Body Yaw + Nadir Tilt)
         │
         ├──► simulation/control/covariance_gate.py
         │         ▲ /ekf/target_state, /odom (P_target + P_drone fusion, Conical Gate)
         │
         ├──► simulation/control/smc_guidance.py
         │         ▲ /ekf/target_state, /odom, /landing/rswitch_adaptive (45° Glide Slope + Soft Landing)
         │
         ├──► simulation/control/touchdown_detector.py
         │         ▲ /odom, /mission/velocity_setpoint (Kinematic Stoppage + Confirmation Latch)
         │
         ├──► simulation/core/mission_fsm_node.py
         │         ▲ /ekf/tracking_mode, /apf/guidance_velocity_cmd, /landing/velocity_cmd, /landing/touchdown
         │         ▼ /mission/velocity_setpoint, /mission/yaw_setpoint, /mission/phase
         │
         └──► simulation/core/offboard_commander.py
                   ▲ /mission/velocity_setpoint, /mission/yaw_setpoint
                   ▼ PX4 MAVLink TrajectorySetpoint / MAVLink Force-Disarm
```

---

## 2. simulation/core — Node điều phối

### 2.1 `mission_fsm_node.py` — State Machine chính

**Chức năng**: Điều phối toàn bộ chu trình tự hành (End-to-End Mission Lifecycle). Quản lý chuyển pha an toàn giữa các chế độ bay, tích hợp Covariance Gate giám sát hạ cánh, tổng hợp velocity setpoint gửi về Offboard Commander.

**Các trạng thái FSM đầy đủ:**

| State | Điều kiện vào | Điều kiện ra |
|---|---|---|
| `IDLE` | Khởi động hệ thống hoặc hoàn tất tiếp đất | `alt > 80% takeoff_alt` → `SEARCH` |
| `SEARCH` | Đủ độ cao cất cánh | `ArUco detected ≥ 5 frames` & EKF `TRACKING` → `REACQUIRE_HOLD` → `FOLLOW` |
| `FOLLOW` | EKF `TRACKING` ổn định | `EKF EXPIRED > search_timeout` → `SEARCH`; Nhận `/operator/land_command` → `APPROACH` |
| `APPROACH` | Nhận lệnh Land từ Operator | Covariance $2\sigma \le 0.45\text{m}$, góc nón $\le 30^\circ$, $R_{xy} \le R_{switch}$ → `GLIDE_SLOPE`; Mất dấu $> 3.0\text{s}$ → `FOLLOW` |
| `GLIDE_SLOPE` | Covariance Gate xác nhận an toàn | $R_{xy} \le 0.40\text{m}$ & $z \le 0.40\text{m}$ & $2\sigma \le 0.35\text{m}$ → `LAND`; Mất dấu $> 1.5\text{s}$ hoặc $h_{err} > 0.6\text{m}$ → Wave-off Abort về `APPROACH` |
| `LAND` | Đạt đỉnh phễu hạ cánh thẳng đứng | `/landing/touchdown` kích hoạt hoặc $z \le 0.12\text{m}$ trong $0.5\text{s}$ → `TOUCHDOWN` / `IDLE` |
| `TOUCHDOWN` | Tiếp xúc sàn bãi đáp xác nhận | Offboard Commander gửi MAVLink Force Disarm (`param2=21196.0`) → `IDLE` |

**ROS 2 Interface:**

| Direction | Topic | Type | Mô tả |
|---|---|---|---|
| Subscribe | `/odom` | `nav_msgs/Odometry` | Trạng thái vị trí và vận tốc drone |
| Subscribe | `/ekf/target_state` | `nav_msgs/Odometry` | Vị trí mục tiêu và ma trận hiệp phương sai |
| Subscribe | `/ekf/tracking_mode` | `std_msgs/String` | Trạng thái lọc EKF: IDLE/TRACKING/DEAD_RECKONING/EXPIRED |
| Subscribe | `/hpad/detected` | `std_msgs/Bool` | Cờ nhận dạng marker ArUco |
| Subscribe | `/hpad/bbox` | `vision_msgs/BoundingBox2D` | Khung nhận diện pixel của marker |
| Subscribe | `/operator/land_command` | `std_msgs/Bool` | Lệnh kích hoạt hạ cánh chính xác |
| Subscribe | `/apf/guidance_velocity_cmd`| `geometry_msgs/Twist` | Vận tốc né vật cản từ APF/I-APF |
| Subscribe | `/apf/target_velocity_ff` | `geometry_msgs/Twist` | Vận tốc bù chuyển động mục tiêu (Feedforward) |
| Subscribe | `/ibvs/yaw_cmd` | `std_msgs/Float64` | Lệnh góc yaw từ bộ điều khiển IBVS |
| Subscribe | `/landing/safe_to_land` | `std_msgs/Bool` | Cờ cho phép hạ cánh từ Covariance Gate |
| Subscribe | `/landing/uncertainty_radius` | `std_msgs/Float64` | Bán kính không định vị trí tương đối $2\sigma$ (m) |
| Subscribe | `/landing/rswitch_adaptive` | `std_msgs/Float64` | Bán kính chuyển mạch dốc trượt thích ứng (m) |
| Subscribe | `/landing/velocity_cmd` | `geometry_msgs/Twist` | Lệnh vận tốc hạ cánh 3D từ SMC Guidance |
| Subscribe | `/landing/sub_phase` | `std_msgs/String` | Pha hạ cánh nội bộ của SMC (GLIDE_SLOPE / FINAL_DESCENT) |
| Subscribe | `/landing/touchdown` | `std_msgs/Bool` | Tín hiệu xác nhận tiếp đất từ Touchdown Detector |
| Publish | `/mission/phase` | `std_msgs/String` | Pha nhiệm vụ hiện tại |
| Publish | `/mission/velocity_setpoint` | `geometry_msgs/Twist` | Vận tốc tổng hợp gửi Offboard Commander |
| Publish | `/mission/yaw_setpoint` | `std_msgs/Float64` | Góc yaw mục tiêu gửi Offboard Commander |
| Publish | `/mission/waveoff_event` | `std_msgs/String` | Cảnh báo sự kiện hủy hạ cánh (Wave-off) |

**Tham số chính** (từ `mission_params.yaml`):

```yaml
mission_fsm:
  follow_distance: 3.5          # Cự ly bám (m)
  alignment_threshold: 0.3      # Ngưỡng căn chỉnh hạ cánh (m)
  land_altitude: 0.5            # Độ cao kích hoạt LAND (m)
  search_timeout: 1.2           # Timeout mất dấu → SEARCH (s)
  approach_timeout: 3.0         # Timeout mất ArUco trong APPROACH → FOLLOW (s)
  takeoff_altitude: 3.0         # Độ cao cất cánh mục tiêu (m)
  yaw_align_enable: true        # Bật căn chỉnh Yaw trước khi FOLLOW
  reacquire_confirm_time: 0.25  # Thời gian confirm tái nhận diện (s)
  reacquire_hold_timeout: 4.0   # Timeout giữ vị trí khi tái nhận diện (s)
```

---

### 2.2 `offboard_commander.py` — Giao tiếp PX4

**Chức năng**: Nhận velocity setpoint từ FSM, chuyển đổi hệ tọa độ ENU→NED, gửi lệnh Offboard tới PX4.

**Chế độ hoạt động:**
- **SIM-ONLY mode**: Tự động kích hoạt khi `px4_msgs` chưa được build. Log setpoint ra console để debug, không crash.
- **Hardware mode**: Build workspace có `px4_msgs`, kết nối qua MicroXRCE-DDS hoặc MAVLink UDP.

**ROS 2 Interface:**

| Direction | Topic | Type | Mô tả |
|---|---|---|---|
| Subscribe | `/mission/velocity_setpoint` | `geometry_msgs/Twist` | Vận tốc lệnh từ FSM (kích hoạt disarm khi nhận cờ dừng) |
| Subscribe | `/mission/yaw_setpoint` | `std_msgs/Float64` | Góc yaw mục tiêu |
| Subscribe | `/landing/touchdown` | `std_msgs/Bool` | Tín hiệu tiếp đất (kích hoạt ngắt động cơ tức thì) |
| Subscribe | `/odom` | `nav_msgs/Odometry` | Giám sát độ cao thực tế để kích hoạt failsafe tiếp đất |
| Publish | PX4 `TrajectorySetpoint` | `px4_msgs/TrajectorySetpoint` / MAVLink UDP/UART | Setpoint vận tốc và góc hướng |

**Cơ chế ngắt động cơ an toàn (Force-Disarm Failsafe):**
- **Touchdown Latch**: Khi nhận `/landing/touchdown = True`, node gửi MAVLink command `MAV_CMD_COMPONENT_ARM_DISARM` với `param1=0.0` và `param2=21196.0` (Force Disarm bypass preflight checks).
- **Ground-Level Resting Failsafe**: Nếu độ cao $z \le 0.12\text{m}$ duy trì liên tục $\ge 0.8\text{s}$ trong khi lệnh vận tốc hạ đang hoạt động, node tự động ngắt động cơ dự phòng, ngăn chặn hoàn toàn hiện tượng nảy hoặc lật máy bay khi trễ bản tin tiếp đất.

**Tham số:**

```yaml
offboard_commander:
  auto_arm: true          # Tự động Arm khi vào Offboard mode
  auto_offboard: true     # Tự chuyển sang Offboard mode
  min_offboard_alt: 2.0   # Độ cao tối thiểu để kích hoạt Offboard (m)
  max_accel: 1.0          # Gia tốc tối đa (m/s²)
  enable_alt_hold: true   # Bật altitude hold P-controller
  target_altitude: 3.0    # Độ cao mục tiêu altitude hold (m)
  alt_hold_kp: 1.0        # Hệ số P của altitude hold
```

> **Lưu ý chuyển sang hardware**: Thay `connection_string = "udp:127.0.0.1:14540"` thành UART port của Companion Computer (ví dụ `/dev/ttyTHS1`).

---

### 2.3 `takeoff.py` — Script cất cánh

**Chức năng**: MAVLink script tự động Arm và takeoff lên độ cao chỉ định.

```bash
python3 simulation/core/takeoff.py --alt 3.0
# Tùy chọn:
#   --alt FLOAT    Độ cao cất cánh (mặc định: 3.0 m)
#   --host STR     MAVLink host (mặc định: 127.0.0.1)
#   --port INT     MAVLink port (mặc định: 14540)
```

---

## 3. simulation/perception — Nhận thức mục tiêu

### 3.1 `aruco_sim_node.py` — Nested Dual-Scale ArUco Detector (Simulation)

**Chức năng**: Nhận ảnh RGB/Depth từ Gazebo, phát hiện cấu trúc **Nested Dual-Scale ArUco Board** (Outer ID 42: 52.5cm khổ A0 cho tầm xa; Inner ID 43: 10cm cho tầm gần sát đất). Sử dụng `cv2.aruco.estimatePoseBoard()` với gốc tọa độ 3D đặt chuẩn xác tại tâm hình học bãi đáp H-Pad. Đảm bảo ước lượng liên tục từ cự ly 8m xuống tận 0.05m mà không bị nhảy tọa độ khi chuyển marker.

**ROS 2 Interface:**

| Direction | Topic | Type | Mô tả |
|---|---|---|---|
| Subscribe | `/camera` | `sensor_msgs/Image` | Ảnh RGB từ Gazebo (640×480 @ 30fps) |
| Subscribe | `/camera_info` | `sensor_msgs/CameraInfo` | Intrinsics camera |
| Subscribe | `/depth_camera` | `sensor_msgs/Image` | Ảnh depth (32FC1) |
| Publish | `/hpad/detected` | `std_msgs/Bool` | Cờ phát hiện marker |
| Publish | `/hpad/position_camera` | `geometry_msgs/PointStamped` | Vị trí 3D tâm bãi đáp theo camera frame |
| Publish | `/hpad/bbox` | `vision_msgs/BoundingBox2D` | Khung nhận diện pixel của marker đang theo dõi |
| Publish | `/hpad/annotated` | `sensor_msgs/Image` | Ảnh debug với bounding box và trục tọa độ |
| Publish | `/hpad/pose` | `geometry_msgs/PoseStamped` | Pose đầy đủ 6-DOF của bãi đáp |

**Tham số:**

```yaml
aruco_sim_node:
  board_config_path: "vision/dual_scale_board_config.yaml" # File định nghĩa 3D board
  outer_marker_size_m: 0.525       # Kích thước outer marker ID 42 (m)
  inner_marker_size_m: 0.100       # Kích thước inner marker ID 43 (m)
  min_detection_distance_m: 0.05   # Cự ly tối thiểu chấp nhận (m)
```

> **Thay thế trong hardware**: Dùng `vision/aruco_detector.py` đọc trực tiếp từ RealSense SDK với cùng cấu trúc board.

---

### 3.2 `ekf_ros_adapter.py` — EKF Target State Estimator

**Chức năng**: Cầu nối EKF với ROS 2. Nhận tọa độ camera frame → tra TF2 → chuyển sang `world` frame → cấp cho EKF 6-state (`x, y, z, vx, vy, vz`). Hỗ trợ Dead Reckoning khi mất tín hiệu.

**Các chế độ EKF:**

| Mode | Ý nghĩa |
|---|---|
| `IDLE` | Chưa nhận được detection nào |
| `TRACKING` | Đang nhận detection, EKF hội tụ |
| `DEAD_RECKONING` | Mất detection < timeout, dự đoán quán tính |
| `PREDICTING_DEGRADED` | Mất detection > threshold, độ chính xác giảm |
| `EXPIRED` | Mất detection quá lâu, FSM cần chuyển SEARCH |

**ROS 2 Interface:**

| Direction | Topic | Type |
|---|---|---|
| Subscribe | `/hpad/position_camera` | `geometry_msgs/PointStamped` |
| Subscribe | `/hpad/detected` | `std_msgs/Bool` |
| TF lookup | `world ← camera_optical_frame` | TF2 |
| Publish | `/ekf/target_state` | `nav_msgs/Odometry` |
| Publish | `/ekf/tracking_mode` | `std_msgs/String` |
| Publish | `/ekf/predicted_marker` | `geometry_msgs/PointStamped` |

**Tham số:**

```yaml
ekf_ros_adapter:
  process_accel_variance: [1.0, 1.0, 0.5]  # Nhiễu gia tốc mô hình chuyển động [x,y,z]
  gate_threshold: 16.27       # Ngưỡng Mahalanobis distance gate (chi-squared 3DOF, p=0.001)
  predict_rate_hz: 50.0       # Tần số bước predict EKF (Hz)
  max_target_speed: 2.5       # Tốc độ tối đa mục tiêu được chấp nhận (m/s)
  min_marker_distance: 2.2    # Khoảng cách tối thiểu chấp nhận measurement (m)
```

---

### 3.3 `depth_to_image_node.py` — Depth Normalizer

**Chức năng**: Chuyển depth image từ định dạng `32FC1` (float, giá trị là khoảng cách tính bằng mét) sang `mono8` (uint8, 0-255) để hiển thị trực tiếp trong RViz2.

```
/depth_camera (32FC1) ──► depth_to_image_node ──► /depth_camera/image_mono (mono8)
```

---

### 3.4 `mock_target_publisher.py` — Mock Target Publisher

**Chức năng**: Publish vị trí H-Pad fake theo quỹ đạo có thể lập trình (circle, line, random) lên `/hpad/position_camera`. Dùng để test EKF và FSM mà **không cần chạy Gazebo hay ArUco**.

```bash
python3 simulation/perception/mock_target_publisher.py --pattern circle --radius 3.0 --speed 0.5
# Patterns: circle, line, figure8, random
```

---

## 4. simulation/control — Thuật toán điều khiển

### 4.1 `apf_planner.py` & `iapf_core.py` — APF / Improved APF (I-APF) 3D Path Planner

**Chức năng**: Tính vector vận tốc điều khiển né vật cản 3D bằng Artificial Potential Field. Hỗ trợ 2 engine có thể chuyển đổi qua tham số `planner_type` hoặc cờ CLI `--planner <iapf|apf>`:
- **`IAPFCore` (`iapf_core.py`) — MẶC ĐỊNH**: Thuật toán Improved APF 3D port từ MATLAB `avoidance/IAPF_Planner_3D_MultiObs.m`. Tích hợp giải quyết GNRON (Sigmoid goal weighting), thoát bẫy cực tiểu địa phương (Dynamic 3D Tangent Plane + Hysteresis), lọc dao động đổi hướng (Oscillation Suppression) và bảo đảm sàn vận tốc thoát hiểm.
- **`APFCore` (`apf_planner.py`)**: Bản APF 2.5D cơ bản để làm đối chứng (benchmark).

**Cấu trúc thuật toán I-APF:**

```
Input: p_drone (world), p_target (world), obstacles (cylinder list)
  │
  ├── make_follow_goal()     → p_follow (điểm đứng cách target d_follow theo mặt nghiêng 3D)
  ├── F_att = 2*k_att*(p_follow - p_drone)
  ├── F_rep = F_rep1 (đẩy xa với Sigmoid weight) + F_rep2 (hướng về đích triệt tiêu GNRON)
  ├── F_total = F_att + F_rep
  │
  ├── Kiểm tra Local Minima (|F_total| < F_enter với Hysteresis)
  │     ├── Có  → Dynamic Tangent 3D: Lấy mẫu 12 hướng tiếp tuyến, Forward-looking 3 bước, chọn tia tối ưu
  │     │         F_cmd_raw = F_total + F_tan
  │     └── Không → F_cmd_raw = F_total
  │
  ├── Oscillation Suppression → Lọc hướng di chuyển mượt mà dựa trên góc lệch Δalpha
  ├── Speed Scheduling        → Điều biến tốc độ theo goal & obstacle, kẹp v_escape_min khi thoát bẫy
  └── Target Feedforward      → Bù vận tốc mục tiêu di động trong pha FOLLOW
                                │
Output: /apf/velocity_cmd (Twist), /apf/yaw_cmd (Float64), /apf/force_markers (MarkerArray)
```

**Nạp vật cản từ SDF:**
```python
from simulation.control.apf_planner import load_cylinder_obstacles_from_sdf
obstacles = load_cylinder_obstacles_from_sdf('/path/to/obstacle_avoidance.sdf')
```

**ROS 2 Interface:**

| Direction | Topic | Type |
|---|---|---|
| Subscribe | `/ekf/target_state` | `nav_msgs/Odometry` |
| Subscribe | `/odom` | `nav_msgs/Odometry` |
| Subscribe | `/mission/phase` | `std_msgs/String` |
| Subscribe | `/ekf/tracking_mode`| `std_msgs/String` |
| Publish | `/apf/velocity_cmd` | `geometry_msgs/Twist` |
| Publish | `/apf/yaw_cmd` | `std_msgs/Float64` |
| Publish | `/apf/force_markers` | `visualization_msgs/MarkerArray` |

**Tham số:**

```yaml
apf_planner:
  planner_type: "iapf"       # "iapf" (cải tiến 3D) hoặc "apf" (cơ bản)
  d0: 2.5                    # Bán kính ảnh hưởng vật cản (m)
  v_max: 1.5                 # Vận tốc tối đa output (m/s)
  d_slow: 1.2                # Bán kính giảm tốc khi gần goal (m)
  k_att: 10.0                # Hệ số lực hút
  k_rep: 2500.0              # Hệ số lực đẩy vật cản (FOLLOW phase)
  k_rep_approach: 125.0      # Hệ số lực đẩy giảm (APPROACH phase)
  follow_distance: 3.5       # Cự ly bám mục tiêu (m)
  hold_follow_altitude: true # Giữ nguyên độ cao trong FOLLOW
  target_lead_time: 0.35     # Dự báo vị trí target trước (s)
  # Tham số riêng của I-APF:
  iapf_f_enter: 0.10         # Ngưỡng kích hoạt thoát kẹt (N)
  iapf_f_exit: 0.30          # Ngưỡng thoát chế độ tiếp tuyến (N)
  iapf_k_tan: 1.0            # Hệ số độ lớn lực tiếp tuyến
  iapf_n_tangent: 12         # Số hướng candidate trên mặt phẳng tiếp tuyến 3D
  iapf_n_pred: 3             # Số bước dự báo trước để đánh giá clearance
  iapf_w_goal: 1.0           # Trọng số hướng về goal
  iapf_w_clear: 1.0          # Trọng số độ thông thoáng vật cản
  iapf_w_prev: 0.60          # Trọng số liên tục với hướng tránh trước đó
```

---

### 4.2 `ibvs_controller.py` — Image-Based Visual Servoing

**Chức năng**: Điều khiển camera gimbal (pitch) và yaw rate của drone để triệt tiêu độ lệch pixel của marker so với tâm ảnh. Hỗ trợ Direct Visual Servoing Fallback khi mất 3D target nhưng vẫn thấy pixel, tự động chuyển chế độ Nadir Tilt ($-85^\circ$) khi hạ cánh gần mặt đất.

**Control law:**
```
e_u = u_marker - u0   (pixel error theo chiều ngang)
e_v = v_marker - v0   (pixel error theo chiều dọc)

yaw_rate   = -K_yaw  × e_u / focal_x
pitch_cmd  = pitch_current + K_pitch × e_v / focal_y
```

**Các chế độ & đặc tính cải tiến:**

| Điều kiện | Hành vi |
|---|---|
| EKF `TRACKING` + có pixel | Full IBVS: điều khiển pitch + yaw |
| EKF `PREDICTING_DEGRADED` + có pixel (< 3s) | Duy trì bám góc theo dự đoán EKF |
| Mất 3D nhưng còn pixel (`has_pixel`) | **Direct Visual Fallback**: triệt tiêu trực tiếp `e_u`, `e_v` |
| Pha `LAND` & độ cao $z < 0.8\text{m}$ | **Nadir Tilt Mode**: tự động ép gimbal pitch về $-85^\circ$ (nhìn thẳng xuống tâm H-Pad) |
| Cự ly sát đất ($z < 0.3\text{m}$) | **Pixel Trimming Deadband**: lọc vùng chết chống rung lắc gimbal |

**ROS 2 Interface:**

| Direction | Topic | Type | Mô tả |
|---|---|---|---|
| Subscribe | `/hpad/bbox` | `vision_msgs/BoundingBox2D` | Bounding box marker trên ảnh |
| Subscribe | `/hpad/position_camera` | `geometry_msgs/PointStamped` | Tọa độ 3D marker trong camera frame |
| Subscribe | `/mission/phase` | `std_msgs/String` | Pha FSM (FOLLOW, GLIDE_SLOPE, LAND, ...) |
| Subscribe | `/odom` | `nav_msgs/Odometry` | Giám sát độ cao drone để kích hoạt Nadir Tilt |
| Publish | `/ibvs/gimbal_pitch` | `std_msgs/Float64` | Lệnh góc pitch gimbal (rad) |
| Publish | `/ibvs/yaw_cmd` | `std_msgs/Float64` | Lệnh yaw rate (rad/s) |
| Publish | `/ibvs/tracking_error` | `geometry_msgs/Vector3` | Sai số pixel `[e_u, e_v, 0]` |

**Tham số:**

```yaml
ibvs_controller:
  K_pitch: 0.92            # Gain điều khiển gimbal pitch
  K_yaw: 0.92              # Gain điều khiển yaw rate
  focal_x: 466.0           # Tiêu cự camera theo X (pixels)
  focal_y: 466.0           # Tiêu cự camera theo Y (pixels)
  u0: 320.0                # Tọa độ tâm ảnh X (pixels)
  v0: 240.0                # Tọa độ tâm ảnh Y (pixels)
  pitch_rate_limit: 1.80   # Tốc độ thay đổi pitch tối đa (rad/s) — phản ứng nhanh khi hạ dốc
  yaw_rate_limit: 0.80     # Tốc độ quay yaw tối đa (rad/s)
  pitch_ema_alpha: 0.65    # Hệ số EMA lọc nhiễu gimbal pitch
```

---

### 4.3 `apf_pointcloud_generator.py` — APF PointCloud Publisher

**Chức năng**: Đọc file SDF world, tạo bản đồ PointCloud2 từ các obstacle và publish lên `/map_generator/global_cloud` để hiển thị trong RViz2. Node chạy song song với `launch_simulation.py`.

```
obstacle_avoidance.sdf ──► apf_pointcloud_generator ──► /map_generator/global_cloud (PointCloud2)
```

---

### 4.4 `covariance_gate.py` — Covariance Gating & Safety Monitor

**Chức năng**: Dung hợp ma trận hiệp phương sai ước lượng bãi đáp từ Target EKF ($P_{target}$) và sai số vị trí drone từ PX4 EKF2 ($P_{drone}$), tính toán ellipsoid không định tương đối $P_{relative} = P_{target} + P_{drone}$. Thiết lập phễu hình nón chấp nhận $30^\circ$ và điều biến bán kính chuyển mạch dốc trượt thích ứng ($R_{switch} \in [4.5\text{m}, 10.0\text{m}]$).

**Nguyên lý an toàn:**
- Tính bán kính tin cậy $2\sigma = 2\sqrt{\max(\text{eig}(P_{relative, xy}))}$.
- Kiểm tra góc nghiêng hình học so với tâm marker: $\theta_{cone} = \arctan(R_{xy} / \Delta z) \le 30^\circ$.
- Cấp quyền `safe_to_enter_glide` khi $2\sigma \le 0.45\text{m}$ và trong nón; cấp quyền `safe_to_descend` khi $2\sigma \le 0.35\text{m}$.

**ROS 2 Interface:**

| Direction | Topic | Type | Mô tả |
|---|---|---|---|
| Subscribe | `/ekf/target_state` | `nav_msgs/Odometry` | Trạng thái mục tiêu và ma trận $P_{target}$ 6×6 |
| Subscribe | `/odom` | `nav_msgs/Odometry` | Trạng thái drone và hiệp phương sai odometry |
| Publish | `/landing/safe_to_land` | `std_msgs/Bool` | Cờ cho phép hạ cánh |
| Publish | `/landing/uncertainty_radius` | `std_msgs/Float64` | Bán kính không định $2\sigma$ (m) |
| Publish | `/landing/rswitch_adaptive` | `std_msgs/Float64` | Bán kính chuyển mạch dốc trượt thích ứng (m) |
| Publish | `/landing/sliding_weight` | `std_msgs/Float64` | Hệ số trọng số bề mặt trượt $w_s \in [0, 1]$ |
| Publish | `/landing/covariance_status` | `diagnostic_msgs/DiagnosticStatus` | Chẩn đoán chi tiết giá trị riêng và góc nón |

**Tham số:**

```yaml
covariance_gate:
  max_uncertainty_enter_glide: 0.45   # Ngưỡng 2σ tối đa cho phép vào GLIDE (m)
  max_uncertainty_continue_glide: 0.50# Ngưỡng 2σ tối đa duy trì GLIDE (m)
  max_uncertainty_final_descent: 0.35 # Ngưỡng 2σ tối đa cho phép vào FINAL_DESCENT (m)
  rswitch_min: 4.5                    # Bán kính chuyển mạch tối thiểu (m)
  rswitch_max: 10.0                   # Bán kính chuyển mạch tối đa (m)
  conical_gate_angle_deg: 30.0        # Góc nửa nón an toàn (độ)
```

---

### 4.5 `smc_guidance.py` — Sliding Mode Control Precision Landing Guidance

**Chức năng**: Hiện thực hóa luật dẫn đường Sliding Mode Control (SMC) theo quỹ đạo dốc trượt nghiêng $45^\circ$ ($\theta_{des} = \pi/4$). Sử dụng luật tiếp cận lũy thừa (Power Reaching Law) trên 3 mặt trượt ($S_1$: cự ly LOS, $S_2$: góc dốc đứng, $S_3$: góc phương vị ngang). Tích hợp thành phần điều biến vận tốc hạ độ cao độc lập chống kẹt lơ lửng (Hover Stall).

**Cấu trúc điều khiển:**
```
Mặt trượt: S = [S1, S2, S3]^T
Power reaching: \dot{S}_i = -k_i |S_i|^(n/m) sgn(S_i)  (n/m = 3/5 = 0.6)
Vận tốc ngang: v_xy = -Kp * e_xy - Ks * sat(S / epsilon) + v_target_ff
Vận tốc đứng:  v_z  = -max(v_descend_touch, 0.20 * r_z)
```

**ROS 2 Interface:**

| Direction | Topic | Type | Mô tả |
|---|---|---|---|
| Subscribe | `/ekf/target_state` | `nav_msgs/Odometry` | Trạng thái mục tiêu và vận tốc bãi đáp |
| Subscribe | `/odom` | `nav_msgs/Odometry` | Trạng thái drone trong hệ quy chiếu world |
| Subscribe | `/landing/rswitch_adaptive` | `std_msgs/Float64` | Bán kính chuyển mạch dốc trượt thích ứng từ Covariance Gate |
| Publish | `/landing/velocity_cmd` | `geometry_msgs/Twist` | Lệnh vận tốc dẫn đường 3D trong hệ ENU |
| Publish | `/landing/sub_phase` | `std_msgs/String` | Pha nội bộ: GLIDE_SLOPE hoặc FINAL_DESCENT |

**Tham số:**

```yaml
landing_guidance:
  ka: 0.20          # Hệ số tốc độ khép cự ly mặt trượt S1
  kb: 0.60          # Hệ số tốc độ khép góc dốc mặt trượt S2
  kc: 0.40          # Hệ số tốc độ khép phương vị mặt trượt S3
  k1: 0.1395        # Gain reaching law mặt trượt 1
  k2: 0.1784        # Gain reaching law mặt trượt 2
  k3: 0.0442        # Gain reaching law mặt trượt 3
  m: 5.0            # Mẫu số lũy thừa
  n: 3.0            # Tử số lũy thừa
  theta_des_deg: 45.0 # Góc dốc tiếp cận mong muốn (độ)
  v_descend_glide: 0.35 # Tốc độ hạ tối đa trong GLIDE_SLOPE (m/s)
  v_descend_touch: 0.15 # Tốc độ hạ êm ái khi tiếp đất (m/s)
  final_gate_wait_alt: 0.40 # Ngưỡng độ cao chuyển sang FINAL_DESCENT (m)
```

---

### 4.6 `touchdown_detector.py` — Drift-Invariant Touchdown Detector

**Chức năng**: Bộ phát hiện chạm đất đa tiêu chí chống trôi dạt cảm biến áp suất/GPS. Kết hợp trạng thái dừng động học (Kinematic Stoppage: $z \le 0.18\text{m}$, $|v_z| \le 0.08\text{m/s}$), lệnh vận tốc hạ đang hoạt động ($v_{z\_cmd} \le -0.10\text{m/s}$), phát hiện xung lực va chạm (Jerk Impact Spike) và bộ chốt xác nhận thời gian liên tục ($0.35\text{s}$).

**Nguyên lý chốt chạm đất:**
1. Tránh race condition: Chỉ đăng ký duy nhất kênh lệnh vận tốc `/mission/velocity_setpoint`.
2. Kiểm tra cự ly sát sàn thực tế: $z_{drone} \le 0.18\text{m}$ và biến thiên độ cao $|dz/dt| \le 0.05\text{m/s}$.
3. Đếm thời gian xác nhận liên tục $\ge 0.35\text{s}$ trước khi chốt `/landing/touchdown = True`.

**ROS 2 Interface:**

| Direction | Topic | Type | Mô tả |
|---|---|---|---|
| Subscribe | `/odom` | `nav_msgs/Odometry` | Vị trí và vận tốc thực tế của drone |
| Subscribe | `/mission/velocity_setpoint` | `geometry_msgs/Twist` | Lệnh vận tốc tổng hợp đang gửi xuống PX4 |
| Publish | `/landing/touchdown` | `std_msgs/Bool` | Cờ chốt tiếp đất thành công (Latched True) |
| Publish | `/landing/touchdown_status` | `diagnostic_msgs/DiagnosticStatus` | Trạng thái các tầng phát hiện chạm đất |

**Tham số:**

```yaml
touchdown_detector:
  optical_height_threshold: 0.40     # Ngưỡng độ cao quang học (m)
  descent_cmd_threshold: -0.10       # Ngưỡng vận tốc hạ tối thiểu của lệnh (m/s)
  stoppage_vz_max: 0.08              # Ngưỡng vận tốc đứng tối đa khi đã tiếp sàn (m/s)
  confirmation_duration_s: 0.35      # Thời gian xác nhận chốt chạm đất (s)
```

---

## 5. simulation/utils — Công cụ giám sát

### 5.1 `hpad_keyboard_controller.py` — H-Pad Manual Controller

**Chức năng**: Điều khiển vị trí H-Pad trong Gazebo bằng bàn phím, gọi service `/world/obstacle_avoidance/set_pose`. Publish Ground Truth vị trí thực tế.

```bash
python3 simulation/utils/hpad_keyboard_controller.py --speed 1.5
# --speed FLOAT: Tốc độ di chuyển H-Pad (mặc định 1.0 m/s)
```

---

### 5.2 `monitor_follow.py` — Follow Phase Monitor

**Chức năng**: Giám sát real-time khoảng cách drone-target, sai số bám và trạng thái EKF trong FOLLOW phase. In bảng thống kê theo thời gian thực.

```bash
python3 simulation/utils/monitor_follow.py
```

---

### 5.3 `monitor_yaw.py` — Yaw Control Monitor

**Chức năng**: Hiển thị real-time: yaw hiện tại của drone, yaw target từ IBVS, sai số góc, và tốc độ quay. Hữu ích để debug yaw tracking.

```bash
python3 simulation/utils/monitor_yaw.py
```

---

### 5.4 `view_drone_camera.py` — Camera Viewer

**Chức năng**: Mở cửa sổ OpenCV hiển thị trực tiếp feed camera của drone từ topic `/camera` hoặc `/hpad/annotated`.

```bash
python3 simulation/utils/view_drone_camera.py --topic /hpad/annotated
```

---

### 5.5 `sync_rviz_gazebo_map.py` — Map Synchronizer

**Chức năng**: Đọc SDF world file, tạo PointCloud2 tương ứng với vị trí các obstacle trong Gazebo và publish liên tục cho RViz2. Đảm bảo bản đồ RViz2 luôn đồng bộ với Gazebo.

---

### 5.6 `drone_rviz_visualizer.py` — Drone 3D Visualizer

**Chức năng**: Subscribe `/odom`, publish `visualization_msgs/Marker` để hiển thị mô hình 3D drone trong RViz2 theo đúng vị trí và orientation.

---

## 6. vision/ — Hardware Vision Stack

> Đây là stack vision cho **phần cứng thực tế** (RealSense D435i thật, không phải Gazebo).

### 6.1 `aruco_detector.py` — ArUco Detector (Hardware)

**Chức năng**: Tương đương `aruco_sim_node.py` nhưng đọc trực tiếp từ RealSense SDK thay vì ROS topic. Xử lý hình ảnh thực từ camera vật lý.

**Interface** (giống aruco_sim_node):
```
Output topics: /hpad/detected, /hpad/position_camera, /hpad/bbox, /hpad/annotated
```

---

### 6.2 `realsense_stream.py` — RealSense Interface

**Chức năng**: Wrapper cho Intel RealSense SDK 2.0. Quản lý vòng đời camera, cấu hình stream (Color 640×480 @ 30Hz + Depth 640×480 @ 30Hz), xử lý lỗi kết nối.

```python
from vision.realsense_stream import RealSenseStream

cam = RealSenseStream()
cam.start()
color_frame, depth_frame = cam.get_frames()
cam.stop()
```

---

### 6.3 `target_state_ekf.py` — EKF Standalone

**Chức năng**: Implement thuần Python của EKF 6-state (`x, y, z, vx, vy, vz`) với:
- **Constant Velocity motion model**
- **Mahalanobis distance gating** để loại outlier
- **Dead Reckoning** khi mất measurement

Không phụ thuộc ROS — có thể import và dùng trực tiếp trong mọi Python script.

```python
from vision.target_state_ekf import TargetStateEKF

ekf = TargetStateEKF()
ekf.predict(dt=0.02)
accepted = ekf.update(measurement=np.array([x, y, z]))
state = ekf.get_state()  # [x, y, z, vx, vy, vz]
```

---

### 6.4 `depth_masker.py` — Depth Masker

**Chức năng**: Tạo binary mask từ depth image để loại bỏ nhiễu nền (background) và giữ lại vùng mục tiêu trong khoảng depth quan tâm.

---

### 6.5 `target_tracking_policy.py` — Tracking Decision Policy

**Chức năng**: Logic cấp cao quyết định drone nên tiếp tục FOLLOW hay chuyển SEARCH dựa trên history detection và EKF state. Wrapper mỏng trên EKF, tách biệt policy khỏi estimation.

---

### 6.6 `record_bag.py` — ROS 2 Bag Recorder

**Chức năng**: Script record ROS 2 bag file chọn lọc theo danh sách topic. Lưu dữ liệu sensor để replay và phân tích offline.

```bash
python3 vision/record_bag.py --output ~/bags/session_01 --duration 60
# Mặc định record: /camera, /depth_camera, /hpad/*, /ekf/*, /odom
```

Xem hướng dẫn đầy đủ: [`docs/README_BAG.md`](docs/README_BAG.md)

---

### 6.7 `utils.py` — Vision Utilities

Tập hợp hàm tiện ích dùng chung cho vision stack:
- `pixel_to_camera_ray()` — Chuyển pixel coordinate sang 3D ray
- `draw_axes()` — Vẽ trục tọa độ lên ảnh
- `rotation_matrix_to_euler()` — Chuyển đổi SO(3) → Euler
- `compute_iou()` — Tính Intersection over Union

---

## 7. hardware/ — Entry Point Phần cứng

### 7.1 `main_aruco_detector.py` — Entry Point Embedded

**Chức năng**: Script khởi động chính khi deploy lên Companion Computer. Orchestrate:
1. Khởi động RealSense stream
2. Chạy ArUco detector với calibration file
3. Chạy EKF adapter
4. Publish ROS 2 topics cho các node phía sau

```bash
python3 hardware/main_aruco_detector.py \
    --calibration /path/to/calibration.yaml \
    --marker-size 0.895 \
    --verbose
```

---

## 8. ROS 2 Topics & TF Tree đầy đủ

### Topics

```
# Sensor / Bridge
/camera                         [sensor_msgs/Image]          RGB 640×480 từ Gazebo/RealSense
/camera_info                    [sensor_msgs/CameraInfo]     Camera intrinsics
/depth_camera                   [sensor_msgs/Image]          Depth 32FC1
/depth_camera/image_mono        [sensor_msgs/Image]          Depth mono8 (cho RViz2)
/depth_camera/points            [sensor_msgs/PointCloud2]    Pointcloud sensor
/odom                           [nav_msgs/Odometry]          Drone odometry (world frame)

# Perception — ArUco
/hpad/detected                  [std_msgs/Bool]              Cờ phát hiện marker
/hpad/position_camera           [geometry_msgs/PointStamped] Vị trí 3D (camera_optical_frame)
/hpad/bbox                      [geometry_msgs/Polygon]      4 đỉnh bounding box (pixels)
/hpad/pose                      [geometry_msgs/PoseStamped]  Pose 6-DOF marker
/hpad/annotated                 [sensor_msgs/Image]          Ảnh debug overlay
/hpad/ground_truth              [geometry_msgs/PointStamped] Ground truth từ Gazebo

# State Estimation — EKF
/ekf/target_state               [nav_msgs/Odometry]          Trạng thái sau lọc (world frame)
/ekf/tracking_mode              [std_msgs/String]            Mode: IDLE/TRACKING/DEAD_RECKONING/EXPIRED
/ekf/predicted_marker           [geometry_msgs/PointStamped] Dự đoán vị trí marker

# Planning — APF
/apf/velocity_cmd               [geometry_msgs/TwistStamped] Lệnh vận tốc né vật cản
/apf/target_position            [geometry_msgs/PointStamped] Điểm bám mục tiêu APF
/apf/forces_debug               [geometry_msgs/Vector3Stamped] Lực hút/đẩy debug
/map_generator/global_cloud     [sensor_msgs/PointCloud2]    Map obstacle cho RViz2

# Control — IBVS
/ibvs/gimbal_pitch              [std_msgs/Float64]           Lệnh góc pitch gimbal (rad)
/ibvs/yaw_cmd                   [std_msgs/Float64]           Lệnh tốc độ yaw (rad/s)
/ibvs/tracking_error            [geometry_msgs/Vector3]      Pixel error (e_u, e_v, 0)

# Precision Landing — Guidance, Covariance & Touchdown
/landing/safe_to_land           [std_msgs/Bool]              Cờ cho phép hạ cánh từ Covariance Gate
/landing/uncertainty_radius     [std_msgs/Float64]           Bán kính không định vị trí tương đối 2σ (m)
/landing/rswitch_adaptive       [std_msgs/Float64]           Bán kính chuyển mạch dốc trượt thích ứng (m)
/landing/sliding_weight         [std_msgs/Float64]           Trọng số mặt trượt ws
/landing/covariance_status      [diagnostic_msgs/DiagnosticStatus] Chẩn đoán ma trận hiệp phương sai
/landing/velocity_cmd           [geometry_msgs/Twist]        Lệnh vận tốc dẫn đường 3D từ SMC
/landing/sub_phase              [std_msgs/String]            Pha SMC: GLIDE_SLOPE | FINAL_DESCENT
/landing/touchdown              [std_msgs/Bool]              Tín hiệu chốt tiếp đất (Latched True)
/landing/touchdown_status       [diagnostic_msgs/DiagnosticStatus] Trạng thái bộ dò tiếp đất

# Mission — FSM
/mission/phase                  [std_msgs/String]            IDLE/SEARCH/FOLLOW/APPROACH/GLIDE_SLOPE/LAND/TOUCHDOWN
/mission/velocity_setpoint      [geometry_msgs/Twist]        Vận tốc tổng hợp → PX4
/mission/yaw_setpoint           [std_msgs/Float64]           Yaw mục tiêu → PX4
/mission/waveoff_event          [std_msgs/String]            Cảnh báo hủy hạ cánh (Wave-off)
/mission/status                 [std_msgs/String]            Status log

# Operator
/operator/land_command          [std_msgs/Bool]              Lệnh kích hoạt hạ cánh chính xác
```

### TF2 Tree

```
world
  └── base_link              (Thân drone, gắn với /odom)
        └── camera_link      (Giá đỡ gimbal)
              └── camera_optical_frame   (Trục quang học: Z-forward, X-right, Y-down)
```

---

## 9. Tham số cấu hình đầy đủ

File: [`simulation/config/mission_params.yaml`](simulation/config/mission_params.yaml)

### `apf_planner`
| Tham số | Mặc định | Đơn vị | Mô tả |
|---|---|---|---|
| `d0` | `2.5` | m | Bán kính ảnh hưởng vật cản |
| `v_max` | `1.5` | m/s | Vận tốc output tối đa |
| `d_slow` | `1.2` | m | Bán kính giảm tốc khi gần goal |
| `target_altitude` | `3.0` | m | Độ cao bay FOLLOW |
| `k_att` | `10.0` | — | Hệ số lực hút |
| `k_rep` | `2500.0` | — | Hệ số lực đẩy (FOLLOW) |
| `k_rep_approach` | `125.0` | — | Hệ số lực đẩy (APPROACH, giảm để tiếp cận) |
| `follow_distance` | `3.5` | m | Cự ly bám mục tiêu |
| `hold_follow_altitude` | `true` | bool | Giữ độ cao trong FOLLOW |
| `target_lead_time` | `0.35` | s | Dự báo vị trí target trước |

### `covariance_gate`
| Tham số | Mặc định | Đơn vị | Mô tả |
|---|---|---|---|
| `max_uncertainty_enter_glide_2sigma_m` | `0.45` | m | Ngưỡng 2σ tối đa để vào GLIDE_SLOPE |
| `max_uncertainty_continue_glide_2sigma_m` | `0.50` | m | Ngưỡng 2σ tối đa để duy trì GLIDE_SLOPE |
| `final_uncertainty_max_2sigma_m` | `0.35` | m | Ngưỡng 2σ tối đa để vào FINAL_DESCENT |
| `r_switch_min` | `4.5` | m | Bán kính chuyển mạch dốc trượt tối thiểu |
| `rswitch_default` | `5.0` | m | Bán kính chuyển mạch dốc trượt mặc định |
| `r_switch_max` | `10.0` | m | Bán kính chuyển mạch dốc trượt tối đa |
| `conical_gate_angle_deg` | `30.0` | deg | Góc nửa nón an toàn của phễu tiếp cận |

### `landing_guidance` (SMC)
| Tham số | Mặc định | Đơn vị | Mô tả |
|---|---|---|---|
| `ka` | `0.20` | s⁻¹ | Hệ số tốc độ khép cự ly mặt trượt S1 |
| `kb` | `0.60` | s⁻¹ | Hệ số tốc độ khép góc dốc mặt trượt S2 |
| `kc` | `0.40` | s⁻¹ | Hệ số tốc độ khép phương vị mặt trượt S3 |
| `k1` / `k2` / `k3` | `0.1395 / 0.1784 / 0.0442` | — | Gains reaching law cho [S1, S2, S3] |
| `m` / `n` | `5.0 / 3.0` | — | Lũy thừa Power Reaching (0.6) |
| `theta_des_deg` | `45.0` | deg | Góc tiếp cận dốc trượt chuẩn |
| `v_descend_glide` | `0.35` | m/s | Tốc độ hạ tối đa trong GLIDE_SLOPE |
| `v_descend_touch` | `0.15` | m/s | Tốc độ hạ êm ái khi tiếp đất |
| `final_gate_wait_alt` | `0.40` | m | Độ cao chuyển sang FINAL_DESCENT |

### `touchdown_detector`
| Tham số | Mặc định | Đơn vị | Mô tả |
|---|---|---|---|
| `optical_height_threshold_m` | `0.40` | m | Độ cao quang học tiếp đất |
| `descent_cmd_threshold_mps` | `-0.10` | m/s | Ngưỡng lệnh vận tốc hạ yêu cầu |
| `stoppage_vz_max_mps` | `0.08` | m/s | Vận tốc đứng tối đa khi đã tiếp sàn |
| `confirmation_duration_s` | `0.35` | s | Thời gian xác nhận trước khi chốt tiếp đất |

### `ibvs_controller`
| Tham số | Mặc định | Đơn vị | Mô tả |
|---|---|---|---|
| `K_pitch` | `0.92` | — | Gain điều khiển gimbal pitch |
| `K_yaw` | `0.92` | — | Gain điều khiển yaw |
| `focal_x` / `focal_y` | `466.0 / 466.0` | px | Tiêu cự camera |
| `pitch_rate_limit` | `1.80` | rad/s | Tốc độ thay đổi pitch tối đa |
| `yaw_rate_limit` | `0.80` | rad/s | Tốc độ quay yaw tối đa |
| `pitch_ema_alpha` | `0.65` | — | Hệ số lọc mượt gimbal pitch |
| `v0` | `240.0` | px | Tâm ảnh Y (nửa height 480) |
| `search_yaw_rate` | `0.20` | rad/s | Tốc độ quét yaw khi SEARCH |
| `pitch_rate_limit` | `1.5` | rad/s | Giới hạn tốc độ pitch |
| `yaw_rate_limit` | `0.50` | rad/s | Giới hạn tốc độ yaw |
| `pitch_ema_alpha` | `0.25` | — | EMA lọc gimbal pitch |

### `mission_fsm`
| Tham số | Mặc định | Đơn vị | Mô tả |
|---|---|---|---|
| `follow_distance` | `3.5` | m | Cự ly bám mục tiêu |
| `alignment_threshold` | `0.3` | m | Ngưỡng căn chỉnh hạ cánh |
| `land_altitude` | `0.5` | m | Độ cao kích hoạt LAND |
| `search_timeout` | `1.2` | s | Timeout EKF EXPIRED → SEARCH |
| `approach_timeout` | `3.0` | s | Timeout mất ArUco trong APPROACH |
| `takeoff_altitude` | `3.0` | m | Độ cao cất cánh |
| `yaw_align_enable` | `true` | bool | Bật căn chỉnh Yaw trước FOLLOW |
| `reacquire_confirm_time` | `0.25` | s | Thời gian confirm tái nhận diện |
| `reacquire_hold_timeout` | `4.0` | s | Timeout giữ vị trí tái nhận diện |

---

## 10. Hướng dẫn test từng module độc lập

### 10.1 Test EKF (không cần Gazebo, không cần ROS)

```bash
cd /home/duy/VDT_project
python3 vision/run_target_state_simulation.py
# Output: simulation_results/target_state_ekf/ (PNG plots + metrics.json)
```

So sánh 3 motion models (CV, CT, IMM):
```bash
python3 vision/run_target_state_simulation.py --compare-models
# Output: simulation_results/target_state_comparison/
```

### 10.2 Test APF Core (không cần Gazebo, không cần ROS)

```bash
python3 -c "
from simulation.control.apf_planner import APFCore, APFParams, load_cylinder_obstacles_from_sdf, make_follow_goal
import numpy as np

planner = APFCore(APFParams(d0=2.0, v_max=1.5, k_att=10.0, k_rep=250.0))
obs = load_cylinder_obstacles_from_sdf('/home/duy/VDT_project/PX4-Autopilot/Tools/simulation/gz/worlds/obstacle_avoidance.sdf')
print(f'✅ Loaded {len(obs)} obstacles from SDF')

p_drone  = np.array([2.0, 0.0, 3.0])
p_target = np.array([6.0, 0.0, 0.0])
p_follow = make_follow_goal(p_drone, p_target, follow_distance=3.5)
result   = planner.compute(p_drone, p_follow, obs)
print(f'✅ APF velocity: {np.round(result.velocity, 3)} m/s')
"
```

### 10.3 Test ArUco Detector (không cần Gazebo, cần OpenCV)

```bash
cd /home/duy/VDT_project
python3 -m pytest tests/test_aruco_detector.py -v
```

### 10.4 Test toàn bộ unit tests

```bash
cd /home/duy/VDT_project
source /opt/ros/humble/setup.bash
python3 -m pytest tests/ -v --tb=short
```

### 10.5 Chạy IBVS test C3 (test tích hợp IBVS + EKF)

```bash
source /opt/ros/humble/setup.bash
python3 tests/test_c3_ibvs.py
```

### 10.6 Test IBVS cô lập (không chạy APF, không chạy FSM)

Khởi động Terminal 1 (PX4+Gazebo) và Terminal 2 (Launcher không có `--autonomous`), rồi:

```bash
# Terminal 3: Drone cất cánh
python3 simulation/core/takeoff.py --alt 3.0

# Terminal 4: Detector + EKF
source /opt/ros/humble/setup.bash
python3 simulation/perception/aruco_sim_node.py &
python3 simulation/perception/ekf_ros_adapter.py

# Terminal 5: IBVS + Offboard (không có FSM)
source /opt/ros/humble/setup.bash
python3 simulation/control/ibvs_controller.py &
python3 simulation/core/offboard_commander.py

# Đặt mission phase giả lập là FOLLOW để IBVS hoạt động:
ros2 topic pub /mission/phase std_msgs/msg/String "{data: 'FOLLOW'}" --rate 5

# Terminal 6: H-Pad manual control
python3 simulation/utils/hpad_keyboard_controller.py
```

> ✅ **Tiêu chí PASS cho IBVS test**: Drone xoay yaw mượt bám theo H-Pad, gimbal pitch biến thiên trong `[-75°, -15°]`, marker không trôi ra khỏi rìa FOV khi H-Pad di chuyển tốc độ ≤ 1.5 m/s.
