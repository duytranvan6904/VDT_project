# HƯỚNG DẪN MÔ PHỎNG HỆ THỐNG TỰ HÀNH (AUTONOMOUS PIPELINE)
## PX4 SITL, Gazebo Harmonic, ROS 2 Humble & Thuật Toán APF + IBVS + EKF

> **Dự án**: Quadrotor H-Pad Tracking, Obstacle Avoidance & Precision Landing  
> **Cảm biến tích hợp**: RealSense D430 (Depth-only / IR1 Stereo stream) + Gimbal 1-trục (Pitch)  
> **Môi trường**: Ubuntu 22.04 LTS, ROS 2 Humble, PX4 Autopilot v1.14+, Gazebo Harmonic  
> **Cập nhật**: 15/09/2026  

---

## 📑 MỤC LỤC
1. [Tổng Quan Kiến Trúc & Luồng Dữ Liệu](#1-tổng-quan-kiến-trúc--luồng-dữ-liệu)
2. [Chi Tiết Chức Năng Các File Mã Nguồn](#2-chi-tiết-chức-năng-các-file-mã-nguồn)
3. [Quy Trình Kiểm Thử Mô Phỏng Từng Bước (Terminal Runbook)](#3-quy-trình-kiểm-thử-mô-phỏng-từng-bước-terminal-runbook)
   - [Cách 1: Chạy Tự Động Hoá Hoàn Toàn (Khuyên Dùng)](#cách-1-chạy-tự-động-hoá-hoàn-toàn-autonomous-pipeline---khuyên-dùng)
   - [Cách 2: Chạy Từng Node Riêng Biệt (Debug & Chẩn Đoán)](#cách-2-chạy-từng-node-riêng-biệt-manual--debugging)
   - [Cách 3: Kiểm Thử Unit Test Độc Lập (Không cần Gazebo)](#cách-3-kiểm-thử-unit-test-độc-lập-chạy-ngay-trên-terminal)
4. [Bảng Tiêu Chí Nghiệm Thu (Verification & Acceptance Criteria)](#4-bảng-tiêu-chí-nghiệm-thu-verification--acceptance-criteria)
5. [Danh Mục ROS 2 Topics, Message Types & TF Tree](#5-danh-mục-ros-2-topics-message-types--tf-tree)
6. [Hướng Dẫn Khắc Phục Sự Cố (Troubleshooting)](#6-hướng-dẫn-khắc-phục-sự-cố-troubleshooting)

---

## 1. Tổng Quan Kiến Trúc & Luồng Dữ Liệu

Hệ thống được thiết kế theo mô hình điều khiển phân tầng (Hierarchical Architecture), cho phép Quadrotor tự động phát hiện mục tiêu di động (H-Pad dán mã ArUco ID 42), tính toán quỹ đạo tránh né chướng ngại vật theo thời gian thực (APF), bám giữ mục tiêu trong trường nhìn camera (IBVS) và chuyển đổi các trạng thái bay thông minh (FSM):

```mermaid
graph TD
    subgraph "Sensing & Vision"
        GZ[Gazebo Harmonic: x500_depth] -->|RGB / IR & Depth| BRIDGE[ros_gz_bridge]
        BRIDGE -->|/camera, /depth_camera| ARUCO[aruco_sim_node.py]
        ARUCO -->|/hpad/position_camera| TF_EKF[ekf_ros_adapter.py]
        TF[TF2: world -> camera_optical_frame] -.->|Transform| TF_EKF
    end

    subgraph "State Estimation"
        TF_EKF -->|/ekf/target_state| APF[apf_planner.py]
        TF_EKF -->|/ekf/target_state| FSM[mission_fsm_node.py]
        ARUCO -->|/hpad/bbox| IBVS[ibvs_controller.py]
    end

    subgraph "Planning & Control"
        SDF[obstacle_avoidance.sdf] -.->|Vị trí vật cản tĩnh| APF
        BRIDGE -->|/odom| APF
        APF -->|/apf/velocity_cmd| FSM
        IBVS -->|/ibvs/gimbal_pitch| GZ_GIMBAL[/model/x500_depth_0/command/gimbal_pitch]
        IBVS -->|/ibvs/yaw_cmd| FSM
        FSM -->|/mission/velocity_setpoint| OFFBOARD[offboard_commander.py]
        FSM -->|/mission/yaw_setpoint| OFFBOARD
    end

    subgraph "PX4 Vehicle Interface"
        OFFBOARD -->|Setpoint Trajectory / SIM-ONLY Log| PX4[PX4 Autopilot SITL]
        RC[Operator / RC Switch] -->|/operator/land_command| FSM
    end
```

### Nguyên lý hoạt động các pha:
1. **IDLE / SEARCH**: Drone cất cánh lên độ cao mặc định 3.0 m. Gimbal pitch giữ ngang (-15°). Drone xoay chậm quanh trục Yaw tìm kiếm H-Pad.
2. **FOLLOW**: Khi phát hiện ArUco ≥ 5 frame liên tiếp, EKF chuyển sang trạng thái TRACKING. APF hút drone về điểm đứng cách H-Pad $d_{follow}=3.5$ m theo phương ngang (không bay vào tâm H-Pad), đồng thời giữ độ cao hiện tại; lực đẩy từ các trụ được cộng để né vật cản. IBVS điều khiển gimbal pitch và yaw có giới hạn tốc độ để giữ marker trong FOV.
3. **APPROACH**: Nhận lệnh hạ cánh từ Operator (`/operator/land_command`), FSM giảm dần độ cao drone từ 3.0 m xuống 0.5 m, lực né vật cản được giảm dần để ưu tiên tiếp cận bãi đáp.
4. **LAND**: Khi drone cách H-Pad $< 0.3$ m, drone hạ cánh thẳng đứng lên bề mặt H-Pad và ngắt động cơ (Disarm).

---

## 2. Chi Tiết Chức Năng Các File Mã Nguồn

Thư mục `simulation_maps/` chứa toàn bộ mã nguồn phục vụ mô phỏng và điều khiển tự hành:

| Tên File | Vai Trò & Chức Năng Chi Tiết | Đầu Vào (Inputs) | Đầu Ra (Outputs) |
|---|---|---|---|
| **`config/mission_params.yaml`** | File tham số ROS 2 tập trung cho toàn bộ hệ thống: cự ly bám $d_{follow}=3.5$ m, giữ độ cao FOLLOW, vận tốc tối đa, hệ số APF ($k_{att}, k_{rep}, d_0$), camera FOV, ngưỡng FSM, timeout mất dấu. | File YAML tĩnh | Được launcher nạp cho từng ROS 2 node |
| **`apf_planner.py`** | Bộ quy hoạch quỹ đạo Artificial Potential Field (APF 3D) chuẩn hóa từ thuật toán gốc MATLAB (`Avoidance/APFplanner_1_Obstacle.m`). Tính toán lực hút mục tiêu, lực đẩy vật cản hình trụ nạp từ SDF, bão hòa vận tốc $v_{max}$, và cơ chế chống kẹt bẫy cục bộ (Local Minima Escape). | `/ekf/target_state`<br>`/odom`<br>World SDF obstacles | `/apf/velocity_cmd`<br>`/apf/target_position`<br>`/apf/forces_debug` |
| **`ekf_ros_adapter.py`** | Cầu nối ước lượng trạng thái: Lấy tọa độ tương đối từ camera, tra cứu TF2 (`world -> camera_optical_frame`) chuyển sang tọa độ toàn cục `world`, đưa vào bộ lọc Kalman mở rộng EKF 6-state ($x, y, z, v_x, v_y, v_z$). Tự động chạy chế độ Dead Reckoning (dự đoán quán tính) khi mục tiêu bị che khuất tạm thời. | `/hpad/position_camera`<br>`/hpad/detected`<br>TF2 Transform | `/ekf/target_state`<br>`/ekf/tracking_mode`<br>`/ekf/predicted_marker` |
| **`ibvs_controller.py`** | Bộ điều khiển thị giác hình ảnh (Image-Based Visual Servoing): Điều khiển góc pitch của camera gimbal thông qua topic Gazebo và tốc độ quay Yaw của drone nhằm triệt tiêu độ lệch pixel của marker so với tâm ảnh ($e_u, e_v \to 0$). | `/hpad/bbox`<br>`/hpad/position_camera` | `/ibvs/gimbal_pitch`<br>`/ibvs/yaw_cmd`<br>`/ibvs/tracking_error` |
| **`mission_fsm_node.py`** | Máy trạng thái hữu hạn quản lý nhiệm vụ (FSM): Thực thi chuyển pha `IDLE -> SEARCH -> FOLLOW -> APPROACH -> LAND -> EMERGENCY_HOVER`. Điều phối nguồn lệnh vận tốc và góc yaw an toàn tới bộ điều khiển bay. | `/ekf/tracking_mode`<br>`/apf/velocity_cmd`<br>`/ibvs/yaw_cmd`<br>`/operator/land_command` | `/mission/phase`<br>`/mission/velocity_setpoint`<br>`/mission/yaw_setpoint`<br>`/mission/status` |
| **`offboard_commander.py`** | Giao tiếp với PX4 Autopilot qua chế độ Offboard: Chuyển đổi hệ tọa độ ENU (ROS) sang NED (PX4). Hỗ trợ chế độ **SIM-ONLY** (tự động kích hoạt khi chưa cài `px4_msgs`, ghi log setpoint kiểm thử) và chế độ chuẩn PX4 MicroXRCE-DDS. | `/mission/velocity_setpoint`<br>`/mission/yaw_setpoint` | PX4 TrajectorySetpoint hoặc console log chẩn đoán |
| **`launch_simulation.py`** | Node Master điều phối mô phỏng: Tự động phát hiện phiên bản bridge, khởi tạo các cầu nối `ros_gz_bridge`, broadcast TF liên tục, mở giao diện RViz2. Hỗ trợ cờ `--autonomous` để tự động khởi động đồng thời cả 5 node tự hành trên. | Tham số dòng lệnh | Quản lý vòng đời toàn bộ tiến trình mô phỏng |
| **`aruco_sim_node.py`** | Node xử lý ảnh thị giác: Nhận ảnh từ RealSense D430 trong Gazebo, giải mã ArUco ID 42 bằng OpenCV, giải bài toán SolvePnP để ước lượng vị trí mục tiêu 3D theo `camera_optical_frame`. | `/camera`<br>`/camera_info`<br>`/depth_camera` | `/hpad/detected`<br>`/hpad/pose`<br>`/hpad/position_camera`<br>`/hpad/bbox`<br>`/hpad/annotated` |
| **`hpad_keyboard_controller.py`** | Công cụ điều khiển bãi đáp H-Pad di động phục vụ thử nghiệm: Lắng nghe phím bàn phím (W, A, S, D, SPACE, R), gọi service Gazebo Harmonic `/world/obstacle_avoidance/set_pose` để di chuyển H-Pad với vận tốc định trước, đồng thời publish Ground Truth. | Phím bấm bàn phím | `/hpad/ground_truth`<br>Gazebo Entity Pose |
| **`depth_to_image_node.py`** | Tiện ích chuẩn hoá depth image: Chuyển đổi dữ liệu độ sâu thô dạng 32FC1 từ Gazebo sang ảnh thang độ xám `mono8` để quan sát trực tiếp trên RViz2. | `/depth_camera` | `/depth_camera/image_mono` |
| **`drone_rviz_visualizer.py`** | Hiển thị trực quan mô hình 3D chiếc Quadrotor trong RViz2 theo dữ liệu `/odom`. | `/odom` | `/drone/marker` |
| **`sync_rviz_gazebo_map.py`** | Đọc file SDF thế giới Gazebo, tạo ra bản đồ PointCloud2 toàn cục tương ứng trong RViz2. | World SDF file | `/map_generator/global_cloud` |

---

## 3. Quy Trình Kiểm Thử Mô Phỏng Từng Bước (Terminal Runbook)

### Yêu cầu chuẩn bị môi trường:
- Đã cài đặt ROS 2 Humble và Gazebo Harmonic (`ros-humble-ros-gzharmonic-bridge`).
- Đường dẫn repository: `/home/duy/VDT_project`.

---

### Cách 1: Chạy Tự Động Hoá Hoàn Toàn (Autonomous Pipeline - KHUYÊN DÙNG)

Quy trình chuẩn gồm 5 Terminal mở theo thứ tự:

```
[Terminal 0] Sinh map (chỉ chạy 1 lần đầu)
      ↓
[Terminal 1] PX4 SITL + Gazebo Harmonic
      ↓
[Terminal 2] Master Launcher (--autonomous)  <-- Tự động chạy APF + EKF + IBVS + FSM + Offboard
      ↓
[Terminal 3] QGroundControl (Takeoff & giám sát)
      ↓
[Terminal 4] ArUco Vision Node
      ↓
[Terminal 5] H-Pad Keyboard Controller (Lái bãi đáp kiểm thử né vật cản)
```

#### Bước 0: Tạo World SDF (Chỉ cần chạy 1 lần nếu chưa tạo)
```bash
# Terminal 0
source /opt/ros/humble/setup.bash
python3 /home/duy/VDT_project/simulation_maps/launch_simulation.py --generate-world
```

#### Bước 1: Khởi động PX4 SITL & Gazebo Sim
```bash
# Terminal 1
cd /home/duy/VDT_project/PX4-Autopilot
export GZ_PARTITION=vdt_harmonic
export GZ_SIM_RESOURCE_PATH="$PWD/Tools/simulation/gz/models:${GZ_SIM_RESOURCE_PATH:-}"
PX4_GZ_WORLD=obstacle_avoidance make px4_sitl gz_x500_depth
```
> *Chờ cửa sổ Gazebo mở hoàn tất, model chiếc drone `x500_depth_0` xuất hiện ở tọa độ `(0, 0, 0)` và H-Pad xuất hiện ở `(5.0, 2.0, 0.02)`. Giữa drone và H-Pad đã có sẵn các trụ chướng ngại vật đỏ/xanh để kiểm thử né vật cản ngay lập tức.*

#### Bước 2: Khởi động Master Launcher với chế độ tự hành
```bash
# Terminal 2
source /opt/ros/humble/setup.bash
export GZ_PARTITION=vdt_harmonic
python3 /home/duy/VDT_project/simulation_maps/launch_simulation.py --autonomous
```
> **Dấu hiệu thành công tại Terminal 2**:
> - Khởi động đồng thời: TF broadcaster, `ros_gz_bridge`, RViz2.
> - Camera tự động chúc xuống **-30°** ngay khi khởi tạo.
> - Khởi động 5 node con: `[AUTONOMOUS] Started ekf_ros_adapter`, `apf_planner`, `ibvs_controller`, `mission_fsm`, `offboard_commander`.
> - `offboard_commander` kết nối MAVLink UDP tới PX4 SITL (`udp:127.0.0.1:14540`).
> - Định kỳ mỗi giây in log EKF & FSM: `[STATUS] Phase: SEARCH/FOLLOW | Drone: (...) | Target: (...) | Dist: ...`.

#### Bước 3: Cất cánh Drone lên độ cao 3.0 m (Khuyên dùng script hoặc PX4 Shell)
Do giao diện thanh trượt Takeoff của QGroundControl bị **hardcode chặn cứng mức tối thiểu là 10.0m** trong mã nguồn QML để an toàn ngoài đời thực, bạn hãy dùng 1 trong 2 cách sau để cất cánh chuẩn 3.0m:

* **Cách A (Thuận tiện nhất - Chạy script Python MAVLink)**:
  Mở terminal và chạy:
  ```bash
  python3 /home/duy/VDT_project/simulation_maps/takeoff.py --alt 3.0
  ```
  *(Script sẽ tự động Arm động cơ và ra lệnh cho drone bay lên đúng 3.0m rồi hover ổn định).*

* **Cách B (Gõ trực tiếp vào Terminal 1 - PX4 Shell)**:
  Ngay tại terminal đang chạy PX4 SITL (Terminal 1), tại dấu nhắc lệnh `pxh>`, gõ:
  ```text
  param set MIS_TAKEOFF_ALT 3.0
  commander takeoff
  ```

* **Giám sát trên QGroundControl (Terminal 3)**:
  ```bash
  # Terminal 3 (Chỉ dùng để quan sát thông số và hình ảnh 3D)
  ~/QGroundControl.AppImage
  ```
  *(Drone bay lên $> 1.8$m $\to$ Camera $-30^\circ$ thấy bãi đáp $\to$ Mission FSM tự chuyển sang `FOLLOW` và `offboard_commander` tự động chiếm quyền OFFBOARD điều khiển APF né vật cản).*

#### Bước 4: Khởi động Node nhận diện thị giác ArUco
```bash
# Terminal 4
source /opt/ros/humble/setup.bash
cd /home/duy/VDT_project
python3 simulation_maps/aruco_sim_node.py
```
> **Dấu hiệu thành công tại Terminal 4**:
> - In thông số camera: `Camera intrinsics loaded: fx=... fy=...`.
> - Khi H-Pad nằm trong tầm quét, in liên tục:
>   `[DETECTION] ID 42 detected | camera_optical xyz=(x, y, z) m`.
> - Xuất ảnh vẽ khung bounding box lên topic `/hpad/annotated`.

#### Bước 5: Điều khiển H-Pad di động để kiểm thử Bám & Né vật cản
```bash
# Terminal 5
source /opt/ros/humble/setup.bash
cd /home/duy/VDT_project
python3 simulation_maps/hpad_keyboard_controller.py --speed 1.5
```
> **Bảng phím điều khiển H-Pad**:
> - `W` / `S`: Di chuyển H-Pad tiến tới / lùi lại theo trục X.
> - `A` / `D`: Di chuyển H-Pad sang trái / phải theo trục Y.
> - `SPACE`: Phanh dừng khẩn cấp H-Pad.
> - `R`: Đặt lại H-Pad về vị trí ban đầu `(4.0, 0.0)`.
> - `+` / `-`: Tăng / giảm tốc độ di chuyển.
>
> **Kịch bản kiểm thử né vật cản**:
> Lái H-Pad đi xuyên qua cụm các trụ hình trụ tròn (obstacles). Quan sát drone:
> 1. Drone tự động bám theo H-Pad ở cự ly an toàn khoảng 3.5 m.
> 2. Khi gặp trụ vật cản, lực đẩy APF đẩy drone lệch sang một bên để tránh va chạm.
> 3. Sau khi vượt qua vật cản, drone quay trở lại hướng thẳng về phía H-Pad.

#### Bước 6 (Tùy chọn): Kích hoạt lệnh hạ cánh chính xác (Operator Land)
Khi muốn kiểm thử pha hạ cánh, mở thêm một terminal hoặc gửi lệnh ROS 2:
```bash
# Terminal 6
source /opt/ros/humble/setup.bash
ros2 topic pub --once /operator/land_command std_msgs/msg/Bool "{data: true}"
```
> **Hiện tượng quan sát**:
> - FSM chuyển sang pha `APPROACH`, drone giảm dần độ cao.
> - Gimbal camera tự động tilt chúi xuống sâu hơn (-45° đến -75°) để không mất dấu marker.
> - Khi cự ly đủ gần ($< 0.3$ m), FSM kích hoạt pha `LAND` hạ cánh an toàn lên mặt phẳng H-Pad.

---

### Cách 2: Chạy Từng Node Riêng Biệt (Manual & Debugging)

Khi cần phát triển hoặc debug thuật toán chi tiết từng module, bạn không dùng cờ `--autonomous` tại Terminal 2, mà khởi chạy từng node trong các terminal độc lập:

```bash
# Terminal 2A: Launcher cơ sở (Chỉ mở Bridge + TF + RViz2)
python3 /home/duy/VDT_project/simulation_maps/launch_simulation.py

# Terminal 2B: EKF Adapter (Ước lượng vị trí & vận tốc mục tiêu)
python3 /home/duy/VDT_project/simulation_maps/ekf_ros_adapter.py

# Terminal 2C: APF Planner (Quy hoạch đường bay tránh vật cản)
python3 /home/duy/VDT_project/simulation_maps/apf_planner.py

# Terminal 2D: IBVS Controller (Điều khiển Gimbal Pitch & Yaw)
python3 /home/duy/VDT_project/simulation_maps/ibvs_controller.py

# Terminal 2E: Mission FSM (Máy trạng thái điều phối)
python3 /home/duy/VDT_project/simulation_maps/mission_fsm_node.py

# Terminal 2F: Offboard Commander (Gửi lệnh điều khiển PX4)
python3 /home/duy/VDT_project/simulation_maps/offboard_commander.py
```

---

### Cách 3: Kiểm Thử Unit Test Độc Lập (Chạy ngay trên Terminal)

Để kiểm tra tính đúng đắn toán học của thuật toán APF, EKF và chuyển đổi hệ tọa độ TF2 mà **không cần mở Gazebo/PX4**:

```bash
source /opt/ros/humble/setup.bash
cd /home/duy/VDT_project

# 1. Test toàn bộ giải thuật APF (Lực hút, Lực đẩy, Bão hòa vận tốc, Nạp vật cản từ SDF)
python3 -c "
from simulation_maps.apf_planner import (
    APFCore, APFParams, cylinder_nearest_point,
    load_cylinder_obstacles_from_sdf, make_follow_goal,
)
import numpy as np

planner = APFCore(APFParams(d0=2.0, v_max=1.5, k_att=10.0, k_rep=250.0))
raw_obs = load_cylinder_obstacles_from_sdf('/home/duy/VDT_project/PX4-Autopilot/Tools/simulation/gz/worlds/obstacle_avoidance.sdf')
obs = [cylinder_nearest_point(np.array([2.0, 0.0, 3.0]), *o) for o in raw_obs]
print(f'✅ Nạp thành công {len(obs)} chướng ngại vật từ file world SDF!')

# Test né vật cản và giữ độ cao FOLLOW
p_drone = np.array([2.0, 0.0, 3.0])
p_target = np.array([6.0, 0.0, 0.0])
p_follow = make_follow_goal(p_drone, p_target, follow_distance=3.5)
result = planner.compute(p_drone, p_follow, obs)
print(f'✅ APF velocity: {np.round(result.velocity, 3)} m/s; z={result.velocity[2]:.3f}')
"

# 2. Test thuật toán EKF Core
python3 vision/run_target_state_simulation.py
```

---

## 4. Bảng Tiêu Chí Nghiệm Thu (Verification & Acceptance Criteria)

Để xác nhận hệ thống mô phỏng hoạt động hoàn hảo, kiểm tra đối chiếu các tiêu chí sau:

| STT | Hạng Mục Kiểm Tra | Lệnh Kiểm Tra / Công Cụ | Tiêu Chí Đạt Chuẩn (PASS) |
|---|---|---|---|
| 1 | **Tần số dữ liệu Cảm biến** | `ros2 topic hz /camera`<br>`ros2 topic hz /depth_camera`<br>`ros2 topic hz /odom` | - `/camera`: $\ge 15$ Hz<br>- `/depth_camera`: $\ge 15$ Hz<br>- `/odom`: $\ge 20$ Hz |
| 2 | **Nhận diện ArUco D430** | `ros2 topic echo --once /hpad/position_camera`<br>`ros2 topic hz /hpad/annotated` | - Trả về tọa độ $(x, y, z)$ theo `camera_optical_frame`<br>- Tần số xử lý $\ge 10$ Hz |
| 3 | **Ước lượng EKF Target** | `ros2 topic echo /ekf/tracking_mode`<br>`ros2 topic echo /ekf/target_state` | - `tracking_mode` chuyển thành `TRACKING`<br>- Ước lượng vị trí mượt, covariance hội tụ<br>- Mất dấu marker $< 1.0$s: Tự duy trì dự đoán `DEAD_RECKONING` |
| 4 | **Tính toán Lực APF** | `ros2 topic echo /apf/velocity_cmd` | - Vận tốc $|\vec{v}| \le v_{max} = 2.0$ m/s<br>- Khi drone cách vật cản $< 2.0$ m: Xuất hiện vector né tránh làm lệch hướng |
| 5 | **Điều khiển IBVS Gimbal** | `ros2 topic echo /ibvs/gimbal_pitch` | - Góc pitch biến thiên mượt trong khoảng $[-75^\circ, -15^\circ]$<br>- Giữ marker không bị trôi ra khỏi rìa FOV |
| 6 | **Chuyển trạng thái FSM** | `ros2 topic echo /mission/phase` | - Khởi động: `IDLE` / `SEARCH`<br>- Thấy mục tiêu: Chuyển sang `FOLLOW`<br>- Khi có lệnh land: Chuyển `APPROACH` $\to$ `LAND` |
| 7 | **Khoảng cách An toàn** | Quan sát Gazebo & RViz2 | Drone không va chạm vào bất kỳ hình trụ vật cản nào ($d_{min} \ge 0.8$ m) |
| 8 | **Độ chính xác Hạ cánh** | Quan sát tiếp xúc bề mặt | Drone đáp gọn gàng trên bề mặt tấm H-Pad $0.9 \times 0.9$ m (sai số tâm $< 20$ cm) |

---

## 5. Danh Mục ROS 2 Topics, Message Types & TF Tree

### Danh sách Topic chính trong hệ thống:
```text
/camera                             [sensor_msgs/msg/Image]               Ảnh RGB từ RealSense D430
/depth_camera                       [sensor_msgs/msg/Image]               Ảnh độ sâu thô (32FC1)
/depth_camera/image_mono            [sensor_msgs/msg/Image]               Ảnh độ sâu hiển thị RViz2 (mono8)
/depth_camera/points                [sensor_msgs/msg/PointCloud2]         Đám mây điểm độ sâu 3D
/odom                               [nav_msgs/msg/Odometry]               Odometry drone từ Gazebo
/hpad/position_camera               [geometry_msgs/msg/PointStamped]      Tọa độ H-Pad theo frame camera
/hpad/detected                      [std_msgs/msg/Bool]                   Cờ phát hiện marker ArUco
/hpad/bbox                          [geometry_msgs/msg/Polygon]           Tọa độ 4 đỉnh hộp bao pixel
/hpad/ground_truth                  [geometry_msgs/msg/PointStamped]      Tọa độ thực tế H-Pad từ Gazebo
/ekf/target_state                   [nav_msgs/msg/Odometry]               Trạng thái H-Pad sau lọc EKF (world frame)
/ekf/tracking_mode                  [std_msgs/msg/String]                 Chế độ lọc: IDLE, TRACKING, DEAD_RECKONING
/apf/velocity_cmd                   [geometry_msgs/msg/TwistStamped]      Vận tốc điều khiển né vật cản APF
/ibvs/gimbal_pitch                  [std_msgs/msg/Float64]                Lệnh góc chúi gimbal servo
/ibvs/yaw_cmd                       [geometry_msgs/msg/TwistStamped]      Lệnh tốc độ quay Yaw bám mục tiêu
/mission/phase                      [std_msgs/msg/String]                 Pha hiện tại của FSM
/mission/velocity_setpoint          [geometry_msgs/msg/TwistStamped]      Vận tốc tổng hợp gửi sang PX4
/mission/yaw_setpoint               [std_msgs/msg/Float64]                Góc yaw mục tiêu gửi sang PX4
/operator/land_command              [std_msgs/msg/Bool]                   Lệnh hạ cánh cưỡng bức từ Operator
```

### Cây biến đổi tọa độ (TF2 Tree):
```text
world
  └── base_link (Drone Body)
        └── camera_link (Gimbal Mount)
              └── camera_optical_frame (Trục quang học: Z trước, X phải, Y xuống)
```

---

## 6. Hướng Dẫn Khắc Phục Sự Cố (Troubleshooting)

### 1. Lỗi Gazebo báo `No subscribers` hoặc Bridge không nhận dữ liệu
- **Nguyên nhân**: Phiên bản bridge không tương thích với Gazebo Harmonic (`ros-humble-ros-ign-bridge` thay vì `ros-humble-ros-gzharmonic-bridge`).
- **Khắc phục**:
  ```bash
  sudo apt update && sudo apt install -y ros-humble-ros-gzharmonic-bridge
  ```
- Luôn kiểm tra biến môi trường trước khi chạy launcher:
  ```bash
  export GZ_PARTITION=vdt_harmonic
  ```

### 2. Node EKF báo lỗi `TF lookup failed (world -> camera_optical_frame)`
- **Nguyên nhân**: Chưa bật Master Launcher hoặc dữ liệu Odometry từ Gazebo chưa có publisher.
- **Khắc phục**:
  1. Kiểm tra lại Terminal 1: Đảm bảo model `x500_depth_0` đã xuất hiện trong Gazebo.
  2. Kiểm tra lại Terminal 2: Đảm bảo `launch_simulation.py` đang chạy (node này đảm nhiệm broadcast TF liên tục ở tần số 30 Hz).

### 3. Offboard Commander báo `[SIM-ONLY] px4_msgs not available`
- **Giải thích**: Đây là tính năng dự phòng an toàn (graceful fallback) đã được tích hợp sẵn. Hệ thống vẫn tính toán toàn bộ giải thuật APF, EKF, IBVS, FSM và xuất log chẩn đoán setpoint bình thường mà không gây crash chương trình.
- **Khi muốn kết nối trực tiếp vào PX4 Offboard flight stack thực**: Build workspace có chứa gói `px4_msgs` tương thích với phiên bản PX4 SITL đang chạy.

### 4. ArUco Sim Node không nhận diện được Marker
- **Nguyên nhân**: Góc nghiêng camera hướng lên trời hoặc marker nằm ngoài FOV; hoặc kích thước marker cấu hình sai.
- **Khắc phục**:
  1. Đặt lại vị trí H-Pad về ngay phía trước drone bằng phím `R` trong `hpad_keyboard_controller.py`.
  2. Điều chỉnh góc camera chúc xuống thủ công để kiểm tra:
     ```bash
     ros2 topic pub --once /model/x500_depth_0/command/gimbal_pitch std_msgs/msg/Float64 "{data: -0.5}"
     ```
  3. Quan sát topic `/hpad/annotated` trên RViz2 để xem trực tiếp khung hình nhận diện.

### 5. Drone quay liên tục hoặc mất tracking trong FOLLOW
- APF hiện bám một điểm cách H-Pad 3.5 m, không hút drone đi xuyên vào target.
- Yaw FOLLOW đi theo sai số góc ngắn nhất và bị giới hạn tốc độ `yaw_rate_limit=0.6 rad/s`, tránh bước nhảy tại biên `+/-π`.
- Kiểm tra đồng thời:
  ```bash
  ros2 topic echo /apf/velocity_cmd
  ros2 topic echo /ibvs/yaw_cmd
  ros2 topic echo /ekf/tracking_mode
  ```
  Nếu `/ibvs/yaw_cmd` thay đổi đều nhưng `/ekf/tracking_mode` chuyển `EXPIRED`, giảm `ibvs.yaw_rate_limit` hoặc kiểm tra lại FOV/gimbal. Nếu `/apf/velocity_cmd` có `linear.z` khác 0 trong FOLLOW, kiểm tra `hold_follow_altitude` trong `mission_params.yaml`.

### 6. APF làm thay đổi độ cao trước khi landing
- Trong FOLLOW, điểm đích APF được đặt tại cùng độ cao hiện tại của drone; pha APPROACH mới cho phép lệnh hạ độ cao.
- Đây là giữ độ cao đối với lực hút target. Lực đẩy vật cản 3D vẫn có thể tạo thành phần `z` nếu drone ở sát phần trên của trụ; hãy kiểm tra `linear.z` trên `/apf/velocity_cmd` khi bắt đầu test vật cản.
