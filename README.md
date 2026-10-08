# VDT Quadrotor Autonomous Tracking System

> **Dự án**: Quadrotor H-Pad Tracking, Obstacle Avoidance & Precision Landing  
> **Cảm biến**: Intel RealSense D430 (Depth + IR Stereo) + Gimbal 1-trục (Pitch)  
> **Môi trường mô phỏng**: Ubuntu 22.04 LTS · ROS 2 Humble · PX4 Autopilot v1.14+ · Gazebo Harmonic  
> **Cập nhật**: 09/2026

---

## 📑 Mục lục

1. [Tổng quan hệ thống](#1-tổng-quan-hệ-thống)
2. [Cấu trúc thư mục](#2-cấu-trúc-thư-mục)
3. [Yêu cầu hệ thống & Cài đặt](#3-yêu-cầu-hệ-thống--cài-đặt)
4. [Hướng dẫn chạy Simulation (Gazebo)](#4-hướng-dẫn-chạy-simulation-gazebo)
5. [Hướng dẫn triển khai Hardware](#5-hướng-dẫn-triển-khai-hardware)
6. [Cấu hình tham số](#6-cấu-hình-tham-số)
7. [Xác nhận & Tiêu chí nghiệm thu](#7-xác-nhận--tiêu-chí-nghiệm-thu)
8. [Troubleshooting](#8-troubleshooting)

---

## 1. Tổng quan hệ thống

Hệ thống điều khiển tự hành theo kiến trúc phân tầng, cho phép Quadrotor:
- **Phát hiện** mục tiêu di động (tấm H-Pad ArUco) bằng camera onboard
- **Bám mục tiêu** (IBVS + EKF) kể cả khi mất tín hiệu tạm thời
- **Tránh vật cản** theo thời gian thực (Hỗ trợ **Improved APF 3D** với Dynamic Tangent Escape chống kẹt Local Minima & Standard APF)
- **Hạ cánh chính xác** lên bề mặt H-Pad khi nhận lệnh Operator

### Kiến trúc phân tầng

> Sơ đồ dưới áp dụng cho cả **Simulation và Hardware** — sự khác biệt chỉ nằm ở nguồn ảnh đầu vào và kênh kết nối PX4.

```
┌──────────────── TẦNG NHẬN THỨC (Perception) ───────────────────┐
│  Camera source:                                                  │
│    [Sim] Gazebo ─► ros_gz_bridge ─► /camera topic               │
│    [HW]  RealSense D430/D435i SDK trực tiếp                     │
│                       │                                          │
│                       ▼                                          │
│      Nested Dual-Scale ArUco Board ────────► EKF Adapter        │
│       (aruco_sim_node / aruco_detector)     (ekf_ros_adapter)   │
│         ID 42 (52.5cm) + ID 43 (10cm)                │          │
│                    │                                 │          │
│              /hpad/bbox                     /ekf/target_state   │
└────────────────────┼─────────────────────────────────┼──────────┘
                     │                                 │
┌────────────────────▼── TẦNG ĐIỀU KHIỂN & DẪN ĐƯỜNG (Control) ────┐
│                                                      │           │
│  ibvs_controller ◄───────────────────────────────────┤           │
│   (Gimbal Pitch + Body Yaw + Nadir Tilt -85°)        │           │
│       │ /ibvs/yaw_cmd, /ibvs/gimbal_pitch            │           │
│       │                                              │           │
│  apf_planner (I-APF 3D né vật cản & bám standoff) ──┤           │
│       │ /apf/guidance_velocity_cmd                   │           │
│       │                                              │           │
│  covariance_gate (P_target + P_drone, Cone 30°) ────┤           │
│       │ /landing/safe_to_land, /landing/rswitch_adaptive         │
│       │                                              │           │
│  smc_guidance (SMC 45° Glide Slope & Soft Landing) ──┤           │
│       │ /landing/velocity_cmd                        │           │
│       │                                              │           │
│       └──────────────► mission_fsm_node ◄────────────┘           │
│                              │ /mission/velocity_setpoint        │
│                              ▼                                   │
│                     touchdown_detector ──► /landing/touchdown    │
└──────────────────────────────┼───────────────────────────────────┘
                               │
┌──────────────── TẦNG CHẤP HÀNH (Actuation) ─────────────────────┐
│              offboard_commander  (ENU → NED)                     │
│              MAVLink Force-Disarm (param2=21196.0)               │
│                              │                                   │
│    [Sim]  PX4 SITL ◄── UDP :14540                                │
│    [HW]   PX4 FC   ◄── UART /dev/ttyTHS1                         │
└──────────────────────────────────────────────────────────────────┘
```

### Pipeline trạng thái FSM

```
IDLE ──(takeoff complete)────────► SEARCH ──(stable ArUco + EKF lock)──► FOLLOW
                                     ▲                                      │
                                     │──────(EKF EXPIRED > timeout)─────────┘
                                                                             │
                                                     (LAND command + fresh visual lock)
                                                                             ▼
                                      APPROACH (APF standoff reduction)
                                          │ Covariance 2σ ≤ 0.45m & Cone ≤ 30°
                                          ▼
                                      GLIDE_SLOPE (SMC 45° glide slope)
                                          │ Rxy ≤ 0.40m, altitude ≤ 0.40m, 2σ ≤ 0.35m
                                          ▼
                                      LAND / FINAL_DESCENT (Nadir tilt -85°, vz = -0.15 m/s)
                                          │ Touchdown confirmed (0.35s latch) or z ≤ 0.12m
                                          ▼
                                      TOUCHDOWN ──(MAVLink Force Disarm)──► IDLE
```

- **Nested Dual-Scale ArUco Board**: Tích hợp Marker lớn ID 42 ($52.5\text{cm}$) cho cự ly xa ($1.2\text{m} - 8.0\text{m}$) và Marker nhỏ ID 43 ($10.0\text{cm}$) đồng tâm cho cự ly gần ($0.05\text{m} - 1.5\text{m}$), giải quyết triệt để bài toán tràn khung hình khi áp sát mặt đất.
- **Covariance Gating**: Kết hợp hiệp phương sai vị trí $P_{relative} = P_{target} + P_{drone}$, kiểm tra nón chấp nhận $30^\circ$, điều biến bán kính chuyển mạch dốc trượt thích ứng $R_{switch} \in [4.5\text{m}, 10.0\text{m}]$, tự động wave-off an toàn nếu mất dấu hoặc sai số lớn trên không.
- **SMC Glide Slope & Soft Landing**: Điều khiển bám tâm bãi đáp theo dốc nghiêng $45^\circ$, bổ sung vận tốc hạ thẳng đứng độc lập $\text{alt\_speed} = \max(v_{touch}, 0.20 \cdot r_z)$ triệt tiêu hiện tượng kẹt lơ lửng (Hover Stall).
- **Drift-Invariant Touchdown & Force Disarm**: Kết hợp điều kiện động học ($z \le 0.18\text{m}, |v_z| \le 0.08\text{m/s}$), lệnh hạ $v_{z\_cmd} \le -0.10\text{m/s}$, chốt nhận diện sau $0.35\text{s}$ và gửi MAVLink Force Disarm (`param2=21196.0`) dứt khoát.

Kết quả kiểm chứng thực nghiệm chi tiết xem tại [tracking_follow_status.md](docs/tracking_follow_status.md).

---

## 2. Cấu trúc thư mục

```
VDT_project/
├── README.md                    ← File này: deployment guide
├── MODULES_REFERENCE.md         ← API & chức năng chi tiết từng module
├── requirements.txt
│
├── simulation/                  ← Toàn bộ code pipeline Gazebo simulation
│   ├── core/                    ← Node chính điều phối nhiệm vụ
│   │   ├── mission_fsm_node.py  ← State machine FSM (APPROACH → GLIDE_SLOPE → LAND → TOUCHDOWN)
│   │   ├── offboard_commander.py← Giao tiếp PX4 Offboard + MAVLink Force Disarm
│   │   └── takeoff.py           ← Script cất cánh
│   ├── perception/              ← Nhận thức môi trường & mục tiêu
│   │   ├── aruco_sim_node.py    ← Nested Dual-Scale ArUco detection trong Gazebo
│   │   ├── ekf_ros_adapter.py   ← EKF target state estimator (0.05m - 8m)
│   │   ├── depth_to_image_node.py
│   │   └── mock_target_publisher.py
│   ├── control/                 ← Thuật toán điều khiển & dẫn đường
│   │   ├── ibvs_controller.py   ← IBVS (Gimbal pitch + Body yaw + Nadir tilt -85°)
│   │   ├── apf_planner.py       ← APF / I-APF path planner ROS 2 Node wrapper
│   │   ├── iapf_core.py         ← Lõi Improved APF 3D (GNRON, 3D Tangent, Oscillation Suppression)
│   │   ├── covariance_gate.py   ← Dual-EKF Covariance Fusion & Conical Safety Gate
│   │   ├── smc_guidance.py      ← SMC 45° Glide Slope & Soft Landing Profile
│   │   ├── touchdown_detector.py← Drift-invariant Touchdown Detection & Confirmation Latch
│   │   └── apf_pointcloud_generator.py
│   ├── utils/                   ← Monitoring & visualization tools
│   ├── worlds/                  ← Gazebo world files (.sdf, .world)
│   ├── config/
│   │   └── mission_params.yaml  ← Tham số tập trung toàn hệ thống
│   └── launch/
│       ├── launch_simulation.py ← Master launcher (ĐIỂM VÀO CHÍNH)
│       └── start_px4_sim.sh
│
├── vision/                      ← Code vision cho hardware thực
│   ├── aruco_detector.py        ← Nested Dual-Scale ArUco detector (RealSense thực)
│   ├── realsense_stream.py      ← RealSense D435i interface
│   ├── target_state_ekf.py      ← EKF standalone (không cần ROS)
│   ├── dual_scale_board_config.yaml ← Định nghĩa tọa độ 3D Nested Board
│   └── dual_scale_aruco_board_A0_52_5cm.pdf ← File in bãi đáp chuẩn A0
│
├── hardware/
│   └── main_aruco_detector.py   ← Entry point deploy lên board nhúng
│
├── avoidance/                   ← Prototype MATLAB APF
├── tests/                       ← Unit & Regression tests
│   ├── test_covariance_gate.py  ← Test hiệp phương sai & conical gate
│   ├── test_smc_guidance.py     ← Test SMC sliding surface & reaching law
│   ├── test_dual_scale_board.py ← Test solver Nested ArUco Board
│   ├── test_tracking_control.py ← Test bám mục tiêu & feedforward
│   └── test_iapf_planner.py     ← Test I-APF 3D
├── simulation_results/          ← Benchmark EKF output (offline)
├── docs/                        ← Tài liệu kỹ thuật
│   ├── IBVS_Implementation_Guide.md
│   ├── precision_landing_proposal.tex
│   ├── precision_landing_integration_plan.md
│   └── tracking_follow_status.md
└── References/                  ← Paper & tài liệu tham khảo
```

---

## 3. Yêu cầu hệ thống & Cài đặt

### 3.1 Yêu cầu phần mềm

| Thành phần | Phiên bản | Ghi chú |
|---|---|---|
| Ubuntu | 22.04 LTS | Bắt buộc |
| ROS 2 | Humble Hawksbill | `ros-humble-desktop` |
| Gazebo | Harmonic | `ros-humble-ros-gzharmonic-bridge` |
| PX4 Autopilot | v1.14+ | Submodule tại `PX4-Autopilot/` |
| Python | 3.10+ | Đi kèm Ubuntu 22.04 |
| OpenCV | 4.x | Cho ArUco detection |
| QGroundControl | Daily / v4.2+ | Trạm mặt đất GCS (`~/QGroundControl.AppImage`) |

### 3.2 Cài đặt dependencies

```bash
# 1. Cài ROS 2 Humble
sudo apt update
sudo apt install -y ros-humble-desktop ros-humble-ros-gzharmonic-bridge

# 2. Cài Python dependencies
cd /home/duy/VDT_project
pip install -r requirements.txt

# 3. Source ROS 2 (thêm vào ~/.bashrc để không cần làm lại)
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

### 3.3 Build PX4 lần đầu

```bash
cd /home/duy/VDT_project/PX4-Autopilot
make px4_sitl        # Build PX4 SITL (lần đầu mất ~10 phút)
```

---

## 4. Hướng dẫn chạy Simulation (Gazebo)

> **Khuyến nghị**: Dùng terminal multiplexer như `tmux` để quản lý nhiều terminal song song.

### 4.1 Cách 1: Chạy Pipeline Tự Động Hoàn Toàn (Recommended)

Gồm **6 terminal** mở song song theo thứ tự (kèm 1 terminal tùy chọn):

---

#### 🟦 Terminal 1 — Khởi động PX4 SITL + Gazebo

```bash
cd /home/duy/VDT_project/PX4-Autopilot
export GZ_PARTITION=vdt_harmonic
export GZ_SIM_RESOURCE_PATH="$PWD/Tools/simulation/gz/models:${GZ_SIM_RESOURCE_PATH:-}"
PX4_GZ_WORLD=obstacle_avoidance make px4_sitl gz_x500_depth
```

> ✅ Thành công khi: Cửa sổ Gazebo mở ra, drone `x500_depth_0` tại `(0,0,0)`, H-Pad tại `(3.0, 1.0, 0.02)`, các trụ vật cản đỏ/xanh hiển thị.

---

#### 🟦 Terminal 2 — Master Launcher (Bridge + TF + RViz2 + Autonomous nodes)

```bash
source /opt/ros/humble/setup.bash
export GZ_PARTITION=vdt_harmonic

# 1. Chạy với Improved APF (I-APF) — MẶC ĐỊNH (Tối ưu hóa tránh bẫy vật cản & GNRON):
python3 /home/duy/VDT_project/simulation/launch/launch_simulation.py --autonomous --planner iapf

# 2. Hoặc chạy với Standard APF cơ bản (để thực hiện kiểm chứng & benchmark đối đầu):
# python3 /home/duy/VDT_project/simulation/launch/launch_simulation.py --autonomous --planner apf
```

> ✅ Thành công khi:
> - Cửa sổ RViz2 mở ra, hiển thị map 3D PointCloud
> - Log xuất hiện: `[AUTONOMOUS] Started ekf_ros_adapter, apf_planner, ibvs_controller, mission_fsm, offboard_commander`
> - Log thuật toán hiển thị: `Initialized Improved APF (I-APF) 3D Planner` (hoặc `Standard APF Planner`)
> - Camera gimbal khởi tạo ở góc mặc định hiện tại `-30°`.
> - Định kỳ in: `[STATUS] Phase: SEARCH | Drone: (...) | Target: (...)`

Landing target uses an A0 landscape board. The printable file is
[`vision/dual_scale_aruco_board_A0_52_5cm.pdf`](vision/dual_scale_aruco_board_A0_52_5cm.pdf);
print at 100% scale (1189 × 841 mm). ID 42 is the 520 mm coarse tag at the
left; ID 43 is the 50 mm fine tag at the landing origin. Gazebo's collision
plane, RViz mesh, texture, detector config, and landing-pad radius use these
same dimensions.

---

#### 🟦 Terminal 3 — Khởi động QGroundControl (GCS quan sát telemetry)

```bash
~/QGroundControl.AppImage
```

> 💡 **Vai trò**: Trạm mặt đất kết nối tự động với PX4 SITL qua MAVLink UDP (`14550`), dùng để:
> - Giám sát trực quan trạng thái bay, toạ độ, độ cao thực tế, la bàn và dữ liệu telemetry theo thời gian thực.
> - Quan sát đường bay 3D và chuyển đổi trạng thái flight mode (`Hold`, `Offboard`,...).
>
> ⚠️ **Lưu ý quan trọng**: **Không** sử dụng thanh trượt Takeoff trên giao diện QGroundControl để cất cánh (do QGC bị hardcode chặn cứng mức tối thiểu 10.0 m trong mã nguồn QML, trong khi bài toán yêu cầu trần bay 3.0 m). Hãy thực hiện cất cánh ở **Terminal 4** ngay sau đây.

---

#### 🟦 Terminal 4 — Cất cánh drone lên 3.0 m

**Cách A (Script Python — Tiện nhất):**
```bash
source /opt/ros/humble/setup.bash
python3 /home/duy/VDT_project/simulation/core/takeoff.py --alt 3.0
```

**Cách B (PX4 Shell — Gõ trực tiếp vào Terminal 1):**
```text
pxh> param set MIS_TAKEOFF_ALT 3.0
pxh> commander takeoff
```

> ✅ Thành công khi: Drone bay lên 3.0 m, `offboard_commander` báo kết nối MAVLink `udp:127.0.0.1:14540`, FSM chuyển sang `FOLLOW` khi nhìn thấy H-Pad.

---

#### 🟦 Terminal 5 — Giám sát Vision & Tracking (Hoặc chạy Vision Node riêng)

> 💡 **Lưu ý**: Khi Terminal 2 chạy với cờ `--autonomous`, toàn bộ 6 node tự hành (gồm cả **ArUco Detector**) đã được kích hoạt tự động ngầm. Bạn có thể mở Terminal 5 để giám sát trạng thái nhận diện và EKF:

```bash
source /opt/ros/humble/setup.bash
# Quan sát kết quả bám mục tiêu của EKF:
ros2 topic echo /ekf/tracking_mode
# Hoặc quan sát cờ nhận diện marker:
ros2 topic echo /hpad/detected
```

*(Nếu ở Terminal 2 bạn **không** dùng cờ `--autonomous` mà chạy manual, hãy chạy script vision trực tiếp tại đây):*
```bash
source /opt/ros/humble/setup.bash
cd /home/duy/VDT_project
python3 simulation/perception/aruco_sim_node.py
```

> ✅ Thành công khi:
> - `Camera intrinsics loaded: fx=466.0 fy=466.0`
> - `[DETECTION] ID 42 detected | camera_optical xyz=(x, y, z) m`
> - Topic `/hpad/annotated` hiển thị bounding box xanh lá trên RViz2
> - `/ekf/tracking_mode` chuyển sang `TRACKING`

---

#### 🟦 Terminal 6 — Điều khiển H-Pad di động

```bash
source /opt/ros/humble/setup.bash
cd /home/duy/VDT_project
python3 simulation/utils/hpad_keyboard_controller.py --speed 0.5
```

**Bảng phím điều khiển H-Pad:**

| Phím | Chức năng |
|---|---|
| `W` / `S` | Di chuyển tiến / lùi (trục X) |
| `A` / `D` | Di chuyển trái / phải (trục Y) |
| `SPACE` | Phanh dừng khẩn cấp |
| `R` | Reset H-Pad về `(3.0, 1.0)` |
| `+` / `-` | Tăng / giảm tốc độ |

---

#### 🟦 Terminal 7 (Tuỳ chọn) — Ra lệnh hạ cánh

Khi muốn test pha `APPROACH → LAND`:
```bash
source /opt/ros/humble/setup.bash
ros2 topic pub --once /operator/land_command std_msgs/msg/Bool "{data: true}"
```

> ✅ Hành vi: FSM vào `APPROACH` và IAPF tiến ngang trong khi giữ độ cao. Gimbal tiếp tục IBVS với slew chậm; mất ảnh thì giữ góc hiện tại. Chỉ khi marker đang được nhìn thấy, covariance đạt ngưỡng và SMC có lệnh mới, drone mới vào `GLIDE_SLOPE`. `LAND/FINAL_DESCENT` chỉ bắt đầu khi `Rxy ≤ 1.0 m`, độ cao tương đối `≤ 1.5 m` và topic uncertainty `≤ 0.12 m` (bán kính 2σ, tương ứng σ ≤ 0.06 m). Mất marker hoặc gate sẽ làm drone hover; touchdown detector mới kết thúc hạ cánh.

---

### 4.2 Cách 2: Chạy Từng Node Riêng Lẻ (Debug Mode)

Dùng khi cần phát triển hoặc debug từng module. **Bỏ flag `--autonomous`** ở Terminal 2:

```bash
# Terminal 2: Chỉ mở Bridge + TF + RViz2
python3 simulation/launch/launch_simulation.py

# Terminal 2A: EKF Adapter
python3 simulation/perception/ekf_ros_adapter.py

# Terminal 2B: APF Planner
python3 simulation/control/apf_planner.py

# Terminal 2C: IBVS Controller
python3 simulation/control/ibvs_controller.py

# Terminal 2D: Mission FSM
python3 simulation/core/mission_fsm_node.py

# Terminal 2E: Offboard Commander
python3 simulation/core/offboard_commander.py
```

---

### 4.3 Tạo World SDF (Chỉ cần chạy 1 lần nếu file chưa tồn tại)

```bash
source /opt/ros/humble/setup.bash
python3 simulation/launch/launch_simulation.py --generate-world
```

---

## 5. Hướng dẫn triển khai Hardware

> [!IMPORTANT]
> Phần này dành cho việc chuyển từ simulation sang phần cứng thực tế (Jetson Nano / Raspberry Pi 5 + RealSense D435i + Flight Controller PX4).

### 5.1 Kiến trúc Phần cứng Vật lý & Phần mềm trên Companion Computer

#### 📌 Phân biệt rõ giữa Phần cứng (Hardware) và Phần mềm (Software):

1. **Thiết bị phần cứng vật lý (Physical Hardware)**:
   - **Companion Computer**: Máy tính nhúng (Nvidia Jetson Nano / Orin Nano hoặc Raspberry Pi 5) chịu trách nhiệm chạy toàn bộ thuật toán.
   - **Camera RGB-D**: Intel RealSense D435 / D435i kết nối qua cổng **USB 3.0** tới Companion Computer.
   - **Flight Controller (PX4 FC)**: Bo điều khiển bay (Pixhawk 4 / 6C / Holybro) nhận lệnh vận tốc/vị trí qua cổng **UART (TELEM2)** hoặc **USB**.
   - **Gimbal / Servo**: Điều khiển góc chúc camera qua kênh PWM từ FC hoặc Companion Computer.

2. **Phần mềm thực thi trên Companion Computer**:
   - `hardware/main_aruco_detector.py`: **Script kiểm tra độc lập (Bench-test)** trên bàn lab để xác nhận camera RealSense và thuật toán ArUco PnP hoạt động tốt trước khi bay.
   - `vision/`: Bộ thư viện lõi xử lý ảnh RealSense, nhận diện ArUco, lọc depth mask và thuật toán toán học **Target-State EKF** (`target_state_ekf.py`).
   - `simulation/perception/`: Node **EKF ROS Adapter** (`ekf_ros_adapter.py`) chạy ở 30 Hz để ước lượng vị trí/vận tốc 3D của H-Pad trong hệ quy chiếu `world`, bù trừ trễ và giữ tracking khi mất dấu.
   - `simulation/control/`: Node **APF Planner** (`apf_planner.py` - tính lực hút tới vị trí dự đoán của EKF & đẩy vật cản) và **IBVS Controller** (`ibvs_controller.py` - điều khiển gimbal).
   - `simulation/core/`: Node **Mission FSM** (`mission_fsm_node.py` - máy trạng thái bay) và **Offboard Commander** (`offboard_commander.py` - giao tiếp MAVLink qua UART tới PX4).

#### 📐 Sơ đồ kết nối tổng thể:

```
════════════════════════════════════════════════════════════════════════════════
                        PHẦN CỨNG VẬT LÝ (PHYSICAL HARDWARE)
════════════════════════════════════════════════════════════════════════════════
  ┌─────────────────────────┐                     ┌─────────────────────────┐
  │   Intel RealSense D435  │                     │   Flight Controller     │
  │   (Camera RGB-D)        │                     │   (Pixhawk / PX4 FC)    │
  └────────────┬────────────┘                     └────────────▲────────────┘
               │ Cáp USB 3.0                                   │ Cáp UART / USB
               ▼                                               │ (MAVLink 115200/921600)
┌──────────────────────────────────────────────────────────────┴────────────────┐
│ COMPANION COMPUTER (Nvidia Jetson Nano / Raspberry Pi 5)                      │
│                                                                               │
│  [1. Lab Bench-test]                                                          │
│  └── hardware/main_aruco_detector.py ◄─── Dùng test RealSense ngoài đời thật  │
│                                           (OpenCV GUI, PnP pose, depth mask)  │
│                                                                               │
│  [2. Flight ROS 2 Pipeline]                                                   │
│  ┌───────────────────────┐         ┌─────────────────────────┐                │
│  │ Driver & Perception   │         │ Guidance & Planning     │                │
│  │ - realsense2_camera   │         │ - apf_planner           │                │
│  │ - aruco_detector node ┼────────►│ - ibvs_controller       │                │
│  │ - ekf_ros_adapter     │         │   (Tính vel_cmd & yaw)  │                │
│  └───────────────────────┘         └────────────┬────────────┘                │
│                                                 │                             │
│                                    ┌────────────▼────────────┐                │
│                                    │ State Machine & Comm    │                │
│                                    │ - mission_fsm_node      │                │
│                                    │ - offboard_commander ───┼────────────────┘
│                                    │   (pyulog / pymavlink)  │
│                                    └─────────────────────────┘
```

### 5.2 Thứ tự triển khai lên hardware

#### Bước 1: Cài đặt môi trường trên Companion Computer

```bash
# Cài ROS 2 Humble (nếu chưa có)
sudo apt install -y ros-humble-desktop

# Cài RealSense SDK 2.0 & ROS wrapper
sudo apt-get install -y librealsense2-dkms librealsense2-utils ros-humble-realsense2-camera

# Clone repo & cài dependencies
cd ~/VDT_project
pip install -r requirements.txt
```

#### Bước 2: Bench-test kiểm tra camera RealSense trên bàn Lab

Trước khi gắn lên khung drone và cấp nguồn bay, chạy script kiểm tra độc lập:

```bash
# Chạy script bench-test với RealSense cắm cổng USB 3.0
python3 hardware/main_aruco_detector.py --dict DICT_6X6_50 --target-id 42
```
> ✅ **Tiêu chí đạt**: Cửa sổ OpenCV mở ra, nhận diện đúng marker 42 với trục 3D toạ độ, FPS $\ge 25$, đo khoảng cách Z-depth khớp với thước đo thực tế.

#### Bước 3: Cấu hình cổng kết nối PX4 cho Offboard (UART)

Trong file `simulation/core/offboard_commander.py`, thay thế connection string UDP (của simulation) sang cổng serial phần cứng:

```python
# Simulation (UDP SITL):
# connection_string = "udp:127.0.0.1:14540"

# Hardware (UART TELEM2 trên Jetson Nano):
connection_string = "/dev/ttyTHS1"  # baudrate 921600 hoặc 57600
# Hoặc cáp USB to FTDI:
# connection_string = "/dev/ttyUSB0"
```

#### Bước 4: Chuyển đổi tầng Vision sang ROS 2 thực tế

- **Phương án A (Khuyên dùng)**: Chạy node ROS 2 chính thức `realsense2_camera_node` để lấy stream `/camera/color/image_raw`, sau đó chạy node ArUco detector đóng gói từ `vision/aruco_detector.py` để publish topic `/hpad/position_camera`.
- **Phương án B**: Chạy adapter script đọc trực tiếp `RealSenseCamera` từ `vision/` và publish dữ liệu vào `/hpad/position_camera` giống giao diện `simulation/perception/aruco_sim_node.py`.

#### Bước 5: Giữ nguyên toàn bộ các node thuật toán lõi

Các node sau **hoàn toàn giữ nguyên logic** khi chạy trên hardware:

| Node | File | Ghi chú |
|---|---|---|
| EKF Adapter | `simulation/perception/ekf_ros_adapter.py` | Subscribe topic `/hpad/position_camera` |
| APF Planner | `simulation/control/apf_planner.py` | Input/output tính toán lực đẩy/hút không đổi |
| IBVS Controller | `simulation/control/ibvs_controller.py` | Điều khiển gimbal bám ảnh |
| Mission FSM | `simulation/core/mission_fsm_node.py` | Quản lý chuyển pha bay |
| Offboard Commander | `simulation/core/offboard_commander.py` | Gửi setpoint vận tốc MAVLink xuống PX4 qua UART |

#### Bước 6: Khởi động pipeline bay tự động trên hardware

```bash
source /opt/ros/humble/setup.bash

# 1. Khởi động Camera & Vision
ros2 launch realsense2_camera rs_launch.py &
python3 simulation/perception/aruco_sim_node.py & # hoặc node hardware vision

# 2. Khởi động các node xử lý lõi
python3 simulation/perception/ekf_ros_adapter.py &
python3 simulation/control/apf_planner.py &
python3 simulation/control/ibvs_controller.py &
python3 simulation/core/mission_fsm_node.py &

# 3. Khởi động Offboard Commander kết nối PX4 FC
python3 simulation/core/offboard_commander.py
```

> [!TIP]
> Trên companion computer ngoài thực địa, nên cấu hình `systemd service` hoặc file `launch.sh` gắn vào `tmux` để tự động chạy khi bật nguồn pin cho máy tính nhúng.

---

## 6. Cấu hình tham số

Tất cả tham số điều khiển được tập trung tại [`simulation/config/mission_params.yaml`](simulation/config/mission_params.yaml).

### Các tham số quan trọng nhất

| Tham số | Node | Mặc định | Mô tả |
|---|---|---|---|
| `planner_type` | `apf_planner` | `"iapf"` | Thuật toán né vật cản: `"iapf"` (cải tiến 3D) hoặc `"apf"` (cơ bản) |
| `follow_distance` | `apf_planner`, `mission_fsm` | `3.5` m | Cự ly bám mục tiêu |
| `target_altitude` | `apf_planner`, `offboard_commander` | `3.0` m | Độ cao bay FOLLOW |
| `v_max` | `apf_planner` | `1.5` m/s | Vận tốc tối đa APF / I-APF |
| `k_att` | `apf_planner` | `10.0` | Hệ số lực hút |
| `k_rep` | `apf_planner` | `2500.0` | Hệ số lực đẩy vật cản (FOLLOW phase) |
| `d0` | `apf_planner` | `2.5` m | Bán kính ảnh hưởng vật cản |
| `iapf_f_enter` | `apf_planner` | `0.10` N | Ngưỡng lực kích hoạt chế độ thoát bẫy Local Minima |
| `iapf_f_exit` | `apf_planner` | `0.30` N | Ngưỡng lực thoát chế độ tiếp tuyến (Hysteresis) |
| `iapf_k_tan` | `apf_planner` | `1.0` | Độ lớn lực tiếp tuyến khi kích hoạt thoát Local Minima |
| `iapf_n_tangent`| `apf_planner` | `12` | Số hướng tiếp tuyến candidate trên mặt phẳng 3D |
| `iapf_n_pred` | `apf_planner` | `3` | Số bước dự báo trước để đánh giá độ thông thoáng |
| `goal_threshold` | `apf_planner` | `0.30` m | Vùng dừng APF quanh standoff goal |
| `yaw_align_min_speed_scale` | `mission_fsm` | `0.60` | Tỉ lệ tối thiểu cho APF guidance khi căn yaw; feedforward target được giữ riêng |
| `reacquire_bbox_timeout` | `mission_fsm` | `0.80` s | Tuổi tối đa bbox mới để xác nhận reacquire; bbox phải đến sau trigger |
| `follow_bbox_timeout` | `mission_fsm` | `0.30` s | Tuổi tối đa của bbox dùng cho lệnh FOLLOW |
| `yaw_rate_limit` | `ibvs_controller` | `0.80` rad/s | Giới hạn slew của IBVS yaw command |
| `yaw_feedforward_min_target_speed` | `ibvs_controller` | `0.30` m/s | Bỏ yaw lead khi chuyển động ước lượng gần nhiễu tĩnh |
| `max_yaw_rate` | `offboard_commander` | `0.80` rad/s | Giới hạn yaw rate cuối cùng gửi PX4 |
| `search_timeout` | `mission_fsm` | `3.0` s | Timeout trước khi quay lại SEARCH |
| `gate_threshold` | `ekf_ros_adapter` | `16.27` | Ngưỡng Mahalanobis gate EKF |
| `process_accel_variance` | `ekf_ros_adapter` | `[0.25, 0.25, 0.5]` | Nhiễu động học cho bộ lọc mục tiêu; trục ngang phù hợp với rate ảnh mô phỏng |
| `K_pitch` | `ibvs_controller` | `0.92` | Gain điều khiển gimbal pitch |
| `K_yaw` | `ibvs_controller` | `0.92` | Gain điều khiển yaw |
| `max_uncertainty_enter_glide` | `covariance_gate` | `0.45` m | Ngưỡng 2σ tối đa để vào GLIDE_SLOPE |
| `max_uncertainty_continue_glide` | `covariance_gate` | `0.50` m | Ngưỡng 2σ tối đa để duy trì GLIDE_SLOPE |
| `final_uncertainty_max` | `covariance_gate` | `0.35` m | Ngưỡng 2σ tối đa để vào FINAL_DESCENT |
| `r_switch_min` / `r_switch_max` | `covariance_gate` | `4.5 / 10.0` m | Dải điều biến thích ứng của bán kính dốc trượt |
| `conical_gate_angle_deg` | `covariance_gate` | `30.0` deg | Góc phễu hình nón an toàn tiếp cận |
| `theta_des_deg` | `landing_guidance` | `45.0` deg | Góc dốc tiếp cận mong muốn (SMC) |
| `v_descend_glide` | `landing_guidance` | `0.35` m/s | Tốc độ hạ dốc trượt tối đa |
| `v_descend_touch` | `landing_guidance` | `0.15` m/s | Tốc độ hạ êm ái khi tiếp đất |
| `optical_height_threshold_m` | `touchdown_detector` | `0.40` m | Độ cao quang học tiếp đất |
| `confirmation_duration_s` | `touchdown_detector` | `0.35` s | Thời gian xác nhận chốt tiếp đất |

### 🚀 Điểm cải tiến của Improved APF (I-APF) so với APF cơ bản

1. **Giải quyết triệt để GNRON (*Goal Non-Reachable with Obstacle Nearby*)**: Lực đẩy được điều biến theo trọng số Sigmoid $weight\_rep = \frac{1}{1 + e^{-d_{goal}}} - 0.5$. Khi drone áp sát đích ($d_{goal} \to 0$), lực cản đẩy lùi triệt tiêu và thành phần $F_{rep2}$ hỗ trợ đẩy drone về đúng tâm đích, loại bỏ hiện tượng bị dội ngược ra xa.
2. **Thoát bẫy cực tiểu địa phương (Local Minima)**: Tích hợp bộ phát hiện bẫy thế năng bằng cơ chế **Hysteresis Schmitt Trigger** ($F_{enter}=0.10\,N$, $F_{exit}=0.30\,N$). Khi drone bị kẹt giữa các vật cản đối xứng, thuật toán tự động sinh 12 tia tiếp tuyến 3D, dự báo khoảng trống an toàn $N_{pred}=3$ bước phía trước để chọn hướng lách tối ưu (hiển thị mũi tên màu **Magenta** trên RViz2).
3. **Khử rung lắc (Oscillation Suppression)**: Lọc hướng bay theo 3 phân đoạn góc $\Delta\alpha$ ($\le 30^\circ$, $\le 60^\circ$, $> 60^\circ$), giữ độ êm ái khi đổi hướng đột ngột, triệt tiêu hiện tượng giật cục cho Quadrotor.
4. **Bảo đảm tốc độ sàn thoát hiểm**: Duy trì vận tốc tối thiểu $v_{escape\_min} = 0.35 \cdot v_{max}$ khi đang thoát kẹt, ngăn drone bị "chết đứng" trong vùng ảnh hưởng của vật cản.

> Xem mô tả đầy đủ tất cả tham số tại [`MODULES_REFERENCE.md`](MODULES_REFERENCE.md).

---

## 7. Xác nhận & Tiêu chí nghiệm thu

Sau khi pipeline chạy ổn định, kiểm tra lần lượt từng tiêu chí:

| STT | Hạng mục | Lệnh kiểm tra | Tiêu chí PASS | Kết quả kiểm chứng thực tế |
|---|---|---|---|---|
| 1 | Tần số sensor | `ros2 topic hz /camera` | ≥ 15 Hz | **PASS** (30 Hz) |
| 2 | Tần số sensor | `ros2 topic hz /odom` | ≥ 20 Hz | **PASS** (30 Hz) |
| 3 | ArUco detection | `ros2 topic echo --once /hpad/position_camera` | Trả về tọa độ `(x,y,z)` | **PASS** (Nested Board ID 42 + 43) |
| 4 | EKF tracking | `ros2 topic echo /ekf/tracking_mode` | Chuyển thành `TRACKING` | **PASS** |
| 5 | EKF dead reckoning | Che khuất H-Pad < 1s | Mode `DEAD_RECKONING`, không mất setpoint | **PASS** |
| 6 | APF/FOLLOW velocity | `ros2 topic echo /mission/velocity_setpoint` | `|v_xy| ≤ 1.5 m/s`; target feedforward còn hoạt động | **PASS** |
| 7 | IBVS yaw | `ros2 topic echo /ibvs/yaw_cmd` | yaw setpoint slew tối đa `0.8 rad/s`, bám mượt | **PASS** |
| 8 | IBVS gimbal | `ros2 topic echo /ibvs/gimbal_pitch` | Pitch mượt `[-85°, -15°]`, nadir tilt cự ly gần | **PASS** |
| 9 | Reacquire handoff | `ros2 topic echo /mission/phase` | Vào FOLLOW sau bbox mới, EKF `TRACKING` | **PASS** |
| 10 | Obstacle avoidance | Quan sát Gazebo | `d_min ≥ 0.8 m` với mọi trụ né vật cản | **PASS** |
| 11 | Landing accuracy | Quan sát tiếp xúc sàn | Sai số tâm H-Pad `< 10 cm`, tiếp đất êm, auto disarm | **PASS** ($R_{xy} \approx 0.01\text{m}$ / $1\text{cm}$, chốt 0.35s, Force Disarm) |

---

## 8. Troubleshooting

### ❌ Gazebo bridge không nhận dữ liệu / `No subscribers`
- **Nguyên nhân**: Sai phiên bản bridge hoặc thiếu biến môi trường `GZ_PARTITION`
- **Khắc phục**:
  ```bash
  sudo apt install -y ros-humble-ros-gzharmonic-bridge
  export GZ_PARTITION=vdt_harmonic  # Phải set trước KHI chạy cả Terminal 1 và Terminal 2
  ```

### ❌ EKF báo `TF lookup failed (world → camera_optical_frame)`
- **Nguyên nhân**: `launch_simulation.py` chưa chạy (node này broadcast TF ở 30 Hz)
- **Khắc phục**: Chạy Terminal 2 trước Terminal 5 (ArUco node), đảm bảo log không có lỗi import.

### ❌ `[SIM-ONLY] px4_msgs not available`
- **Giải thích**: Đây là **fallback an toàn** đã được tích hợp. Hệ thống vẫn tính toán đầy đủ APF/EKF/IBVS/FSM và xuất log chẩn đoán, không crash.
- **Khi cần kết nối thực**: Build workspace chứa `px4_msgs` tương thích PX4 v1.14.

### ❌ ArUco node không nhận diện marker
- **Khắc phục**:
  1. Reset H-Pad: bấm `R` trong `hpad_keyboard_controller.py`
  2. Kiểm tra gimbal chúc xuống:
     ```bash
     ros2 topic pub --once /model/x500_depth_0/command/gimbal_pitch std_msgs/msg/Float64 "{data: -0.5}"
     ```
  3. Quan sát `/hpad/annotated` trên RViz2 để xem trực tiếp khung nhận diện

### ❌ Drone quay liên tục hoặc mất tracking trong FOLLOW
- **Nguyên nhân phổ biến**: EKF EXPIRED do sai số TF hoặc delay camera
- **Debug đồng thời**:
  ```bash
  ros2 topic echo /apf/velocity_cmd
  ros2 topic echo /apf/guidance_velocity_cmd
  ros2 topic echo /apf/target_velocity_ff
  ros2 topic echo /ibvs/yaw_cmd
  ros2 topic echo /ekf/tracking_mode
  ```
  - Nếu `/ekf/tracking_mode` = `EXPIRED` → kiểm tra lại camera FPS và TF tree
  - `/apf/velocity_cmd` là tổng APF + feedforward để quan sát; FSM lấy hai thành phần riêng để yaw alignment không giảm bù vận tốc target.
  - Trong FOLLOW, xem `/mission/velocity_setpoint`: `linear.z` phải bằng `0`; `offboard_commander` giữ độ cao. Nếu target di chuyển mà `/apf/target_velocity_ff` khác 0 nhưng setpoint XY không theo, kiểm tra vision freshness và trạng thái FOLLOW.

### ❌ APF thay đổi độ cao trong FOLLOW
- FOLLOW và APF_APPROACH giữ `linear.z = 0` tại FSM; Offboard altitude hold sở hữu trục z. Chỉ SMC GLIDE_SLOPE và FINAL_DESCENT được điều khiển hạ độ cao.
- APF dùng vùng dừng 0.30 m quanh standoff goal để tránh đổi chiều liên tục do nhiễu EKF khi marker đứng yên. Target velocity feedforward chỉ bị triệt khi tốc độ ước lượng thấp hơn 0.20 m/s.

### ❌ Drone hover ở GLIDE_SLOPE hoặc FINAL_DESCENT
- Kiểm tra `/landing/covariance_status`, `/landing/uncertainty_radius`, `/landing/sub_phase`, `/hpad/detected` và `/ekf/tracking_mode`.
- `uncertainty_radius` là bán kính confidence 2σ: cần `≤ 0.30 m` để tiếp tục glide và `≤ 0.12 m` để vào final descent. Nếu gate đóng, zero velocity là hành vi an toàn; kiểm tra covariance odometry, TF và chất lượng marker trước khi thử lại.

---

*Chi tiết API, ROS 2 Topics, và hướng dẫn test từng module xem tại [`MODULES_REFERENCE.md`](MODULES_REFERENCE.md)*
