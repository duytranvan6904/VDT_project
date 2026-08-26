# PROJECT SCOPE — Quadrotor H-Pad Tracking, Obstacle Avoidance & Precision Landing

> **Phiên bản**: 3.0 — 26/08/2026  
> **Nhóm**: Duy (Vision/Estimation) · Tuân (Control/Planning) · Việt Anh (Embedded/Integration)  
> **Thời gian**: 9 tuần (24/08 → 25/10/2026)

---

## 1. Phạm Vi Bài Toán (Scope)

### 1.1. Tên đề tài

**"Bám Bãi Đáp Di Động, Tránh Vật Cản và Hạ Cánh Chính Xác Theo Lệnh Sử Dụng Một Camera Độ Sâu Duy Nhất với Gimbal Chủ Động trên Quadrotor"**

*(Autonomous Dynamic H-Pad Following, Obstacle Avoidance, and Operator-Commanded Precision Landing Using a Single Active-Gimbal Depth Camera on a Quadrotor)*

### 1.2. Phạm vi triển khai

| Hạng mục | Phạm vi |
|----------|---------|
| **Nền tảng bay** | Quadrotor duy nhất (Holybro S500 V2), sử dụng PX4 Autopilot (Pixhawk 6C), companion computer Raspberry Pi 5 (4GB RAM) |
| **Cảm biến** | 1 camera RGB-D duy nhất gắn trên gimbal 1-axis Pitch servo (kết hợp Yaw Quadrotor) |
| **Mục tiêu bám** | H-Pad (bãi đáp) được dán ArUco marker, gắn trên platform di động (xe đẩy / xe robot) di chuyển |
| **Nhiệm vụ** | Pipeline 4 pha liên tục: **SEARCH → FOLLOW → APPROACH → LAND** |
| **Phần mềm** | ROS 2 Humble, PX4 SITL + Gazebo (mô phỏng), PX4 Offboard (thực nghiệm) |
| **Trigger hạ cánh** | Manual bởi Operator qua RC Switch (Human-in-the-Loop) |

---

## 2. Mô Tả Bài Toán

### 2.1. Tổng quan

Quadrotor thực hiện **nhiệm vụ liên tục 4 pha** — cất cánh, tìm kiếm và bám theo bãi đáp H-Pad di động, né vật cản trên đường bay, và hạ cánh chính xác lên H-Pad khi Operator ra lệnh — **chỉ sử dụng duy nhất một camera RGB-D** gắn trên gimbal chủ động.

Ý tưởng cốt lõi là **chia sẻ luồng dữ liệu từ một cảm biến duy nhất** cho đồng thời hai tác vụ: (1) nhận diện và theo dõi mục tiêu qua luồng RGB, (2) nhận diện vật cản qua luồng Depth — và **kết hợp hai luồng thông tin này** trong một bộ planner thống nhất để tạo ra quỹ đạo bay an toàn, liên tục.

### 2.2. Pha SEARCH — Tìm kiếm mục tiêu

**Ý tưởng**: Sau khi cất cánh và hover ổn định, drone quay chậm quanh trục Yaw để quét trường nhìn 360°. Gimbal giữ ngang (pitch 0°) để tối đa hóa tầm nhìn ngang. Khi ArUco marker trên H-Pad được phát hiện ổn định (≥ 10 frame liên tiếp), drone chuyển sang bám theo.

**Đầu vào**: RGB frame từ camera  
**Đầu ra**: Tín hiệu "mục tiêu tìm thấy" + tọa độ 3D ban đầu của H-Pad

### 2.3. Pha FOLLOW — Bám mục tiêu & né vật cản

**Ý tưởng**: Đây là pha chính và phức tạp nhất. Drone duy trì khoảng cách theo dõi ($d_{follow}$ ≈ 3-5m) với H-Pad di động, đồng thời liên tục né vật cản trên đường bay.

- **Luồng RGB** → ArUco detection → pose estimation → EKF smoother → vị trí + vận tốc mượt của H-Pad → **lực hấp dẫn** (attractive force) trong trường lực
- **Luồng Depth** → loại bỏ vùng H-Pad (masking) → phát hiện vật cản thực → **lực đẩy** (repulsive force) trong trường lực
- Tổng hợp lực → velocity setpoint → PX4 Offboard

Gimbal tự động tilt nhẹ xuống khi drone bay phía trên H-Pad để giữ marker trong FOV.

**Đầu vào**: RGB + Depth frame, Drone odometry  
**Đầu ra**: Velocity setpoint, Gimbal angle

### 2.4. Pha APPROACH — Tiếp cận bãi đáp

**Ý tưởng**: Khi Operator nhấn RC Switch "LAND NOW", drone bắt đầu giảm dần khoảng cách và độ cao tới H-Pad. Gimbal tilt xuống sâu hơn để duy trì marker trong FOV. Cơ chế tránh vật cản vẫn hoạt động nhưng với gain giảm (ưu tiên hội tụ về đích).

Drone căn chỉnh tâm marker với tâm camera frame, đảm bảo alignment trước khi chuyển sang hạ cánh. Nếu marker bị mất > 3s hoặc Operator tắt switch → quay lại FOLLOW.

**Đầu vào**: RGB + Depth frame, RC Switch, Drone state  
**Đầu ra**: Position setpoint, Gimbal angle

### 2.5. Pha LAND — Hạ cánh chính xác

**Ý tưởng**: Khi alignment đạt ngưỡng (error < 0.3m) và altitude < 0.5m, drone hạ thẳng đứng. Gimbal nhìn thẳng xuống (pitch −90°). Disarm motor khi phát hiện touchdown (gia tốc kế spike hoặc altitude ≈ 0).

**Đầu vào**: ArUco fine pose  
**Đầu ra**: Land command → PX4

---

## 3. Điều Kiện Biên, Phần Cứng Thực Nghiệm và KPI Nghiệm Thu

### 3.1. Điều kiện biên (Constraints)

| Ràng buộc | Giá trị | Ghi chú |
|-----------|---------|---------|
| Vận tốc platform di động | ≤ 1.0 m/s | Tương đương đi bộ / xe đẩy chậm |
| Khoảng cách giữa các vật cản | ≥ 1.5 m | Đảm bảo quadrotor có lối đi |
| Độ cao bay follow | 3 - 4 m | Đủ cao để FOV nhìn rộng, đủ thấp để detect marker |
| Khoảng cách follow (horizontal) | 2 - 3 m so với target | |
| Camera duy nhất | 1 × Depth Camera (RGB-D) | D435i |
| Tốc độ bay tối đa drone | ≤ 2.0 m/s | Giới hạn an toàn ngoài trời |
| Wind condition | ≤ Beaufort 3 (≤ 3.4 m/s) | Bay ngoài trời điều kiện gió nhẹ |
| Trigger hạ cánh | Manual (RC Switch Channel 7/8) | Human-in-the-Loop |
| Kill Switch | Luôn sẵn sàng (RC Channel 5/6) | Safety-critical |

### 3.2. Phần cứng thực nghiệm

| Thành phần | Thiết bị | Vai trò |
|------------|----------|---------|
| **Frame** | Holybro S500 V2 | Quadrotor frame tiêu chuẩn PX4 |
| **Flight Controller** | Pixhawk 6C (PX4 v1.14+) | Autopilot, IMU, barometer |
| **Companion Computer** | Raspberry Pi 5 (4GB RAM) | Chạy ROS 2, vision pipeline,planner |
| **Camera** | Intel RealSense D435i | RGB + Depth stream |
| **Gimbal** | 1-axis Pitch servo (1× MG90S + giá đỡ) | Tilt camera (gập lên/xuống); hướng ngang Pan điều khiển qua góc Yaw của Quadrotor |
| **RC Transmitter** | Radiomaster TX16S (hoặc tương đương) | Manual override, Kill Switch, Land trigger |
| **Landing Platform** | H-Pad (50×50cm) với ArUco marker trên xe đẩy / xe RC | Mục tiêu di động |
| **Battery** | LiPo 4S 5200mAh | Thời gian bay ≥ 10 phút |
| **Giao tiếp PX4 ↔ Pi** | UART / micro-XRCE-DDS | ROS 2 – PX4 bridge |


### 3.3. KPI Nghiệm Thu

| # | Metric | Target | Phương pháp đo |
|---|--------|--------|-----------------|
| 1 | **Tracking error** (pha FOLLOW) | < 1.0 m (RMSE) | Log pose error từ EKF vs. ground truth (GPS hoặc Optitrack nếu có) |
| 2 | **Min obstacle clearance** | > 0.8 m | Đo khoảng cách gần nhất từ drone tới vật cản trong mỗi flight |
| 3 | **Landing accuracy** (tâm marker) | < 30 cm | Đo khoảng cách giữa tâm drone và tâm H-Pad sau touchdown |
| 4 | **End-to-end latency** (sensor → cmd) | < 150 ms | Timestamp đo trên ROS 2 topic chain |
| 5 | **Follow duration** (continuous) | ≥ 60 s | Thời gian duy trì FOLLOW liên tục không mất target |
| 6 | **Mission success rate** | ≥ 70% | Trên ≥ 5 lần bay: SEARCH → FOLLOW → APPROACH → LAND thành công |
| 7 | **APF computation time** | < 20 ms / cycle | Benchmark VO-APF node trên Pi 5 |

---

## 4. Framework Bài Toán / State Machine Chi Tiết

### 4.1. Kiến trúc phần mềm (ROS 2 Nodes)

```
┌─────────────────────────────────────────────────────────────────────┐
│                     ROS 2 HUMBLE NODE GRAPH                        │
│                                                                     │
│  ┌────────────────┐     /camera/color/image_raw                    │
│  │  realsense2_   │────────────────────────┐                       │
│  │  camera_node   │     /camera/depth/      │                       │
│  │  (or depthai)  │───── image_rect_raw ──┐ │                       │
│  └────────────────┘                       │ │                       │
│                                           │ │                       │
│  ┌────────────────────────────────────────▼─▼──────────┐           │
│  │  /vision_node (Duy)                                  │           │
│  │  ├── ArUco detect → /hpad/pose [PoseStamped]        │           │
│  │  ├── ArUco BBox → /hpad/bbox [BoundingBox2D]        │           │
│  │  └── Depth masking → /obstacles/depth [Image]        │           │
│  └──────────┬────────────────────┬──────────────────────┘           │
│             │                    │                                   │
│  ┌──────────▼──────────┐  ┌─────▼──────────────────────┐           │
│  │  /ekf_node (Duy)    │  │  /apf_planner_node (Tuân)  │           │
│  │  State: [x,y,z,     │  │  ├── Subscribe:            │           │
│  │   vx,vy,vz]         │  │  │   /hpad/pose (attract)  │           │
│  │  Pub: /hpad/         │  │  │   /obstacles/depth      │           │
│  │   state_filtered     │  │  │   /ekf/state            │           │
│  └──────────┬──────────┘  │  │   /drone/odom            │           │
│             │              │  ├── Compute VO-APF         │           │
│             └──────────────│  ├── Follow guidance law    │           │
│                            │  ├── Landing visual servo   │           │
│                            │  └── Pub: /cmd/velocity     │           │
│                            └─────────┬─────────────────┘           │
│                                      │                              │
│  ┌──────────────────┐    ┌───────────▼──────────────────┐          │
│  │ /rc_input_node   │    │ /mission_manager_node (V.Anh)│          │
│  │ (PX4 RC channel) │───►│ ├── State Machine (4 states) │          │
│  │ /rc/channels     │    │ ├── Gimbal servo PWM control │          │
│  └──────────────────┘    │ ├── Offboard mode manager    │          │
│                          │ └── Pub: /cmd/offboard        │          │
│                          └───────────┬──────────────────┘          │
│                                      │                              │
│                          ┌───────────▼──────────────────┐          │
│                          │  px4_ros_com / micro_xrce_dds│          │
│                          │  → PX4 Autopilot (Pixhawk)   │          │
│                          └──────────────────────────────┘          │
└─────────────────────────────────────────────────────────────────────┘
```

### 4.2. Data Flow — Xử lý luồng dữ liệu song song từ 1 camera

```
                    ┌──────────────────┐
                    │   1 × RGB-D      │
                    │   Camera         │
                    └────┬────────┬────┘
                         │        │
                    RGB Stream   Depth Stream
                         │        │
                         ▼        │
              ┌──────────────┐    │
              │ ArUco Detect │    │
              │ (cv2.aruco)  │    │
              └───┬──────┬───┘    │
                  │      │        │
             Pose 3D   BBox      │
             (x,y,z,   (u,v,    │
              R,P,Y)   w,h)     │
                  │      │        │
                  │      │        ▼
                  │      │   ┌──────────────┐
                  │      └──►│ Depth Masking │  ← Dilate BBox 15%
                  │          │ Set H-Pad     │     rồi zero-out
                  │          │ region = 0    │     vùng H-Pad
                  │          └──────┬───────┘
                  │                 │
                  ▼                 ▼
           ┌──────────┐    Filtered Depth
           │   EKF    │    (chỉ vật cản thực)
           │ Smoother │           │
           └────┬─────┘           │
                │                 │
     [x,y,z,vx,vy,vz]           │
      (H-Pad filtered)           │
                │                 │
                ▼                 ▼
         ┌────────────────────────────┐
         │        VO-APF Engine       │  ← + drone odometry
         │                            │
         │  F_att = f(H-Pad state)    │  ← Lực hấp dẫn tới H-Pad
         │  F_rep = Σ f_VO(obs_i)     │  ← Lực đẩy từ vật cản
         │  F_total = F_att + F_rep   │
         └────────────┬───────────────┘
                      │
                      ▼
            ┌──────────────────┐
            │ Guidance Law /   │  ← Tuỳ state: Follow / Approach / Land
            │ Landing Servo    │
            └────────┬─────────┘
                     │
                     ▼
          Velocity/Position Setpoint
                     │
                     ▼
            PX4 Offboard Mode
```

**Điểm then chốt**: Hai luồng RGB và Depth chạy **song song nhưng phối hợp** — BBox từ ArUco detection (RGB) được dùng để mask vùng H-Pad ra khỏi Depth map trước khi đưa vào APF. Nhờ đó, H-Pad **không bị coi là vật cản** mặc dù xuất hiện trong depth frame.

### 4.3. State Machine — Bảng trạng thái

| State | Entry Condition | Exit Condition | Hành vi chính | Gimbal |
|-------|----------------|----------------|---------------|--------|
| **SEARCH** | Takeoff hoàn tất | ArUco detected liên tục ≥ 10 frames | Hover tại chỗ, quay Yaw 360° chậm tìm marker | Pitch 0° |
| **FOLLOW** | ArUco detected ổn định | (1) Operator nhấn "LAND" → APPROACH (2) Marker lost > 5s → SEARCH | Bám H-Pad ở khoảng cách $d_{follow}$, VO-APF né vật cản liên tục | Pitch: $-\arctan(\Delta h / d)$ |
| **APPROACH** | Operator nhấn RC Switch + Marker visible | (1) Aligned + alt < 0.5m → LAND (2) Marker lost > 3s → FOLLOW (3) Operator cancel → FOLLOW | Giảm khoảng cách + altitude dần, align tâm marker, APF giảm gain | Pitch tilt xuống dần |
| **LAND** | Aligned (err < 0.3m) + alt < 0.5m | Touchdown detected (acc spike) hoặc alt ≈ 0 | Hạ thẳng đứng, disarm khi chạm | Pitch −90° |

### 4.4. Sơ đồ chuyển trạng thái

```
                     ┌──────────────────────────────────┐
                     │           TAKEOFF                 │
                     └──────────────┬───────────────────┘
                                    │ Takeoff complete
                     ┌──────────────▼───────────────────┐
                ┌───►│           SEARCH                  │
                │    │  • Hover + quay Yaw tìm marker    │
                │    └──────────────┬───────────────────┘
                │                   │ ArUco detected ≥ 10 frames
                │    ┌──────────────▼───────────────────┐
                │    │           FOLLOW                  │
                ├────│  • Bám H-Pad ở d_follow           │◄─────────┐
                │    │  • VO-APF né vật cản              │          │
           Lost │    │  • EKF smooth vị trí/vận tốc      │          │
           > 5s │    └──────────────┬───────────────────┘          │
                │                   │                               │
                │                   │ Operator nhấn                 │
                │                   │ RC Switch "LAND"              │ Lost > 3s
                │                   │ + Marker visible              │ OR Cancel
                │    ┌──────────────▼───────────────────┐          │
                │    │          APPROACH                  │──────────┘
                │    │  • Giảm distance + altitude       │
                │    │  • Gimbal tilt xuống               │
                │    │  • Align tâm marker                │
                │    │  • VO-APF gain giảm 50%            │
                │    └──────────────┬───────────────────┘
                │                   │ Aligned + alt < 0.5m
                │    ┌──────────────▼───────────────────┐
                │    │            LAND                   │
                │    │  • Hạ thẳng đứng                  │
                │    │  • Gimbal pitch −90°               │
                │    │  • Disarm khi touchdown             │
                │    └──────────────┬───────────────────┘
                │                   │ Touchdown confirmed
                │    ┌──────────────▼───────────────────┐
                │    │          COMPLETE                  │
                │    └──────────────────────────────────┘
```

### 4.5. Gimbal Servo Control — Điều khiển theo State

| State | Gimbal Pitch | Công thức | Mục đích |
|-------|-------------|-----------|----------|
| SEARCH | $0°$ | Cố định | Quét ngang tìm marker |
| FOLLOW | $0° \to -20°$ | $\theta_g = -\arctan({\Delta h}/{d_{horiz}})$ | Giữ H-Pad trong FOV khi bay trên |
| APPROACH | $-20° \to -60°$ | $\theta_g = -\arctan({d_{horiz}}/{h_{alt}})$ | Nhìn xuống dần khi tiếp cận |
| LAND | $-60° \to -90°$ | Tuyến tính theo altitude | Nhìn thẳng xuống |

---

## 5. Phương Án Triển Khai Thuật Toán

### 5.1. Tổng quan tài liệu tham khảo

Dự án tham khảo 8 công trình nghiên cứu liên quan, được phân tích trong bảng dưới đây:

| # | Bài báo | Phương pháp chính | Gap so với dự án |
|---|---------|-------------------|------------------|
| [1] | Łuczak & Granosik, 2025 — *Autonomous UAV Landing & Collision Avoidance w/ Depth Camera + Active Gimbal* | Gimbal chủ động + depth camera cho landing trên terrain lạ; PX4 + Jetson Nano | Chỉ landing, không có pha follow/tracking target di động |
| [2] | Keipour et al., 2022 — *Visual Servoing Approach to UAV Landing on Moving Vehicle* | Visual servoing trong image space, velocity cmd 3D trực tiếp | Không depth-based obstacle avoidance; chỉ landing |
| [3] | Han et al., 2021 — *Fast-Tracker: Robust Aerial System for Tracking Agile Target* | Target motion prediction + kinodynamic search + trajectory optimizer | Dùng riêng D435 mapping + camera mono tracking; không có landing |
| [4] | Ji et al., 2021 — *Elastic Tracker: Spatio-temporal Trajectory Planner for Aerial Tracking* | Occlusion-aware path finding, visibility cost, B-spline optimizer | Camera mapping & tracking tách biệt; không landing; cần compute nặng |
| [5] | Pan et al., 2021 — *Fast-Tracker 2.0: Active Vision & Human Location Regression* | Deep learning human detection, 360° gimbal, occlusion-aware planner | Gimbal cho active vision nhưng deep model nặng; chưa kết hợp landing |
| [6] | Qi et al., 2019 — *Autonomous Landing of Low-cost Quadrotor on Moving Platform* | Multi-size ArUco, 3D point-cluster loại false pose, adaptive backstepping | Monocular (không depth); không obstacle avoidance; không follow phase |
| [7] | Zhou et al., 2020 — *EGO-Planner: ESDF-free Gradient-based Local Planner* | B-spline gradient-based, không cần ESDF; lightweight realtime | Chỉ tránh vật cản tĩnh; không target tracking/landing |
| [8] | Ma'arif et al., 2021 — *APF Algorithm for Obstacle Avoidance in UAV for Dynamic Env* | So sánh APF truyền thống, modified APF, virtual-force APF | Chỉ mô phỏng MATLAB; không perception thực; local minima chưa giải quyết triệt để |

### 5.2. Điểm mới trong dự án so với các công trình trước

> **Nhận xét chung từ literature review**: Các nghiên cứu hiện có giải quyết tracking, obstacle avoidance, và landing **một cách riêng lẻ** hoặc chỉ kết hợp 2/3 tác vụ. Chưa có công trình nào kết hợp cả 3 trên một pipeline liên tục, sử dụng **duy nhất một camera RGB-D** với **gimbal chủ động**.

| # | Điểm mới | So sánh với nghiên cứu trước |
|---|----------|------------------------------|
| **N1** | **Unified Single-Camera Architecture cho 3 tác vụ** — 1 RGB-D camera duy nhất phục vụ đồng thời tracking (RGB → ArUco), obstacle avoidance (Depth → APF), và precision landing (RGB → visual servo) | Fast-Tracker [3] & Elastic Tracker [4] dùng ≥ 2 camera tách biệt. Łuczak [1] dùng 1 depth camera nhưng chỉ cho landing+avoidance, không tracking. |
| **N2** | **Cơ chế né vật cản APF & nâng cấp Velocity-Adaptive APF (VO-APF)** — Triển khai APF tiêu chuẩn làm baseline, sau đó mở rộng VO-APF điều biến lực đẩy theo vận tốc tương đối drone-obstacle | Ma'arif [8] khảo sát APF cải tiến nhưng chỉ mô phỏng MATLAB, không tích hợp perception thực. Dự án đưa APF tiêu chuẩn & VO-APF lên Quadrotor thực nghiệm với camera độ sâu. |
| **N3** | **H-Pad-Aware Depth Masking** — Pipeline loại bỏ vùng H-Pad (dùng BBox từ ArUco) khỏi Depth map trước khi tính APF, giải quyết xung đột "mục tiêu bám = vật cản" | Hoàn toàn mới — không có trong bất kỳ paper nào được khảo sát. Đây là hệ quả trực tiếp của kiến trúc single-camera. |
| **N4** | **Active Gimbal Multi-Phase Control theo State Machine** — Gimbal tự chuyển góc nhìn theo pha (ngang → chéo → thẳng xuống) | Łuczak [1] dùng gimbal nhưng chỉ cho landing. Fast-Tracker 2.0 [5] dùng gimbal cho tracking. Chưa ai kết hợp gimbal chuyển đổi liên tục qua nhiều pha. |

### 5.3. Phương án thuật toán cho từng module

#### 5.3.1. Module Vision — Nhận diện mục tiêu & xử lý Depth

**Phương án đề xuất: ArUco Detection + Depth Masking Pipeline**

| Bước | Thuật toán / Kỹ thuật | Tham khảo |
|------|----------------------|-----------|
| Phát hiện ArUco marker | `cv2.aruco.detectMarkers()` + `solvePnP()` | Qi [6]: dùng multi-size ArUco |
| Pose estimation (6DOF) | PnP từ 4 corner points → pose (x,y,z, R,P,Y) trong camera frame | Standard OpenCV |
| Depth Masking | Dilate ArUco BBox 15% → zero-out vùng đó trên depth map | **Mới (N3)** |
| Obstacle extraction | Từ filtered depth → voxel grid / region clustering → danh sách obstacle (pos, size) | Tham khảo EGO-Planner [7] voxelize |

**Xử lý luồng dữ liệu song song**: Camera output RGB + Depth ở ~30 FPS. Vision node xử lý:
1. **Thread 1 (RGB)**: ArUco detect → pose + BBox → publish `/hpad/pose`, `/hpad/bbox`
2. **Thread 2 (Depth)**: Nhận BBox → mask → extract obstacles → publish `/obstacles/depth`

Hai thread chia sẻ BBox qua shared memory (lock-free queue) để đảm bảo < 5ms sync delay.

**Phương án thay thế**: Nếu ArUco detection không đủ ổn định ở khoảng cách xa (> 5m), có thể bổ sung multi-scale ArUco (marker lớn bao quanh marker nhỏ) theo phương pháp của Qi [6].

---

#### 5.3.2. Module Estimation — EKF cho H-Pad State

**Phương án đề xuất: Extended Kalman Filter 6 state**

| Thành phần | Chi tiết |
|------------|----------|
| State vector | $\mathbf{x} = [x, y, z, v_x, v_y, v_z]^T$ (vị trí + vận tốc H-Pad trong world frame) |
| Motion model | Constant velocity: $\dot{x} = v_x$, $\dot{v}_x = 0$ + process noise $Q$ |
| Measurement | ArUco pose (x,y,z) từ vision node, ~30Hz |
| Output | Smooth pose + velocity estimate → dùng cho attractive force + prediction |

**Vai trò trong pipeline**: EKF cung cấp **vận tốc ước lượng** của H-Pad — thông tin này được VO-APF dùng để tính lực hấp dẫn có dự đoán (predictive attraction), giúp drone bám target mượt hơn khi target đổi hướng.

**Phương án thay thế**: Adaptive EKF tự động tune Q, R dựa trên innovation sequence — chỉ triển khai nếu hoàn thành Phase B trước tuần 7.

---

#### 5.3.3. Module Planning — APF & VO-APF cho Tracking & Avoidance

**Lộ trình triển khai 2 giai đoạn (Tăng tính an toàn & chắc chắn cho dự án)**:
- **Giai đoạn 1 (Ưu tiên ban đầu)**: Triển khai thuật toán **APF tiêu chuẩn (Standard APF)** dựa trên vị trí tương đối giữa Quadrotor, H-Pad và vật cản. Mục tiêu: Hoàn thiện nhanh baseline planner, nghiệm thu luồng điều khiển và né vật cản cơ bản.
- **Giai đoạn 2 (Nâng cấp mở rộng)**: Sau khi APF tiêu chuẩn chạy thành công và ổn định, tiến hành nâng cấp lên **Velocity-Adaptive APF (VO-APF)** để điều biến lực đẩy theo vận tốc tương đối, giúp phản ứng sớm và mượt hơn.

> Tham khảo: Ma'arif [8] (APF baseline), Fast-Tracker [3] & EGO-Planner [7] (ý tưởng velocity-awareness)

**Tại sao chọn APF làm baseline thay vì B-spline/EGO-Planner?**
- APF tính toán **nhẹ** (< 20ms/cycle trên Pi 5), phù hợp real-time constraint.
- Dễ triển khai, gỡ lỗi và kiểm thử từng bước.
- APF **tự nhiên** kết hợp attraction (tới target) + repulsion (từ obstacles) trong một framework thống nhất.
- Với sparse obstacles (vật cản rải rác), APF **đủ hiệu quả** mà không cần B-spline trajectory optimization phức tạp.

**Cấu trúc lực trong APF & VO-APF**:

1. **Attractive force (Lực hấp dẫn)**: Hướng Quadrotor về vị trí mục tiêu H-Pad (hoặc vị trí dự đoán từ EKF).
2. **Repulsive force (Lực đẩy vật cản)**:
   - *Giai đoạn APF tiêu chuẩn*: $F_{rep} = k_{rep} \left(\frac{1}{d} - \frac{1}{d_0}\right) \frac{1}{d^2} \hat{\mathbf{d}}$ (phụ thuộc vào khoảng cách $d$).
   - *Giai đoạn VO-APF (Nâng cấp)*: Lực đẩy được nhân thêm hệ số vận tốc tương đối $\mathbf{v}_{rel} \cdot \hat{\mathbf{d}}$ để tăng lực đẩy khi bay thẳng vào vật cản và giảm lực khi bay song song.

3. **Phase-dependent gain**: 
   - FOLLOW: Full APF gain (gain = 1.0)
   - APPROACH: Repulsive gain giảm 50% (ưu tiên hội tụ về landing pad)
   - LAND: APF tắt, chuyển hoàn toàn sang IBVS visual servo.

**Xử lý Local Minima**: Với sparse obstacles, local minima ít xảy ra. Nếu phát hiện drone bị kẹt (velocity < threshold > 3s), áp dụng **virtual wall escape** — thêm lực nhiễu ngẫu nhiên vuông góc để thoát ra.

---

#### 5.3.4. Module Landing — Visual Servo & Precision Touchdown

**Phương án đề xuất: Image-Based Visual Servoing (IBVS) + Multi-phase descent**

> Tham khảo: Keipour [2] (visual servoing landing), Qi [6] (multi-size ArUco), Łuczak [1] (gimbal-assisted landing)

| Pha | Chiến lược | Tham khảo |
|-----|-----------|-----------|
| APPROACH (5m → 0.5m) | Giảm khoảng cách dần, align marker center với image center, gimbal tilt xuống | Kết hợp Keipour [2] (image-space control) + Łuczak [1] (gimbal descent) |
| LAND (< 0.5m) | Hạ thẳng đứng, gimbal −90°, fine alignment bằng ArUco corner sub-pixel | Qi [6]: multi-marker robustness |

**Ý tưởng chính**: Thay vì reconstruct 3D pose phức tạp ở giai đoạn cuối, dùng **IBVS error** (pixel error giữa marker center và image center) trực tiếp làm feedback — nhanh hơn và robust hơn khi altitude thấp.

---

#### 5.3.5. Module Integration — State Machine & Offboard Control

**Phương án đề xuất: Finite State Machine (FSM) + PX4 Offboard API**

| Thành phần | Chi tiết |
|------------|----------|
| FSM | 4 states: SEARCH → FOLLOW → APPROACH → LAND, transition logic rõ ràng |
| Offboard interface | `px4_ros_com` + micro-XRCE-DDS agent |
| Safety | Kill switch (RC ch5/6), watchdog timer (heartbeat), geo-fence |
| Gimbal control | PWM output từ GPIO Pi 5, PID smoothing (EMA filter) |

---

### 5.4. Tổng hợp phương án thuật toán

```
┌──────────────────────────────────────────────────────────────────────┐
│                    ALGORITHM STACK OVERVIEW                          │
│                                                                      │
│  ┌─────────────┐  ┌────────────┐  ┌──────────────┐  ┌───────────┐  │
│  │  VISION     │  │ ESTIMATION │  │  PLANNING    │  │  LANDING  │  │
│  │             │  │            │  │              │  │           │  │
│  │ ArUco Det.  │  │ EKF 6-state│  │ VO-APF       │  │ IBVS      │  │
│  │ Depth Mask  │  │ (pos+vel)  │  │ (vel-adapt.) │  │ (image    │  │
│  │ (N3: new)   │  │            │  │ (N2: new)    │  │  space)   │  │
│  └──────┬──────┘  └─────┬──────┘  └──────┬───────┘  └─────┬─────┘  │
│         │               │                │                 │        │
│         └───────────────┴────────────────┴─────────────────┘        │
│                              │                                       │
│                    ┌─────────▼──────────┐                           │
│                    │  STATE MACHINE     │                           │
│                    │  (FSM 4 states)    │                           │
│                    │  + Gimbal Control  │                           │
│                    │  (N4: new)         │                           │
│                    └─────────┬──────────┘                           │
│                              │                                       │
│                    ┌─────────▼──────────┐                           │
│                    │  PX4 OFFBOARD      │                           │
│                    └────────────────────┘                           │
└──────────────────────────────────────────────────────────────────────┘
```

---

## 6. Phân Công Module Chính và Người Phụ Trách

### 6.1. Phân công theo module

| Module | Người phụ trách | Mô tả trách nhiệm |
|--------|-----------------|---------------------|
| **Vision Pipeline** | **Duy** | ArUco detection, pose estimation, Depth masking (N3), multi-thread RGB/Depth processing |
| **EKF Estimation** | **Duy** | EKF smoother cho H-Pad state (pos + vel), tune Q/R, đánh giá tracking accuracy |
| **APF / VO-APF Planner** | **Tuân** | Triển khai Standard APF trước (baseline), nâng cấp VO-APF sau khi ổn định (N2), follow guidance law |
| **Landing Controller** | **Tuân** | IBVS landing servo, approach trajectory, alignment logic |
| **Mô hình 6DOF & Sim** | **Tuân** | Mô hình UAV Simulink, validate APF / VO-APF trong simulation trước khi port ROS 2 |
| **State Machine & FSM** | **Việt Anh** | FSM 4 states, transition logic, safety monitor, watchdog |
| **Gimbal Control** | **Việt Anh** | Servo PWM control, multi-phase gimbal angle (N4), PID smoothing |
| **PX4 Integration** | **Việt Anh** | Offboard mode, micro-XRCE-DDS setup, RC channel parsing, kill switch |
| **Hardware Assembly** | **Việt Anh** | Lắp ráp camera + gimbal + Pi 5 lên S500 frame, wiring, balancing |

### 6.2. Timeline chi tiết

| Tuần | Duy (Vision/Estimation) | Tuân (Control/Planning) | Việt Anh (Embedded/Integration) | Milestone |
|------|------------------------|------------------------|---------------------------------|-----------|
| **W1** | ArUco Detection + Pose Estimation trên Pi 5 | Mô hình 6DOF UAV Simulink | ROS 2 + PX4 SITL + Gazebo setup | Các module cơ bản chạy độc lập |
| **W2** | Depth Masking pipeline (N3) | Thiết kế & mô phỏng Standard APF trên Simulink | Servo Gimbal hardware + ROS node | Vision + APF + Gimbal đều có prototype |
| **W3** | EKF Smoother (H-Pad velocity) | Tuning Standard APF + Guidance Law | State Machine FSM + Offboard interface | EKF output ổn định, Standard APF hoạt động |
| **W4** | Multi-thread vision optimization | Port Standard APF sang ROS 2; Nghiên cứu mở rộng VO-APF | RC Landing Trigger + Gimbal multi-phase (N4) | Tất cả nodes chạy trên ROS 2 |
| **W5** | **Full system integration SITL** | Landing visual servo (IBVS) | Hardware assembly lên S500 | SITL demo SEARCH→FOLLOW→APPROACH→LAND |
| **W6** | Flight test #1-2: SEARCH + FOLLOW | Flight test với Standard APF; Nâng cấp VO-APF | Flight test: State machine + gimbal | Outdoor flight test bắt đầu |
| **W7** | Flight test #3-5: Full pipeline | Flight test nâng cao với VO-APF | Flight test: RC trigger + safety | Toàn bộ pipeline bay ngoài trời |
| **W8** | Data analysis + metrics đo lường | So sánh hiệu năng Standard APF vs VO-APF | Data analysis + system reliability | KPI evaluation hoàn tất |
| **W9** | Báo cáo + Video demo | Báo cáo + Kết quả simulation vs real | Báo cáo + Hardware documentation | Nộp báo cáo + demo video |

### 6.3. Deliverables

| Deliverable | Responsible | Deadline |
|-------------|-------------|----------|
| Vision ROS 2 node (ArUco + Depth Mask) | Duy | W4 |
| EKF ROS 2 node | Duy | W4 |
| VO-APF ROS 2 node + Simulink validation | Tuân | W5 |
| Landing servo ROS 2 node | Tuân | W5 |
| State machine + Gimbal + PX4 integration | Việt Anh | W5 |
| Full SITL demo video | Cả nhóm | W5 |
| Flight test log data (≥ 5 flights) | Cả nhóm | W7 |
| Final report + demo video | Cả nhóm | W9 |

---

## Tài Liệu Tham Khảo

| # | Tài liệu |
|---|----------|
| [1] | Łuczak, P. & Granosik, G. (2025). *Autonomous UAV Landing and Collision Avoidance System for Unknown Terrain Utilizing Depth Camera with Actively Actuated Gimbal.* Sensors, 25(19), 6165. |
| [2] | Keipour, A. et al. (2022). *Visual Servoing Approach to Autonomous UAV Landing on a Moving Vehicle.* Sensors, 22(17), 6549. |
| [3] | Han, Z. et al. (2021). *Fast-Tracker: A Robust Aerial System for Tracking Agile Target in Cluttered Environments.* IEEE ICRA. |
| [4] | Ji, J. et al. (2021). *Elastic Tracker: A Spatio-temporal Trajectory Planner for Flexible Aerial Tracking.* IEEE RAL. |
| [5] | Pan, N. et al. (2021). *Fast-Tracker 2.0: Improving Autonomy of Aerial Tracking with Active Vision and Human Location Regression.* IET Cyber-Syst. Robot. |
| [6] | Qi, D. et al. (2019). *Autonomous Landing Solution of Low-cost Quadrotor on a Moving Platform.* Robotics Auton. Syst. |
| [7] | Zhou, X. et al. (2020). *EGO-Planner: An ESDF-Free Gradient-Based Local Planner for Quadrotors.* IEEE RAL. |
| [8] | Ma'arif, A. et al. (2021). *Artificial Potential Field Algorithm for Obstacle Avoidance in UAV Quadrotor for Dynamic Environment.* IEEE COMNETSAT. |
