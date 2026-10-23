# HƯỚNG DẪN TRIỂN KHAI THUẬT TOÁN TỰ HÀNH & HẠ CÁNH CHÍNH XÁC LÊN RASPBERRY PI 5

> **Tài liệu Kỹ thuật Nội bộ — Dự án VDT Precision Landing**  
> **Nền tảng mục tiêu**: Raspberry Pi 5 (8GB) + Pixhawk 6C / PX4 Autopilot + Camera Intel RealSense D435i / D455  
> **Phiên bản thuật toán**: v2.4 (Đã kiểm chứng chuẩn công nghiệp & nghiệm thu mô phỏng Gazebo Harmonic)

---

## 1. TỔNG QUAN HỆ THỐNG VÀ KIẾN TRÚC PHẦN CỨNG TRÊN RASPBERRY PI 5

```
 +-----------------------------------------------------------------------------------+
 |                             RASPBERRY PI 5 (COMPANION COMPUTER)                  |
 |                                                                                   |
 |  +-----------------------+     +------------------------+     +----------------+  |
 |  | ArUco Detector        |     | Target State EKF       |     | IAPF Planner   |  |
 |  | (Dual-Scale Marker)   | --> | (6-DOF Kinematic Gate) | --> | (3D Obstacle)  |  |
 |  +-----------------------+     +------------------------+     +----------------+  |
 |             |                              |                          |           |
 |             v                              v                          v           |
 |  +-----------------------+     +------------------------+     +----------------+  |
 |  | IBVS Gimbal/Yaw       |     | Covariance Gate        | --> | Mission FSM    |  |
 |  | (Visual Servo Nadir)  |     | (Adaptive Ellipsoid)   |     | (Central Core) |  |
 |  +-----------------------+     +------------------------+     +----------------+  |
 |                                            |                          |           |
 |                                            v                          v           |
 |                                +------------------------+     +----------------+  |
 |                                | Touchdown Detector     | <-- | SMC Guidance   |  |
 |                                | (Kinematic Drift-Free) |     | (45° Glide)    |  |
 |                                +------------------------+     +----------------+  |
 |                                            |                          |           |
 |                                            v                          v           |
 |                                +-----------------------------------------------+  |
 |                                | Offboard Commander (Attitude Leveling & Slew) |  |
 |                                +-----------------------------------------------+  |
 +----------------------------------------|------------------------------------------+
                                          | UART TELEM2 (921600 baud) / MAVLink
                                          v
 +-----------------------------------------------------------------------------------+
 |                             PIXHAWK 6C (AUTOPILOT PX4 v1.14+)                     |
 |  - Multicopter Attitude Rate Controller (250 Hz)                                  |
 |  - Position / Velocity Trajectory Tracker (50 Hz)                                 |
 |  - Emergency Manual Override Monitor (Auto Disengage Offboard on RC stick touch) |
 +-----------------------------------------------------------------------------------+
```

### 1.1. Giao tiếp Phần cứng & Pinout kết nối
1. **Kết nối Pi 5 <-> Pixhawk 6C**:
   - Sử dụng cổng **UART0 / ttyAMA0** của Pi 5 (Chân GPIO 14-TXD, GPIO 15-RXD, GND) nối sang cổng **TELEM2** của Pixhawk 6C.
   - Tốc độ baud: **921600 bps** (bắt buộc để đảm bảo truyền gói tin Offboard 50Hz không bị trễ buffer).
   - Thiết lập PX4 parameter: `SER_TEL2_BAUD = 921600`, `MAV_2_CONFIG = TELEM 2`, `MAV_2_MODE = Onboard`.
2. **Camera RealSense D435i / D455**:
   - Cắm vào cổng **USB 3.0 (màu xanh)** trên Pi 5 để đảm bảo băng thông truyền đồng thời RGB 640x480 @ 30fps và Depth Stream.
3. **Nguồn cấp**:
   - Pi 5 cần nguồn độc lập 5V/5A (qua mạch hạ áp BEC chất lượng cao từ pin LiPo 4S/6S), **tuyệt đối không lấy nguồn 5V từ Pixhawk** để tránh sụt áp gây reset chip.

---

## 2. TỔNG HỢP CÁC BẢN VÁ THUẬT TOÁN QUAN TRỌNG VỪA HOÀN THIỆN

Đội ngũ nhúng khi triển khai code C++ (hoặc Python) lên Pi 5 cần lưu ý đặc biệt 4 cải tiến then chốt sau đây để tránh lỗi rơi máy bay hoặc lật thân:

### 2.1. Triệt tiêu Hiện tượng Chúc Mũi / Cắm Đầu khi Disarm trên Bãi Chạy 1.0 m/s
* **Nguyên nhân vật lý**: Khi bay bám bãi $1.0\,\text{m/s}$, drone bắt buộc phải nghiêng góc pitch âm (chúc mũi $\theta \approx -5^\circ$). Khi chân trước chạm sàn trước, phản lực nâng mũi lên khiến vòng lặp điều khiển góc pitch của PX4 tăng ga 2 rotor sau để dìm mũi xuống. Hai chân trước thành điểm tựa đòn bẩy (fulcrum) làm đuôi drone bị nhấc bổng cắm thẳng đầu xuống đất.
* **Công thức xử lý bắt buộc (Touchdown Attitude Leveling - `corr_fade`)**:
  Trong khối tính vận tốc hạ cánh cuối cùng (`SMCGuidance` hoặc FSM fallback):
  $$\text{corr\_fade} = \text{clip}\left(\frac{r_z - 0.10}{0.10}, 0.0, 1.0\right)$$
  $$v_{x,\text{cmd}} = \text{clip}(kp \cdot \text{corr\_fade} \cdot dx, -v_{\text{final\_xy\_max}}, v_{\text{final\_xy\_max}}) + v_{x,\text{target}}$$
  $$v_{y,\text{cmd}} = \text{clip}(kp \cdot \text{corr\_fade} \cdot dy, -v_{\text{final\_xy\_max}}, v_{\text{final\_xy\_max}}) + v_{y,\text{target}}$$
  * **Ý nghĩa**: Khi độ cao tương đối $r_z \le 0.10\,\text{m}$ (sắp chạm sàn), hệ số $\text{corr\_fade} \to 0$, triệt tiêu hoàn toàn thành phần bù vị trí $kp \cdot dx$. Drone chỉ duy trì vận tốc tịnh tiến đúng bằng vận tốc bãi đáp, **góc pitch/roll tự động duỗi thẳng về $0.0^\circ$ (thăng bằng phẳng)**, giúp cả 4 chân tiếp đất cùng lúc.
* **Duy trì lực ép nhẹ trong cửa sổ Disarm (`offboard_commander`)**:
  * Khi nhận tín hiệu chạm đất, gửi burst 5 gói tin `FORCE DISARM` (`param1=0, param2=21196`), đồng thời **tiếp tục gửi setpoint vận tốc bãi đáp kèm vận tốc hạ nhẹ $v_z = -0.12\,\text{m/s}$** trong tối đa $0.35\,\text{s}$ cho tới khi PX4 xác nhận động cơ đã ngắt hoàn toàn (`armed == false`). Lực ép nhẹ giữ chân drone bám dính mặt sàn, không bị nảy đuôi.

### 2.2. Tối ưu Tốc độ Hạ Thẳng Đứng trên Bãi Tĩnh (Không bị Treo Chậm)
* **Pha APPROACH khi đã căn giữa ($h_{\text{err}} \le 0.45\,\text{m}$)**:
  $$v_z = \begin{cases} -0.50\,\text{m/s} & \text{nếu } z > 1.2\,\text{m} \\ -0.35\,\text{m/s} & \text{nếu } 0.6 < z \le 1.2\,\text{m} \end{cases}$$
* **SMC Glide Descent Profile**:
  $$v_{\text{descent}} = \min\left(v_{\text{descend\_fast}}, \max(0.35, 0.40 \cdot r_z)\right) \implies v_z = -v_{\text{descent}}$$
  * Trên bãi tĩnh ($v_{xy} = 0$), drone vẫn duy trì tốc độ hạ dứt khoát $0.55\,\text{m/s}$ ở độ cao $> 1.5\,\text{m}$ và giảm êm dần về $0.22\,\text{m/s}$ khi tiếp sàn.
* **Cập nhật tham số**:
  - `v_descend_fast`: $0.55\,\text{m/s}$ (trước là $0.40\,\text{m/s}$).
  - `v_descend_touch`: $0.22\,\text{m/s}$ (trước là $0.15\,\text{m/s}$).
  - `land_fallback_xy_speed_limit`: $1.35\,\text{m/s}$ (trước là $0.65\,\text{m/s}$, đủ công suất bám bãi $1.0\,\text{m/s}$).
  - `v_final_xy_max`: $1.30\,\text{m/s}$.

### 2.3. Triệt tiêu Hiện tượng Xoay Yaw Linh tinh (Nadir Singularity Avoidance)
* **Khóa góc Yaw khi nhìn chúc đứng (Nadir Cutoff)**:
  - Khi cự ly ngang tới mục tiêu $d_{xy} < 1.0\,\text{m}$ hoặc góc pitch gimbal $<-60^\circ$ (nhìn thẳng xuống đất), **vô hiệu hóa hoàn toàn lệnh xoay yaw theo vector sai số $(dx, dy)$**.
  - Việc xoay yaw khi nhìn thẳng xuống đất sinh ra điểm kỳ dị $\text{atan2}(dy, dx)$, làm ảnh xoay tròn như mâm quay và thân drone giật lắc. Tại cự ly gần này, chỉ giữ hướng yaw ổn định và để APF/SMC điều khiển tịnh tiến XY.

### 2.4. Chính sách An toàn khi Hủy Hạ Cánh (Go-Around / Abort Policy)
* Khi quá trình hạ cánh bị hủy do mất dấu mục tiêu quá timeout:
  - Cưỡng bức đặt `land_requested = false` (hủy chốt lệnh hạ cánh).
  - Drone tự động leo ngược lên độ cao an toàn $3.0\,\text{m}$, chuyển về trạng thái `FOLLOW` bám mục tiêu từ xa.
  - **Tuyệt đối không tự động bổ nhào xuống hạ cánh tiếp** nếu chưa có lệnh xác nhận mới từ phi công / trạm mặt đất GCS.

---

## 3. THƯ VIỆN C++ MẪU ĐÃ CUNG CẤP TRONG REPOSITORY

Toàn bộ thuật toán lõi đã được đóng gói thành các file C++ Header-only độc lập (decoupled khỏi ROS/Gazebo) trong thư mục `embedded_handover/cpp_templates/`:

| Tên File Header C++ | Phân hệ Chức năng | Đầu vào chính | Đầu ra chính |
| :--- | :--- | :--- | :--- |
| [`Types.hpp`](file:///home/duy/VDT_project/embedded_handover/cpp_templates/Types.hpp) | Định nghĩa cấu trúc dữ liệu chung | Vector3, Quaternion, State | Các enum Phase, TrackingMode |
| [`SMCGuidance.hpp`](file:///home/duy/VDT_project/embedded_handover/cpp_templates/SMCGuidance.hpp) | Dẫn đường trượt 3D SMC Glide Slope | Vị trí/Vận tốc Drone & Bãi đáp | Vector vận tốc `[vx, vy, vz]`, Yaw rate |
| [`TouchdownDetector.hpp`](file:///home/duy/VDT_project/embedded_handover/cpp_templates/TouchdownDetector.hpp) | Nhận diện tiếp đất Kinematic Stoppage | Altitude $z$, Actual $v_z$, Cmd $v_z$ | Cờ `touchdown_confirmed` (Latched) |
| [`CovarianceGate.hpp`](file:///home/duy/VDT_project/embedded_handover/cpp_templates/CovarianceGate.hpp) | Cổng an toàn hiệp phương sai EKF | Ma trận hiệp phương sai $P_{xy}$ | Cờ `safe_to_land`, $R_{\text{switch}}$ |
| [`IAPFPlanner.hpp`](file:///home/duy/VDT_project/embedded_handover/cpp_templates/IAPFPlanner.hpp) | Quy hoạch lực thế nhân tạo né vật cản | Vị trí chướng ngại vật, Goal pos | Vector vận tốc APF 3D |
| [`IBVSController.hpp`](file:///home/duy/VDT_project/embedded_handover/cpp_templates/IBVSController.hpp) | Visual Servoing hướng Gimbal/Yaw | Tọa độ pixel $(u, v)$, Góc gimbal | Lệnh Gimbal Pitch, Drone Yaw |
| [`MissionFSM.hpp`](file:///home/duy/VDT_project/embedded_handover/cpp_templates/MissionFSM.hpp) | Máy trạng thái nhiệm vụ toàn chu trình | Trạng thái EKF, Lệnh phi công | Trạng thái `MissionPhase` tiếp theo |
| [`TrackingControl.hpp`](file:///home/duy/VDT_project/embedded_handover/cpp_templates/TrackingControl.hpp) | Ghép vận tốc feedforward & Slew rate | Guidance vel, Feedforward vel | Vận tốc giới hạn gia tốc |

---

## 4. HƯỚNG DẪN BUILD & TỐI ƯU HIỆU NĂNG TRÊN RASPBERRY PI 5

### 4.1. Cài đặt Môi trường Hệ điều hành trên Pi 5
1. Khuyến nghị cài đặt **Ubuntu 24.04 Server (64-bit)** hoặc **Ubuntu 22.04 LTS** + **ROS 2 Humble**.
2. Cài đặt các thư viện phụ thuộc bắt buộc:
   ```bash
   sudo apt-get update && sudo apt-get install -y \
       build-essential cmake git libeigen3-dev \
       libopencv-dev liblibrealsense2-dev \
       ros-humble-rclcpp ros-humble-geometry-msgs \
       ros-humble-sensor-msgs ros-humble-nav-msgs \
       python3-pip python3-pymavlink
   ```

### 4.2. Cờ Tối ưu Trình biên dịch C++ (GCC/Clang cho ARM Cortex-A76)
Khi tạo `CMakeLists.txt` trên Raspberry Pi 5, hãy bật các cờ tối ưu hóa kiến trúc vi xử lý sau:
```cmake
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_BUILD_TYPE Release)

# Tối ưu hóa sâu cho 4 nhân Cortex-A76 trên Pi 5:
add_compile_options(
    -O3
    -march=armv8.2-a+crypto+crc
    -mtune=cortex-a76
    -ffast-math
    -funroll-loops
    -pthread
)
```

### 4.3. Thiết lập Quyền Ưu tiên Thời gian Thực (Real-Time Scheduling)
Để tránh vòng lặp điều khiển Offboard 50Hz bị Linux ngắt nhịp (jitter):
1. Cấu hình quyền `limits.conf`:
   ```bash
   sudo sh -c 'echo "ubuntu soft rtprio 99\nubuntu hard rtprio 99" >> /etc/security/limits.conf'
   ```
2. Trong mã nguồn C++ của node điều khiển chính:
   ```cpp
   #include <pthread.h>
   #include <sched.h>

   void set_realtime_priority() {
       struct sched_param param;
       param.sched_priority = 80; // Cao hơn các tiến trình người dùng thông thường
       if (pthread_setschedparam(pthread_self(), SCHED_FIFO, &param) != 0) {
           perror("Failed to set SCHED_FIFO priority");
       }
   }
   ```

---

## 5. BẢNG THAM SỐ CHUẨN CẤU HÌNH (PARSED YAML MẪU)

Trích xuất từ file tham số chuẩn [`simulation/config/mission_params.yaml`](file:///home/duy/VDT_project/simulation/config/mission_params.yaml) để đội nhúng nạp vào hệ thống:

```yaml
# Cấu hình chuẩn nạp cho Raspberry Pi 5
precision_landing:
  # 1. Bám bắt mục tiêu & Lực thế APF
  follow_distance_m: 2.2                # Cự ly bám ngang (m)
  cruise_altitude_m: 3.0                # Độ cao hành trình (m)
  follow_speed_limit_mps: 2.2           # Giới hạn tốc độ bám đuổi (m/s)
  obstacle_clearance_margin_m: 0.35     # Khoảng cách an toàn tối thiểu với vật cản

  # 2. Bộ dẫn đường trượt hạ cánh SMC
  smc_v_max_mps: 1.80                   # Vận tốc tiếp cận tối đa (m/s)
  v_final_xy_max_mps: 1.30              # Trần bù vận tốc bãi di động (m/s)
  v_descend_fast_mps: 0.55              # Tốc độ hạ cao trong Glide & bãi tĩnh (m/s)
  v_descend_touch_mps: 0.22             # Tốc độ hạ sát sàn tiếp đất (m/s)
  final_descent_rxy_m: 0.85             # Bán kính phễu bắt đầu hạ thẳng đứng (m)
  final_descent_alt_m: 0.60             # Độ cao bắt đầu hạ thẳng đứng (m)

  # 3. Chống chúc mũi & Điều hòa tiếp sàn
  corr_fade_low_alt_m: 0.10             # Độ cao triệt tiêu 100% gia tốc nghiêng (m)
  corr_fade_high_alt_m: 0.20            # Độ cao bắt đầu làm mịn tư thế thăng bằng (m)
  disarm_press_down_vz_mps: -0.12       # Lực ép nhẹ giữ chân bám sàn khi disarm (m/s)
  disarm_hold_duration_s: 0.35          # Cửa sổ giữ setpoint chờ PX4 ngắt motor (s)

  # 4. Phát hiện chạm sàn (Touchdown Detector)
  optical_height_threshold_m: 0.35      # Cự ly quang học camera - sàn (m)
  touchdown_max_altitude_m: 0.14        # Trần độ cao chạm sàn thực tế của x500 (m)
  confirmation_duration_s: 0.18         # Thời gian xác nhận chạm đất dứt khoát (s)
  stoppage_vz_max_mps: 0.08             # Ngưỡng vận tốc đứng coi như đã dừng (m/s)
  jerk_impact_threshold: 3.50           # Ngưỡng giật gia tốc va chạm sàn (m/s^3)

  # 5. An toàn & Can thiệp tay
  max_accel_mps2: 1.60                  # Giới hạn gia tốc tịnh tiến (góc nghiêng <= 9.5°)
  auto_reland_after_abort: false        # TẮT: Wave-off thì giữ cao độ 3m, chờ lệnh mới
```

---

## 6. CHECKLIST QUY TRÌNH TEST THỰC TẾ TRÊN DRONE (BENCH TEST -> FLIGHT TEST)

1. **Bước 1: Bench Test trên mặt đất (Không gắn cánh quạt)**:
   - Cấp nguồn Pi 5 và Pixhawk. Kiểm tra tín hiệu nhịp tim Heartbeat MAVLink trên topic `/fmu/in/trajectory_setpoint` hoặc lệnh MAVLink qua `mavlink_shell.py`.
   - Giơ board ArUco trước camera: Kiểm tra góc quay Gimbal và lệnh xoay Yaw bám theo marker.
   - Thử nâng hạ board ArUco để kiểm tra EKF target estimate có mượt mà không bị bước nhảy $\Delta v > 0.4\,\text{m/s}$.
2. **Bước 2: Test an toàn Cắt lệnh Thủ công (Manual Override Test)**:
   - Arm drone trên bàn test, bật OFFBOARD.
   - Gạt cần ga/pitch trên tay điều khiển RC: Đảm bảo Pixhawk lập tức nhả chế độ OFFBOARD về POSCTL/ALTCTL và Pi 5 lập tức dừng phát lệnh tự động.
3. **Bước 3: Test bay thực tế cất cánh & Bám bãi tĩnh**:
   - Cất cánh lên $3.0\,\text{m}$, drone tự động tìm bãi đáp, vào `FOLLOW`, ra lệnh hạ cánh.
   - Quan sát tốc độ hạ thẳng đứng: Đảm bảo drone hạ dứt khoát $0.55\,\text{m/s}$ ở trên cao và tiếp đất êm dịu trong $< 5\,\text{s}$.
4. **Bước 4: Test hạ cánh trên xe chạy $1.0\,\text{m/s}$**:
   - Xe chở bãi đáp chạy thẳng đều $1.0\,\text{m/s}$. Ra lệnh hạ cánh.
   - Quan sát khoảnh khắc tiếp đất: Nhờ cơ chế `corr_fade`, thân drone phải phẳng lì ($\text{Pitch} \approx 0^\circ$), 4 chân tiếp đất cùng lúc, Disarm dứt khoát mà không bị ngả đầu chúc mũi.
