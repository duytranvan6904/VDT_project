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
  stoppage_vz_max_mps: 0.12             # Ngưỡng vận tốc đứng coi như đã dừng (m/s)
  fast_confirmation_duration_s: 0.10    # Thời gian xác nhận nhanh khi có va chạm hoặc PX4 landed (s)
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

---

## 7. BẢNG TỔNG HỢP & GHI CHÚ THAY ĐỔI CHI TIẾT TỪNG FILE SO VỚI BẢN BÀN GIAO ĐẦU THÁNG (COMMIT `274f1c8`)

Bảng dưới đây ghi chú toàn bộ những thay đổi kỹ thuật giữa bản hiện tại và bản commit ban đầu gửi team nhúng ngày 08/10/2026 (`commit 274f1c88: feat(embedded): package algorithms, C++ templates, and GCS tools for Raspberry Pi 5 deployment`):

### 7.1. Cấu hình Hệ thống (`embedded_handover/config/`)

| Tên File | Các thông số thay đổi | Ghi chú & Mục đích kỹ thuật |
| :--- | :--- | :--- |
| [`mission_params.yaml`](file:///home/duy/VDT_project/embedded_handover/config/mission_params.yaml) | - `follow_distance_m`: $3.5 \to 2.2\,\text{m}$<br>- `follow_speed_limit_mps`: $1.8 \to 2.2\,\text{m/s}$<br>- `v_descend_fast_mps`: $0.40 \to 0.55\,\text{m/s}$<br>- `v_descend_touch_mps`: $0.15 \to 0.22\,\text{m/s}$<br>- `v_final_xy_max_mps`: $0.35 \to 1.30\,\text{m/s}$<br>- `final_descent_alt_m`: $0.40 \to 0.60\,\text{m}$<br>- `final_descent_rxy_m`: $0.40 \to 0.85\,\text{m}$<br>- Bổ sung: `corr_fade_low_alt_m: 0.10`, `corr_fade_high_alt_m: 0.20`, `disarm_press_down_vz_mps: -0.12`, `disarm_hold_duration_s: 0.35`<br>- Bổ sung: `auto_reland_after_abort: false` | - Tối ưu cự ly bám và góc nhìn camera ArUco.<br>- Nâng trần vận tốc $XY$ ở pha cuối từ $0.35$ lên $1.30\,\text{m/s}$ để bám kịp bãi di động $1.0\,\text{m/s}$ (trước đây trần $0.35\,\text{m/s}$ làm drone bị tụt lại phía sau).<br>- Tăng tốc độ hạ thẳng đứng trên bãi tĩnh, tránh bị treo chậm.<br>- Cấu hình làm phẳng góc pitch (`corr_fade`) triệt tiêu cắm mũi khi chạm đất.<br>- Khóa an toàn: wave-off thì giữ cao độ $3\,\text{m}$, không tự ý bổ nhào hạ tiếp. |

### 7.2. Các File Thuật toán Tham chiếu Python (`embedded_handover/algorithms/`)

| Tên File Python | Thay đổi chính so với bản đầu tháng | Chi tiết kỹ thuật & Lý do sửa đổi |
| :--- | :--- | :--- |
| [`smc_guidance.py`](file:///home/duy/VDT_project/embedded_handover/algorithms/smc_guidance.py) | - Triển khai cơ chế làm phẳng tư thế `corr_fade`.<br>- Tối ưu hóa profile hạ cao độ theo phân vùng.<br>- Nâng `v_max_approach` ($1.5 \to 1.8\,\text{m/s}$) và `v_final_xy_max` ($0.35 \to 1.30\,\text{m/s}$). | - Khi $r_z \le 0.10\,\text{m}$, $\text{corr\_fade} \to 0$ triệt tiêu thành phần gia tốc bù ngang $kp \cdot dx$, đưa thân drone về tư thế phẳng hoàn toàn ($\theta = 0^\circ$) ngay khoảnh khắc tiếp đất.<br>- Khi $r_z > 0.40\,\text{m}$, duy trì $v_z = -0.55\,\text{m/s}$ giúp hạ nhanh, chỉ giảm xuống $-0.22\,\text{m/s}$ khi sát sàn. |
| [`touchdown_detector.py`](file:///home/duy/VDT_project/embedded_handover/algorithms/touchdown_detector.py) | - Bổ sung chốt khẩn cấp `HARD_DECK_CONTACT`.<br>- Tinh chỉnh điều kiện va chạm `IMPACT_JERK_STOPPED`.<br>- Rút ngắn thời gian xác nhận (debounce) từ $0.40\,\text{s} \to 0.18\,\text{s}$ (hoặc $0.10\,\text{s}$ khi va chạm). | - Xử lý trường hợp khi chân drone chạm sàn ở $z \approx 0.02\,\text{m}$, phản lực tiếp xúc làm $v_z$ vọt lên dương khiến bộ lọc cũ không chịu chốt tiếp đất. `HARD_DECK_CONTACT` chốt ngay nếu $z \le 0.05\,\text{m}$ và đang có lệnh hạ, ngắt motor trong $0.2\,\text{s}$. |
| [`tracking_control.py`](file:///home/duy/VDT_project/embedded_handover/algorithms/tracking_control.py) | - Bổ sung lớp `FollowYawReferenceSelector`.<br>- Thêm tính toán `landing_pitch_target` và `landing_pitch_from_geometry`.<br>- Bổ sung hệ số làm mịn `taper_scale` và giới hạn tốc độ xoay `yaw_rate_limit`. | - Tự động chọn hướng yaw bám theo vector vận tốc bãi hoặc góc quang học mà không gây lộn vòng yaw $360^\circ$ khi camera chúc thẳng đứng.<br>- Điều hòa góc nghiêng pitch mục tiêu theo quỹ đạo tiếp cận. |
| [`mission_fsm_node.py`](file:///home/duy/VDT_project/embedded_handover/algorithms/mission_fsm_node.py) | - Bổ sung biến chốt lệnh `land_latched_` và chính sách `auto_reland_after_abort`.<br>- Tăng thời gian timeout chống chập chờn: `search_timeout_s` ($1.2 \to 5.0\,\text{s}$), `approach_timeout_s` ($3.0 \to 7.0\,\text{s}$).<br>- Cập nhật ngưỡng chuyển trạng thái sang `FINAL_DESCENT` ($0.60\,\text{m} / 0.85\,\text{m}$).<br>- Reset bộ điều khiển yaw khi bắt lại mục tiêu. | - Đảm bảo khi mất mục tiêu trong lúc hạ cánh, drone leo lên lại $3\,\text{m}$ thì giữ nguyên trạng thái `FOLLOW`, không tự động nhảy lại vào `LAND` mà bắt buộc có lệnh từ người điều khiển.<br>- Triệt tiêu hiện tượng xoay yaw giật cục khi mục tiêu vừa tái xuất hiện. |
| [`ibvs_controller.py`](file:///home/duy/VDT_project/embedded_handover/algorithms/ibvs_controller.py) | - Bổ sung bù góc nghiêng thân drone `drone_pitch`.<br>- Khóa xoay yaw khi nhìn chúc đứng `nadir_yaw_cutoff_rad = -60°`. | - Khi camera chúc xuống đất ($<-60^\circ$ hoặc cự ly $< 1.0\,\text{m}$), triệt tiêu lệnh xoay yaw quang học để loại bỏ điểm kỳ dị góc quay. |
| [`target_state_ekf.py`](file:///home/duy/VDT_project/embedded_handover/algorithms/target_state_ekf.py) | - Bổ sung mô hình lan truyền hiệp phương sai CWNA 6 biến trạng thái rời rạc.<br>- Bổ sung cổng khoảng cách Mahalanobis loại bỏ giá trị đo ngoại lai.<br>- Kẹp gia tốc tức thời `max_dv_per_step = 0.40 m/s`. | - Ước lượng vận tốc bãi đáp ổn định, lọc sạch các đột biến do nhiễu nhận diện ArUco gây ra. |
| [`covariance_gate.py`](file:///home/duy/VDT_project/embedded_handover/algorithms/covariance_gate.py) | - Chuyển từ góc nón cứng $30^\circ$ sang phễu hiệp phương sai $2\sigma$ thích nghi.<br>- Mở rộng `pad_radius_m` ($0.25 \to 0.75\,\text{m}$) và `max_uncertainty_final_descent` ($0.35 \to 0.90\,\text{m}$). | - Trước đây góc nón cứng $30^\circ$ từ chối hạ cánh sai khi drone đang ở độ cao bám đuổi tiêu chuẩn ($2.2\,\text{m}$). Phễu thích nghi cho phép chuyển trạng thái hạ cánh tin cậy. |
| [`iapf_core.py` & `apf_planner.py`](file:///home/duy/VDT_project/embedded_handover/algorithms/iapf_core.py) | - Bổ sung phát hiện bẫy thẳng hàng `is_collinear_trap`.<br>- Bổ sung vector lực tiếp tuyến vuông góc $t_{ccw}, t_{cw}$ (`f_tan`). | - Tránh việc drone bị kẹt (stall) bất động khi drone, chướng ngại vật và bãi đáp nằm trên cùng một đường thẳng. |
| [`offboard_commander.py`](file:///home/duy/VDT_project/embedded_handover/algorithms/offboard_commander.py) | - Bổ sung bộ lọc giới hạn gia tốc (slew rate limiter) 3 trục.<br>- Bổ sung cửa sổ giữ lực ép hạ nhẹ $v_z = -0.12\,\text{m/s}$ trong $0.35\,\text{s}$ khi phát hiện tiếp đất kèm burst gói tin force disarm. | - Giúp drone chuyển đổi vận tốc mượt mà, không bị giật dòng động cơ.<br>- Đảm bảo chân drone bám dính mặt bãi đáp và ngắt cánh dứt khoát. |

### 7.3. Các File Thư viện Header C++ (`embedded_handover/cpp_templates/`)

| Tên File Header C++ | Thay đổi chính so với bản đầu tháng | Chi tiết kỹ thuật & Đảm bảo Parity 100% |
| :--- | :--- | :--- |
| [`TrackingControl.hpp`](file:///home/duy/VDT_project/embedded_handover/cpp_templates/TrackingControl.hpp) | Đồng bộ đầy đủ `FollowYawReferenceSelector`, `landing_pitch_target`, `landing_pitch_from_geometry`, và biến `taper_scale`. | Khắc phục hiện tượng lộn vòng yaw trong C++ và hỗ trợ tính góc pitch thăng bằng khi hạ cánh. |
| [`IBVSController.hpp`](file:///home/duy/VDT_project/embedded_handover/cpp_templates/IBVSController.hpp) | Bổ sung `drone_pitch` feedforward và ngưỡng ngắt yaw quang học `nadir_yaw_cutoff_rad = -1.05 rad` ($-60^\circ$). | Đồng bộ 1:1 với logic chống quay yaw vô định khi nhìn thẳng xuống sàn. |
| [`TouchdownDetector.hpp`](file:///home/duy/VDT_project/embedded_handover/cpp_templates/TouchdownDetector.hpp) | Bổ sung kiểm tra tiếp xúc sàn khẩn cấp `hard_deck` (`actual_z <= 0.05 && cmd_down`) và hỗ trợ `fast_confirm_trigger` ở sát đất. | Chốt tiếp đất trong vòng $0.18\,\text{s}$, không bị treo do phản lực sàn. |
| [`TargetStateEKF.hpp`](file:///home/duy/VDT_project/embedded_handover/cpp_templates/TargetStateEKF.hpp) | Thay thế các dòng mã giả chú thích thành phương trình giải tích Kalman hoàn chỉnh: lan truyền CWNA $P = FPF^T + Q$ và cập nhật $x = x + Ky$, $P = (I - KH)P$ kèm kẹp `max_dv_per_step`. | Khắc phục lỗi chí mạng: bản đầu tháng chỉ có mã giả khiến vận tốc bãi đáp trong C++ không bao giờ được cập nhật từ phép đo. Bản mới giải tích đầy đủ không cần thư viện ma trận ngoài. |
| [`CovarianceGate.hpp`](file:///home/duy/VDT_project/embedded_handover/cpp_templates/CovarianceGate.hpp) | Thay thế kiểm tra nón $30^\circ$ bằng phễu sai số thích nghi $2\sigma$, cập nhật bán kính bãi $0.75\,\text{m}$ và trần sai số $0.90\,\text{m}$. | Giúp máy trạng thái C++ chuyển mượt từ `APPROACH` sang `GLIDE_SLOPE` mà không bị từ chối oan. |
| [`IAPFPlanner.hpp`](file:///home/duy/VDT_project/embedded_handover/cpp_templates/IAPFPlanner.hpp) | Bổ sung phát hiện bẫy thẳng hàng `is_collinear_trap` và sinh lực tiếp tuyến vuông góc $t_{ccw}, t_{cw}$ né chướng ngại vật. | Khắc phục lỗi kẹt drone tại $x = 3.45\,\text{m}$ khi đối đầu trực diện với vật cản trên đường thẳng tới goal. |
| [`MissionFSM.hpp`](file:///home/duy/VDT_project/embedded_handover/cpp_templates/MissionFSM.hpp) | Bổ sung chốt `land_latched_`, chính sách `auto_reland_after_abort: false`, nâng timeout lên $5.0\,\text{s} / 7.0\,\text{s}$, cập nhật ngưỡng độ cao $0.60\,\text{m} / 0.85\,\text{m}$. | Đồng bộ chính sách can thiệp an toàn và triệt tiêu chuyển đổi trạng thái chập chờn. |
| [`SMCGuidance.hpp`](file:///home/duy/VDT_project/embedded_handover/cpp_templates/SMCGuidance.hpp) | Bổ sung hệ số làm phẳng tư thế `corr_fade`, phân tầng tốc độ hạ thẳng đứng $-0.55\,\text{m/s} / -0.22\,\text{m/s}$, và cập nhật các ngưỡng vận tốc tối đa. | Đồng bộ công thức triệt tiêu cắm mũi và tối ưu tốc độ hạ dứt khoát trên bãi tĩnh. |
| [`test_cpp_pipeline.cpp`](file:///home/duy/VDT_project/embedded_handover/cpp_templates/test_cpp_pipeline.cpp) *(MỚI)* | Chương trình mô phỏng vòng kín khép kín độc lập, kiểm thử toàn diện toàn bộ các file C++ trên cả 3 kịch bản: bãi di động $1.0\,\text{m/s}$, bãi tĩnh, và né vật cản 3D. | Đội nhúng có thể biên dịch và chạy trực tiếp trên Pi 5 bằng lệnh: `g++ -std=c++17 -O3 -Iembedded_handover/cpp_templates test_cpp_pipeline.cpp -o sim && ./sim` để nghiệm thu nhanh. |

