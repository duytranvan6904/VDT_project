# 📦 TÀI LIỆU BÀN GIAO TOÀN DIỆN PIPELINE ĐIỀU KHIỂN & HẠ CÁNH CHÍNH XÁC (C++ EMBEDDED RASPBERRY PI 5)

> **Dự án**: Hệ thống Quadrotor Tự hành Bám mục tiêu & Hạ cánh Chính xác trên Bãi đáp ArUco H-Pad  
> **Tác giả bàn giao**: Duy (Algorithm & Simulation Lead)  
> **Đối tượng tiếp nhận**: Thành viên Team Lập trình Nhúng (Embedded C++ Engineer)  
> **Phần cứng mục tiêu**: Raspberry Pi 5 (Companion Computer) + Pixhawk (PX4 Autopilot) + Intel RealSense D435i  
> **Phiên bản mã nguồn**: Nâng cấp từ branch `feature/iapf-planner` lên `feature/precision-landing` (Đã kiểm chứng thành công 100% trên SITL)

---

## 📑 MỤC LỤC
1. [Kiến trúc Hệ thống Thực nghiệm](#1-kiến-trúc-hệ-thống-thực-nghiệm)
2. [Cấu trúc Gói Bàn giao (`embedded_handover/`) & Chi tiết từng File](#2-cấu-trúc-gói-bàn-giao-embedded_handover--chi-tiết-từng-file)
3. [SO SÁNH & CÁC ĐIỂM THAY ĐỔI CỐT LÕI SO VỚI BRANCH `iapf-planner`](#3-so-sánh--các-điểm-thay-đổi-cốt-lõi-so-với-branch-iapf-planner)
4. [Hướng dẫn Thiết lập Mạng Nội bộ (Mobile Hotspot) & Khắc phục Lỗi Multicast](#4-hướng-dẫn-thiết-lập-mạng-nội-bộ-mobile-hotspot--khắc-phục-lỗi-multicast)
5. [Hướng dẫn Thiết lập & Vận hành trên Máy tính Trạm (Host PC / GCS)](#5-hướng-dẫn-thiết-lập--vận-hành-trên-máy-tính-trạm-host-pc--gcs)
6. [Hướng dẫn Cài đặt & Triển khai C++ trên Raspberry Pi 5](#6-hướng-dẫn-cài-đặt--triển-khai-c-trên-raspberry-pi-5)
7. [Giao thức Giao tiếp Giữa Pi 5 và Pixhawk (PX4)](#7-giao-thức-giao-tiếp-giữa-pi-5-và-pixhawk-px4)
8. [Quy trình Vận hành Thực nghiệm An toàn (Pre-flight Checklist)](#8-quy-trình-vận-hành-thực-nghiệm-an-toàn-pre-flight-checklist)

---

## 1. Kiến trúc Hệ thống Thực nghiệm

Toàn bộ hệ thống bay thực tế bao gồm 3 thành phần liên kết không dây và có dây:

```
┌────────────────────────────────┐
│   MÁY TÍNH TRẠM (HOST GCS)    │
│  - Phát lệnh /operator/land... │
│  - Giám sát FSM, 2σ, Odometry  │
│  - Emergency Abort/Kill Switch │
└───────────────▲────────────────┘
                │ Wi-Fi 2.4/5GHz (Hotspot điện thoại cá nhân)
                │ Unicast UDP / ROS 2 DDS (Domain ID: 42)
┌───────────────▼─────────────────────────────────────────────────┐
│              RASPBERRY PI 5 (COMPANION COMPUTER)                │
│                                                                 │
│  [Camera D435i] ──► [Dual-Scale ArUco] ──► [Target EKF (6-state)]│
│                             │                       │           │
│                             ▼                       ▼           │
│                       [IBVS Gimbal/Yaw]      [Covariance Gate]  │
│                                                     │           │
│  [I-APF 3D Avoidance] ◄─────────────────────────────┤           │
│            │                                        ▼           │
│            └────────► [Mission FSM Node] ◄─── [SMC 45° Guidance]│
│                              │                      ▲           │
│                              ▼                      │           │
│                    [Offboard Commander] ◄─── [Touchdown Detect] │
└──────────────────────────────┬──────────────────────────────────┘
                               │ UART TELEM2 (/dev/ttyAMA0 @ 921600 baud)
                               │ hoặc USB CDC (/dev/ttyACM0)
                               │ MAVLink Offboard / Micro XRCE-DDS
┌──────────────────────────────▼──────────────────────────────────┐
│             PIXHAWK FLIGHT CONTROLLER (PX4 AUTOPILOT)           │
│  - PX4 EKF2: Odometry, IMU, Baro, GPS                           │
│  - Position/Velocity Controller (Offboard mode)                 │
│  - Motor Actuation & Force Disarm Failsafe                      │
└─────────────────────────────────────────────────────────────────┘
```

---

## 2. Cấu trúc Gói Bàn giao (`embedded_handover/`) & Chi tiết từng File

Toàn bộ tài nguyên phục vụ việc chuyển đổi sang C++ nhúng được đóng gói tại thư mục `embedded_handover/`:

```
embedded_handover/
├── README.md                      # Tài liệu bàn giao chi tiết (file này)
├── algorithms/                    # Mã nguồn thuật toán gốc (Python logic reference)
│   ├── target_state_ekf.py        # EKF 6-state ước lượng vị trí, vận tốc, covariance bãi đáp
│   ├── iapf_core.py               # Thuật toán I-APF 3D tránh vật cản, thoát bẫy cực tiểu địa phương
│   ├── tracking_control.py        # Logic xác thực chuyển động mục tiêu (TargetMotionGate)
│   ├── ibvs_controller.py         # Visual servoing điều khiển góc pitch gimbal và yaw drone
│   ├── covariance_gate.py         # Giám sát an toàn 2σ và phễu nón trước khi cho phép hạ cánh
│   ├── smc_guidance.py            # Dẫn đường dốc trượt 45° Sliding Mode Control & tiếp đất mềm
│   ├── touchdown_detector.py      # Bộ dò tiếp đất đa tầng độc lập trôi dạt barometer
│   ├── mission_fsm_node.py        # Máy trạng thái hữu hạn 7 pha điều phối toàn bộ chu trình
│   └── offboard_commander.py      # Bộ chuyển đổi ENU->NED, gửi TrajectorySetpoint và Force Disarm
├── vision/                        # Xử lý hình ảnh và cấu hình bãi đáp
│   ├── aruco_detector.py          # Class DualScaleArUcoDetector (PnP Board Pose Estimation)
│   ├── dual_scale_board_config.yaml# Cấu hình 3D tọa độ các góc của 2 marker trên bãi đáp
│   ├── generate_dual_scale_board.py# Script tạo ảnh và kiểm tra hình học bãi đáp
│   ├── dual_scale_aruco_board_A0_52_5cm.pdf # File in ấn tỉ lệ chuẩn 1:1 khổ A0
│   ├── realsense_stream.py        # Wrapper đọc luồng ảnh RGB-D từ camera RealSense D435
│   └── main_aruco_detector.py     # Entry point chạy standalone vision trên máy tính nhúng
├── config/                        # Cấu hình tham số và mạng
│   ├── mission_params.yaml        # Toàn bộ tham số hệ thống đã tinh chỉnh chuẩn
│   └── cyclonedds_hotspot.xml     # Cấu hình Unicast DDS cho mạng phát từ điện thoại
├── cpp_templates/                 # Template C++20 Header-Only chuẩn (đã biên dịch kiểm chứng)
│   ├── Types.hpp                  # Định nghĩa Vector3, Odometry, FSM Enums, Structs kết quả
│   ├── TargetStateEKF.hpp         # Lớp C++ TargetStateEKF với Mahalanobis Gating
│   ├── IAPFPlanner.hpp            # Lớp C++ IAPFPlanner với GNRON và Tangent Escape
│   ├── TrackingControl.hpp        # Lớp C++ TargetMotionGate và hàm gộp vận tốc bám
│   ├── CovarianceGate.hpp         # Lớp C++ CovarianceGate tính bán kính 2σ và phễu nón 30°
│   ├── SMCGuidance.hpp            # Lớp C++ SMCGuidance với Power Reaching Law (n/m = 3/5)
│   ├── TouchdownDetector.hpp      # Lớp C++ TouchdownDetector chốt tiếp đất động học 0.35s
│   ├── IBVSController.hpp         # Lớp C++ IBVSController với chế độ Nadir Tilt -85°
│   └── MissionFSM.hpp             # Lớp C++ MissionFSM quản lý chuyển pha an toàn
└── host_tools/                    # Công cụ vận hành trên Máy tính trạm
    ├── host_env_setup.sh          # Script cấu hình môi trường ROS 2 và mạng Unicast trên PC
    └── gcs_operator_terminal.py   # Dashboard điều khiển trạm: phát lệnh Land, Abort, xem Telemetry
```

---

## 3. SO SÁNH & CÁC ĐIỂM THAY ĐỔI CỐT LÕI SO VỚI BRANCH `iapf-planner`

> [!IMPORTANT]
> Trước đây bạn đã nhận mã nguồn từ branch `feature/iapf-planner`. Trong phiên bản hiện tại (`feature/precision-landing`), toàn bộ hệ thống đã được nâng cấp toàn diện để chuyển từ việc "chỉ né vật cản và bám mục tiêu ở cự ly xa" sang **"chu trình hoàn chỉnh: Tìm kiếm ➔ Bám đuổi ➔ Tiếp cận ➔ Hạ cánh dốc trượt 45° ➔ Tiếp đất êm ái ➔ Ngắt động cơ tức thì"**.
> 
> Dưới đây là **7 thay đổi kỹ thuật lớn** mà thành viên nhúng cần đặc biệt lưu ý khi chuyển thể code:

### 1. Nâng cấp Nhận thức: Nested Dual-Scale ArUco Board thay cho Marker Đơn lẻ
* **Phiên bản cũ (`iapf-planner`)**:
  * Chỉ nhận diện 1 marker duy nhất (Outer ID 42).
  * **Hạn chế nghiêm trọng**: Khi drone hạ thấp xuống dưới $0.6\text{m}$, marker to bị phóng đại tràn ra ngoài rìa khung ảnh camera (FOV), gây mất dấu hoàn toàn ngay khoảnh khắc chuẩn bị chạm đất khiến drone bị mất phương hướng.
* **Phiên bản hiện tại (`precision-landing`)**:
  * Sử dụng **Dual-Scale ArUco Board** khổ A0 (Hai marker đa tỉ lệ bố trí trên cùng tấm bạt, không đè lồng lên nhau để bảo toàn mã bit ArUco): Marker lớn ID 42 (52cm) đặt lệch trái dùng cho tầm xa ($0.8\text{m} \to 5.0\text{m}$) và Marker nhỏ ID 43 (10cm) đặt **chính giữa tâm bãi đáp** dùng cho tiếp đất cự ly sát đất ($0.05\text{m} \to 1.2\text{m}$).
  * Sử dụng thuật toán `cv::aruco::estimatePoseBoard()` thay vì `estimatePoseSingleMarkers()`.
  * **Lưu ý khi viết C++**: Cả 2 marker đều được định nghĩa tọa độ 3D góc thực tế quy về **tâm hình học của bãi đáp $(0,0,0)$ (trùng tâm marker nhỏ ID 43)**. Khi camera chuyển từ nhìn marker to sang marker nhỏ, tọa độ 3D trả về **liên tục tuyệt đối, không có bước nhảy (zero discontinuity)**.

### 2. Tách biệt Guidance Velocity và Target Motion Feedforward (`TargetMotionGate`)
* **Phiên bản cũ (`iapf-planner`)**:
  * Vận tốc né vật cản APF và vận tốc mục tiêu từ EKF bị cộng dồn trực tiếp vào nhau. Khi bãi đáp đứng yên, nhiễu ước lượng vận tốc của EKF khiến drone bị rung giật lắc lư qua lại quanh điểm bám.
* **Phiên bản hiện tại (`precision-landing`)**:
  * Tách làm 2 topic rõ ràng: `/apf/guidance_velocity_cmd` (vector hướng về goal và né vật) và `/apf/target_velocity_ff` (vector bù vận tốc bãi đáp).
  * Tích hợp bộ lọc trễ **`TargetMotionGate`**: Chỉ khi EKF đo được vận tốc bãi đáp $\ge 0.35\text{m/s}$ liên tục trong ít nhất 3 frames thì mới kích hoạt Feedforward; khi vận tốc giảm về $\le 0.18\text{m/s}$ trong 4 frames thì tắt hoàn toàn. Khi mục tiêu đứng yên, Feedforward bằng 0 tuyệt đối giúp drone treo đứng im như khóa vị trí.

### 3. Module Hoàn toàn Mới 1: `Covariance Gate` — Giám sát An toàn Hiệp phương sai Kép
* **Phiên bản cũ (`iapf-planner`)**:
  * Chưa có module này. Việc quyết định hạ cánh phụ thuộc mù quáng vào khoảng cách hình học đơn thuần, bất chấp việc EKF có đang bị nhiễu hay trôi dạt hay không.
* **Phiên bản hiện tại (`precision-landing`)**:
  * File tham chiếu: [`covariance_gate.py`](file:///home/duy/VDT_project/embedded_handover/algorithms/covariance_gate.py) / [`CovarianceGate.hpp`](file:///home/duy/VDT_project/embedded_handover/cpp_templates/CovarianceGate.hpp).
  * **Toán học cốt lõi**:
    1. Dung hợp ma trận hiệp phương sai ước lượng bãi đáp $P_{target}$ (từ Target EKF) và sai số vị trí drone $P_{drone}$ (từ PX4 odometry): $P_{relative} = P_{target} + P_{drone}$.
    2. Tính giá trị riêng cực đại của ma trận mặt phẳng ngang $\lambda_{max}(P_{relative, xy})$.
    3. Xác định bán kính không định $2\sigma = 2\sqrt{\lambda_{max}}$.
    4. **Điều kiện an toàn hạ cánh**: $2\sigma \le 0.45\text{m}$ VÀ drone nằm trong phễu nón an toàn $30^\circ$ ($\arctan(R_{xy} / \Delta z) \le 30^\circ$).
    5. Tính bán kính chuyển mạch dốc trượt thích ứng: $R_{switch} = \text{clamp}(5.0 + 2.0 \times 2\sigma, 4.5, 10.0)\text{ m}$.

### 4. Module Hoàn toàn Mới 2: `SMC Guidance` — Dẫn đường Dốc trượt 45°
* **Phiên bản cũ (`iapf-planner`)**:
  * Chỉ hạ cánh thẳng đứng đơn giản hoặc hạ theo APF ($v_z = -v_{descent}$). Khi drone hạ thẳng từ trên cao, nếu drone nghiêng thân để chỉnh vị trí thì camera gắn cố định/gimbal rất dễ hất marker ra ngoài tầm nhìn.
* **Phiên bản hiện tại (`precision-landing`)**:
  * File tham chiếu: [`smc_guidance.py`](file:///home/duy/VDT_project/embedded_handover/algorithms/smc_guidance.py) / [`SMCGuidance.hpp`](file:///home/duy/VDT_project/embedded_handover/cpp_templates/SMCGuidance.hpp).
  * Bộ điều khiển Trượt 3D (Sliding Mode Control) với **Góc dốc tiếp cận cố định $45^\circ$ ($\theta_{des} = \pi/4$)**: Drone vừa tiến tới vừa hạ độ cao đồng bộ, giúp camera luôn nhìn chéo thấy bãi đáp ở trung tâm tầm nhìn.
  * Luật tiếp cận lũy thừa (Power Reaching Law): $\dot{S}_i = -k_i |S_i|^{3/5} \text{sgn}(S_i)$.
  * Chia 2 phân pha:
    * `GLIDE_SLOPE`: Bám dốc $45^\circ$, vận tốc hạ $v_z \le -0.35\text{m/s}$.
    * `FINAL_DESCENT`: Khi độ cao $z \le 0.40\text{m}$ và sai số ngang $R_{xy} \le 0.40\text{m}$, chuyển sang căn tâm khép kín và hạ êm dịu $v_z = -0.15\text{m/s}$ sát sàn.

### 5. Module Hoàn toàn Mới 3: `Touchdown Detector` — Dò Tiếp đất Phi Trôi dạt
* **Phiên bản cũ (`iapf-planner`)**:
  * Dựa hoàn toàn vào độ cao tuyệt đối $z$ từ GPS/Barometer. Trong thực tế, cảm biến áp suất khí quyển (barometer) bị trôi nhiệt độ và trôi áp suất khí quyển từ $0.5\text{m} \to 1.0\text{m}$ sau vài phút bay, dẫn đến việc drone nghĩ rằng nó đã chạm đất khi còn đang lơ lửng, hoặc đâm sầm xuống đất mà vẫn cố điều khiển cánh quạt.
* **Phiên bản hiện tại (`precision-landing`)**:
  * File tham chiếu: [`touchdown_detector.py`](file:///home/duy/VDT_project/embedded_handover/algorithms/touchdown_detector.py) / [`TouchdownDetector.hpp`](file:///home/duy/VDT_project/embedded_handover/cpp_templates/TouchdownDetector.hpp).
  * **Cơ chế Kinematic Stoppage**: Không phụ thuộc độ cao tuyệt đối. Xác nhận tiếp đất khi:
    1. Lệnh vận tốc đứng đang yêu cầu hạ: $v_{z\_cmd} \le -0.10\text{m/s}$.
    2. Vận tốc đứng thực tế triệt tiêu: $|v_z| \le 0.08\text{m/s}$.
    3. Tốc độ thay đổi độ cao thực tế triệt tiêu: $|dz/dt| \le 0.05\text{m/s}$.
    4. Độ cao nằm trong trần lân cận mặt đất $z \le 0.25\text{m}$.
    5. Điều kiện thỏa mãn liên tục trong **$0.35\text{s}$** $\implies$ Chốt `/landing/touchdown = True` (Latched).

### 6. Cải tiến Bộ điều khiển IBVS (Visual Servoing)
* Thêm chức năng **Gimbal Pitch Sweep** trong pha `SEARCH` để tự động lia camera lên xuống quét tìm bãi đáp dưới mặt đất.
* Thêm chế độ **Nadir Tilt Mode**: Khi pha `LAND` hạ xuống dưới độ cao $z < 0.8\text{m}$, gimbal tự động ép góc pitch thẳng đứng xuống sàn ($-85^\circ$), đảm bảo Inner Marker ID 43 nằm trọn vẹn trong khung hình ở cự ly cách đất vài centimet.
* Thêm vùng chết pixel (**Pixel Trimming Deadband**) khi $z < 0.3\text{m}$ nhằm triệt tiêu hoàn toàn rung giật gimbal do nhiễu pixel cận cảnh.
* Quản lý trạng thái `visual_measurement_active`: Khi mất dấu hình ảnh, drone lập tức phanh dừng góc yaw hiện tại, không quay tròn thân đuổi theo tọa độ ảnh cũ.

### 7. Máy Trạng thái FSM & Cơ chế Ngắt Động Cơ Cưỡng Bức (Force-Disarm Failsafe)
* Chu trình FSM mở rộng thành 7 trạng thái: `IDLE` $\to$ `SEARCH` $\to$ `FOLLOW` $\to$ `APPROACH` $\to$ `GLIDE_SLOPE` $\to$ `LAND` $\to$ `TOUCHDOWN`.
* Tích hợp cơ chế **Wave-off Abort**: Trong lúc đang trượt dốc `GLIDE_SLOPE`, nếu bất ngờ mất dấu camera $> 1.5\text{s}$ hoặc sai số quỹ đạo $h_{err} > 0.6\text{m}$ hoặc $2\sigma > 0.55\text{m}$, drone lập tức hủy hạ cánh và quay về pha `APPROACH`/`FOLLOW` an toàn.
* **Force-Disarm Failsafe**: Khi Touchdown Detector báo chạm sàn, Offboard Commander gửi lệnh MAVLink:
  `MAV_CMD_COMPONENT_ARM_DISARM` với `param1 = 0.0` (Disarm) và `param2 = 21196.0` (Force Disarm bypass pre-flight safety checks). Động cơ dừng quay ngay lập tức, triệt tiêu 100% hiện tượng trượt sàn hoặc lật cánh quạt.

---

## 4. Hướng dẫn Thiết lập Mạng Nội bộ (Mobile Hotspot) & Khắc phục Lỗi Multicast

### ⚠️ Cảnh báo Kỹ thuật Sống còn về Hotspot Điện thoại
Khi dùng điện thoại phát Wi-Fi (Android / iOS Hotspot), hệ điều hành điện thoại **mặc định vô hiệu hóa hoặc chặn toàn bộ gói tin UDP Multicast** để tiết kiệm pin. Trong khi đó, cơ chế tìm kiếm node mặc định của ROS 2 (DDS Simple Discovery) lại dựa hoàn toàn vào Multicast UDP!
> **Hậu quả**: PC trạm và Pi 5 cùng kết nối vào Wi-Fi của điện thoại, `ping` thấy nhau bình thường, nhưng lệnh `ros2 topic list` không nhìn thấy bất kỳ topic nào của nhau!

### Cách giải quyết: Sử dụng CycloneDDS với Cấu hình Unicast Peers
Hệ thống đã chuẩn bị sẵn file cấu hình [`embedded_handover/config/cyclonedds_hotspot.xml`](file:///home/duy/VDT_project/embedded_handover/config/cyclonedds_hotspot.xml).

#### Bước 1: Xem IP của cả 2 thiết bị trên Hotspot
- Bật Hotspot trên điện thoại, kết nối cả Laptop (Host) và Pi 5 vào mạng này.
- Kiểm tra IP trên Host:
  ```bash
  hostname -I
  # Giả sử IP máy Host là: 192.168.43.100
  ```
- Kiểm tra IP trên Pi 5 (hoặc xem trong danh sách thiết bị kết nối trên điện thoại):
  ```bash
  hostname -I
  # Giả sử IP Pi 5 là: 192.168.43.50
  ```

#### Bước 2: Điền IP vào file `cyclonedds_hotspot.xml`
Mở file `embedded_handover/config/cyclonedds_hotspot.xml` và cập nhật đúng 2 IP trên:
```xml
<Peers>
    <Peer address="192.168.43.50"/>   <!-- IP của Raspberry Pi 5 -->
    <Peer address="192.168.43.100"/>  <!-- IP của Máy tính Host GCS -->
    <Peer address="127.0.0.1"/>
</Peers>
```

#### Bước 3: Xuất biến môi trường trên CẢ HAI MÁY (Host và Pi 5)
Thêm vào file `~/.bashrc` của cả Host và Pi 5:
```bash
export ROS_DOMAIN_ID=42
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file:///path/to/embedded_handover/config/cyclonedds_hotspot.xml
```

---

## 5. Hướng dẫn Thiết lập & Vận hành trên Máy tính Trạm (Host PC / GCS)

Trên máy tính trạm, kỹ sư điều hành không cần cài đặt gói mô phỏng nặng nề mà chỉ cần công cụ điều khiển nhẹ trong `host_tools/`.

### 1. Chuẩn bị môi trường trên Host PC
```bash
cd /home/duy/VDT_project/embedded_handover/host_tools
source host_env_setup.sh
```

### 2. Khởi chạy GCS Operator Terminal
Chạy script giao diện điều khiển trạm mặt đất:
```bash
python3 gcs_operator_terminal.py
```
Giao diện dòng lệnh hiển thị bảng thông số thời gian thực từ Pi 5 truyền về:
```
[STATUS] Phase: FOLLOW       | EKF: TRACKING   | Gate: SAFE (OK)    | 2σ: 0.182m | SMC: GLIDE_SLOPE  | Contact: IN AIR
```

### 3. Các thao tác chỉ huy từ Host PC:
* **Kích hoạt hạ cánh**: Nhấn phím `L` + `Enter`. Script sẽ publish `std_msgs/Bool(data=True)` lên topic `/operator/land_command`. Drone lập tức chuyển từ pha `FOLLOW` sang `APPROACH` và hạ cánh tự động khi Covariance Gate mở.
* **Hủy lệnh khẩn cấp (Emergency Hold/Abort)**: Nhấn phím `A` + `Enter`. Script sẽ publish `data=False`. Drone lập tức hủy quá trình trượt dốc, nâng độ cao và quay lại giữ vị trí `FOLLOW`.
* **Thoát GCS**: Nhấn `Q` + `Enter`.

### 4. (Tùy chọn) Bật RViz2 trên Host PC để quan sát trực quan 3D
Nếu muốn xem 3D Odometry và TF Tree từ drone:
```bash
rviz2
# Subscribe vào:
#  - /odom (nav_msgs/Odometry)
#  - /ekf/target_state (nav_msgs/Odometry)
#  - /tf và /tf_static
```

---

## 6. Hướng dẫn Cài đặt & Triển khai C++ trên Raspberry Pi 5

### 1. Cài đặt các thư viện nền tảng trên Raspberry Pi OS / Ubuntu 24.04 (ARM64)
```bash
sudo apt update && sudo apt install -y \
    build-essential \
    cmake \
    git \
    libeigen3-dev \
    libopencv-dev \
    libopencv-contrib-dev \
    ros-humble-ros-base \
    ros-humble-rmw-cyclonedds-cpp \
    ros-humble-vision-msgs \
    ros-humble-nav-msgs \
    ros-humble-sensor-msgs \
    ros-humble-diagnostic-msgs
```

### 2. Cài đặt Driver RealSense D435i (`librealsense2`)
```bash
# Cài đặt từ apt hoặc build từ source với cờ tối ưu ARM
sudo apt install -y librealsense2-dev librealsense2-utils
# Cắm camera vào cổng USB 3.0 (xanh dương) và kiểm tra:
rs-enumerate-devices
```

### 3. Tích hợp các file Header C++ vào ROS 2 C++ Node trên Pi 5
Các template trong `cpp_templates/` là **Header-Only C++20**, hoàn toàn độc lập với ROS:
* Đưa thư mục `cpp_templates/` vào `include/` của ROS 2 C++ package trên Pi 5.
* Ví dụ `CMakeLists.txt` mẫu trên Pi 5:
```cmake
cmake_minimum_required(VERSION 3.16)
project(precision_landing_embedded)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Tối ưu hóa hiệu năng cho CPU Cortex-A76 của Raspberry Pi 5
add_compile_options(-O3 -mcpu=cortex-a76)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(vision_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

add_executable(landing_controller_node src/main_controller_node.cpp)
ament_target_dependencies(landing_controller_node
  rclcpp
  nav_msgs
  geometry_msgs
  std_msgs
  sensor_msgs
  vision_msgs
  diagnostic_msgs
)
target_link_libraries(landing_controller_node ${OpenCV_LIBS})

install(TARGETS
  landing_controller_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
```

---

## 7. Giao thức Giao tiếp Giữa Pi 5 và Pixhawk (PX4)

### Kết nối Vật lý
* Cổng **TELEM2** trên Pixhawk nối với **UART0** (GPIO 14 TX, GPIO 15 RX) hoặc **UART1** trên header 40 chân của Pi 5:
  * Pixhawk TELEM2 TX $\to$ Pi 5 RX (GPIO 15 / Pin 10)
  * Pixhawk TELEM2 RX $\to$ Pi 5 TX (GPIO 14 / Pin 8)
  * Pixhawk GND $\to$ Pi 5 GND (Pin 6)
  * **Lưu ý**: KHÔNG nối chân nguồn 5V của TELEM2 với chân 5V của Pi 5 (cấp nguồn Pi 5 qua BEC độc lập 5V/5A).

### Cấu hình PX4 Parameter trên QGroundControl:
* `UXRCE_DDS_CFG` = `TELEM2` (hoặc `MAV_1_CONFIG` = `TELEM2` nếu dùng MAVLink)
* `SER_TEL2_BAUD` = `921600 8N1`

### Chạy Micro XRCE-DDS Agent trên Pi 5 (Nếu dùng DDS Native):
```bash
MicroXRCE-DDS-Agent serial --dev /dev/ttyAMA0 -b 921600
```
Lúc này, các topic như `/fmu/in/trajectory_setpoint` và `/fmu/out/vehicle_odometry` sẽ xuất hiện trực tiếp trong ROS 2 trên Pi 5.

### Hoặc Sử dụng MAVLink Router / MAVROS (Nếu dùng MAVLink):
```bash
ros2 launch mavros px4.launch fcu_url:=/dev/ttyAMA0:921600
```
Node `offboard_commander` gửi setpoint qua topic `/mavros/setpoint_raw/local` và lệnh Force-Disarm qua service `/mavros/cmd/command`.

---

## 8. Quy trình Vận hành Thực nghiệm An toàn (Pre-flight Checklist)

1. **Chuẩn bị Bãi đáp**: In file `vision/dual_scale_aruco_board_A0_52_5cm.pdf` trên decal hoặc bạt phẳng khổ A0, dán cố định trên mặt đất phẳng.
2. **Kiểm tra Calib Camera**: Chạy `main_aruco_detector.py` trên Pi 5, đưa bãi đáp vào tầm nhìn, đảm bảo tâm trục tọa độ 3D hiển thị chuẩn xác tại tâm bãi đáp ở mọi cự ly từ 4m xuống 10cm.
3. **Kiểm tra Mạng Hotspot**: Chạy `host_env_setup.sh` trên PC trạm, gõ `ros2 node list` trên PC trạm và xác nhận nhìn thấy các node chạy trên Pi 5.
4. **Cất cánh**: Chuyển drone sang chế độ Position Hold, cho drone cất cánh lên độ cao $2.5\text{m} \to 3.0\text{m}$ phía trên bãi đáp.
5. **Kiểm tra Khóa Mục tiêu**: Trên GCS Terminal, quan sát phase chuyển từ `IDLE` $\to$ `SEARCH` $\to$ `FOLLOW`. Drone tự động hướng gimbal và xoay mũi drone về phía bãi đáp.
6. **Ra Lệnh Hạ cánh**: Khi bãi đáp đã được EKF khóa ổn định ($2\sigma < 0.35\text{m}$), nhấn `L` trên GCS Terminal.
7. **Quan sát Tự động**:
   * Drone tiến vào dốc nghiêng $45^\circ$, vừa hạ vừa bám tâm (`GLIDE_SLOPE`).
   * Khi cách sàn $0.4\text{m}$, drone chuyển sang hạ thẳng đứng (`LAND`), gimbal chúi thẳng $-85^\circ$.
   * Chạm mặt bãi đáp: Sau $0.35\text{s}$, Touchdown Detector kích hoạt Force Disarm ngắt động cơ tức thì.
8. **An toàn Khẩn cấp**: Luôn cầm điều khiển RC trên tay; nếu xảy ra bất thường về gió hoặc vật cản, gạt công tắc chuyển Flight Mode trên tay cầm sang **Position** hoặc **Manual/Kill** để giành lại quyền điều khiển bất kỳ lúc nào!
