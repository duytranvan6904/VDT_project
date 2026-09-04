# HƯỚNG DẪN CHI TIẾT TRIỂN KHAI THUẬT TOÁN IBVS VÀ EKF 
## Điều Kiển Gimbal Servo Pitch & Drone Yaw Kết Hợp APF Planner

> **Dự án**: Autonomous Dynamic H-Pad Following, Obstacle Avoidance & Precision Landing  
> **Tác giả**: Duy (Vision/Estimation) & Tuân (Control/Planning)  
> **Phiên bản**: 2.0 (Cập nhật theo yêu cầu thiết kế thực tế)

---

## 1. TỔNG QUAN HỆ THỐNG VÀ LUỒNG DỮ LIỆU

### 1.1. Mục tiêu bài toán
Thuật toán **Image-Based Visual Servoing (IBVS)** kết hợp bộ ước lượng **Extended Kalman Filter (EKF)** có nhiệm vụ:
1. Đón nhận tọa độ 3D $\mathbf{P}_c = [x_c, y_c, z_c]^T$ của tâm ArUco marker từ camera RGB-D.
2. Quy đổi ra tọa độ điểm ảnh $(u, v)$ qua mô hình Pinhole Camera.
3. Khi mục tiêu bị che khuất hoặc mất dấu tạm thời ($< 3-5\text{s}$), EKF sẽ duy trì dự đoán vị trí mục tiêu trong không gian 3D và chiếu ngược ra $(u^{pred}, v^{pred})$ để hệ thống không bị gián đoạn tracking.
4. Điều khiển đồng thời **Gimbal Servo Pitch (gập camera)** bằng xung PWM trực tiếp và **Quadrotor Yaw Angle** ($\psi_{cmd}$) đồng bộ với vector vận tốc né vật cản 3D của bộ **APF Planner**.

### 1.2. Sơ đồ khối kiến trúc phần mềm

```
┌────────────────────────────────────────────────────────────────────────────────────────┐
│                                   LUỒNG DỮ LIỆU IBVS + EKF                             │
│                                                                                        │
│  ┌──────────────────────┐   3D Pose (x_c, y_c, z_c)                                    │
│  │ Vision Node (RGB-D)  ├────────────────────────────┐                                 │
│  └──────────────────────┘                            │                                 │
│                                                      ▼                                 │
│  ┌──────────────────────┐   Drone Pose (World) ┌──────────┐                            │
│  │ Drone Odometry / EKF ├─────────────────────►│ EKF      │                            │
│  └──────────────────────┘                      │ Target   ├── Target State (World)     │
│                                                │ Estimator│     [x, y, z, vx, vy, vz]  │
│                                                └─────┬────┘                            │
│                                                      │                                 │
│                                                      ▼                                 │
│                                                ┌───────────┐                           │
│                                                │ Pinhole   ├── Pixel (u, v, z)         │
│                                                │ Projection│                           │
│                                                └─────┬─────┘                           │
│                                                      │                                 │
│                                                      ▼                                 │
│                                                ┌───────────┐                           │
│                                                │ IBVS      │                           │
│                                                │ Controller│                           │
│                                                └─┬───────┬─┘                           │
│                                                  │       │                             │
│                   Gimbal Pitch Cmd (rad)         │       │ Yaw Setpoint psi_cmd        │
│                                                  ▼       ▼                             │
│                                            ┌──────────┐ ┌───────────────────────────┐ │
│                                            │ PWM      │ │ Combined Offboard Cmd     │ │
│                                            │ Mapping  │ │ (APF Velocity 3D +        │ │
│                                            │ & Filter │ │  IBVS Yaw Setpoint)       │ │
│                                            └────┬─────┘ └─────────────┬─────────────┘ │
│                                                 │                     │               │
│                                                 ▼                     ▼               │
│                                           Gimbal Servo PWM       PX4 TrajectorySetpoint│
└────────────────────────────────────────────────────────────────────────────────────────┘
```

---

## 2. TRIỂN KHAI CHI TIẾT CÁC MODULE THUẬT TOÁN

### 2.1. Module 1: Chuyển Đổi Tọa Độ & Mô Hình Pinhole Camera

Camera độ sâu Intel RealSense D435i trích xuất tọa độ 3D của tâm ArUco trong hệ tọa độ Camera $\mathbf{P}_c = [x_c, y_c, z_c]^T$.

#### Công thức chiếu phối cảnh (Pinhole Model):
$$u = \lambda_x \frac{x_c}{z_c} + u_0$$
$$v = \lambda_y \frac{y_c}{z_c} + v_0$$

- **Thông số cấu hình D435i chuẩn ($640 \times 480$)**:
  - $u_0 = 320 \text{ px}$ (Tâm ảnh ngang)
  - $v_0 = 240 \text{ px}$ (Tâm ảnh dọc)
  - $\lambda_x = \lambda_y \approx 380 \text{ px}$ (Tiêu cự pixel)

#### Sai số điểm ảnh (Image Error):
Goal là đưa mục tiêu về tâm ảnh $(u_0, v_0)$:
$$e_u = u - u_0 = \lambda_x \frac{x_c}{z_c}$$
$$e_v = v - v_0 = \lambda_y \frac{y_c}{z_c}$$

---

### 2.2. Module 2: Bộ Ước Lượng Mục Tiêu EKF Khi Che Khuất (Occlusion Handling)

Để giải quyết bài toán mục tiêu bị che khuất bởi vật cản hoặc bị khuất khỏi FOV camera trong thời gian ngắn ($< 3-5\text{s}$), ta xây dựng bộ lọc **Extended Kalman Filter 6 trạng thái** làm việc trong hệ tọa độ Thế giới (World Frame - ENU/NED).

#### 1. Véctơ trạng thái & Mô hình động học (Constant Velocity - CV):
$$\mathbf{x}_w = [x_w, y_w, z_w, v_{wx}, v_{wy}, v_{wz}]^T$$

Phương trình trạng thái:
$$\mathbf{x}_k = \mathbf{F} \mathbf{x}_{k-1} + \mathbf{w}_k$$
$$\mathbf{F} = \begin{bmatrix} \mathbf{I}_3 & T_s \mathbf{I}_3 \\ \mathbf{0}_3 & \mathbf{I}_3 \end{bmatrix}, \quad \mathbf{Q} = \text{diag}(\sigma_x^2, \sigma_y^2, \sigma_z^2, \sigma_{vx}^2, \sigma_{vy}^2, \sigma_{vz}^2)$$

#### 2. Biến đổi Tọa độ Camera $\leftrightarrow$ World:
Chuyển tọa độ đo từ Camera sang World qua Ma trận biến đổi $\mathbf{T}_{cam}^{world}$ (dựa trên odometry và góc nghiêng camera):
$$\mathbf{P}_w^{meas} = \mathbf{R}_{drone} \cdot \mathbf{R}_{gimbal}(\theta_g) \cdot \mathbf{P}_c + \mathbf{p}_{drone}$$

#### 3. Thuật toán Xử lý Theo Trạng Thái Detection:

- **TRƯỜNG HỢP A: Target Visible (`is_detected == True`)**:
  - **Predict**:
    $$\mathbf{x}_k^- = \mathbf{F} \mathbf{x}_{k-1}$$
    $$\mathbf{P}_k^- = \mathbf{F} \mathbf{P}_{k-1} \mathbf{F}^T + \mathbf{Q}$$
  - **Update**:
    $$\mathbf{K}_k = \mathbf{P}_k^- \mathbf{H}^T (\mathbf{H} \mathbf{P}_k^- \mathbf{H}^T + \mathbf{R})^{-1}, \quad \mathbf{H} = [\mathbf{I}_3 \quad \mathbf{0}_3]$$
    $$\mathbf{x}_k = \mathbf{x}_k^- + \mathbf{K}_k (\mathbf{P}_w^{meas} - \mathbf{H} \mathbf{x}_k^-)$$
    $$\mathbf{P}_k = (\mathbf{I} - \mathbf{K}_k \mathbf{H}) \mathbf{P}_k^-$$

- **TRƯỜNG HỢP B: Target Occluded / Lost (`is_detected == False`)**:
  - **Chỉ thực hiện Predict**:
    $$\mathbf{x}_k = \mathbf{x}_k^- = \mathbf{F} \mathbf{x}_{k-1}$$
    $$\mathbf{P}_k = \mathbf{P}_k^- = \mathbf{F} \mathbf{P}_{k-1} \mathbf{F}^T + \mathbf{Q}$$
  - **Dự đoán tọa độ Camera & Pixel**:
    $$\mathbf{P}_c^{pred} = \mathbf{R}_{gimbal}^{-1}(\theta_g) \cdot \mathbf{R}_{drone}^{-1} \cdot (\mathbf{x}_k[1:3] - \mathbf{p}_{drone})$$
    $$u^{pred} = \lambda_x \frac{x_c^{pred}}{z_c^{pred}} + u_0, \quad v^{pred} = \lambda_y \frac{y_c^{pred}}{z_c^{pred}} + v_0$$
    $\rightarrow$ Cấp tọa độ $[u^{pred}, v^{pred}, z_c^{pred}]$ này cho bộ điều khiển IBVS duy trì góc ngắm!

---

### 2.3. Module 3: Thuật Toán IBVS Cho Gimbal Pitch & Drone Yaw

Ma trận tương tác rút gọn (Interaction Matrix) cho góc Pitch camera và góc Yaw drone:

#### 1. Phương trình vi phân ảnh:
$$\dot{u} = -\left( \lambda_x + \frac{u^2}{\lambda_x} \right) \omega_{pitch} + v \cdot \omega_{yaw} + \text{Term}(v_C)$$
$$\dot{v} = -\frac{u \cdot v}{\lambda_x} \omega_{pitch} - u \cdot \omega_{yaw} + \text{Term}(v_C)$$

Trong đó $\text{Term}(v_C)$ là thành phần vận tốc tịnh tiến do drone di chuyển (do APF planner quy định). Thành phần này được bù bằng feedforward hoặc được EKF trơn hóa.

#### 2. Luật điều khiển góc đặt (Proportional Control Law):

- **Điều khiển Gimbal Pitch Angle ($\theta_g$)**:
  $$\theta_{gimbal}^{ref}(k) = \theta_{gimbal}^{ref}(k-1) + K_{p, pitch} \cdot \left( \frac{v - v_0}{\lambda_y} \right) \cdot T_s$$
  *Định hướng*: Khi $v > v_0$ (target ở nửa dưới ảnh), gimbal cần pitch xuống (giảm góc $\theta_g$).

- **Điều khiển Drone Yaw Angle ($\psi_{drone}$)**:
  $$\psi_{drone}^{ref}(k) = \psi_{drone}^{current} + \arctan2(x_c, z_c)$$
  Hoặc sử dụng luật phản hồi IBVS pixel:
  $$\psi_{drone}^{ref}(k) = \psi_{drone}^{ref}(k-1) + K_{p, yaw} \cdot \left( \frac{u - u_0}{\lambda_x} \right) \cdot T_s$$

---

### 2.4. Module 4: Điều Khiển Gimbal Servo Pitch Bằng PWM Trực Tiếp

Không cần bộ điều khiển dòng điện/vận tốc 3 tầng phức tạp của động cơ brushless 2 trục. Động cơ Servo MG90S nhận tín hiệu xung PWM góc trực tiếp từ Pi 5 GPIO hoặc mạch điều khiển Servo.

#### 1. Công thức ánh xạ từ Góc đặt sang Xung PWM:
Với Servo MG90S chuẩn:
- Góc $\theta_g = 0^\circ$ (nhìn ngang) $\leftrightarrow \text{PWM} = 1500 \ \mu s$
- Góc $\theta_g = -90^\circ$ (gập thẳng xuống) $\leftrightarrow \text{PWM} = 1000 \ \mu s$ (hoặc $2000 \ \mu s$ tùy hướng lắp)

$$\text{PWM}(\theta_g) = 1500 + \left( \frac{\theta_g}{\pi/2} \right) \times 500 \quad (\mu s)$$

#### 2. Bộ lọc thông thấp (EMA) & Gioi hạn Tốc độ (Rate Limiter):
Để tránh rung giật cơ khí cho servo và biến dạng hình ảnh camera:
1. **Rate Limiter**: Giới hạn tốc độ thay đổi góc $\left| \frac{\Delta \theta_g}{\Delta t} \right| \le 180^\circ/\text{s}$.
2. **Bộ lọc EMA (Exponential Moving Average)**:
   $$\text{PWM}_{filtered}(k) = \alpha \cdot \text{PWM}_{raw}(k) + (1 - \alpha) \cdot \text{PWM}_{filtered}(k-1)$$
   với $\alpha = 0.2 \sim 0.3$.

---

### 2.5. Module 5: Thiết Lập 4 Pha Tracking (State Machine Mapping)

Cấu hình luật điều khiển theo 4 pha hoạt động của bài toán:

| Pha | Điều kiện chuyển pha | Luật Gimbal Pitch ($\theta_g$) | Luật Drone Yaw ($\psi$) | Tương tác APF |
|---|---|---|---|---|
| **SEARCH** | Chờ detect $\ge 10$ frames | Cố định $\theta_g = 0^\circ$ (nhìn ngang) | Quay Yaw chậm $\dot{\psi} = 15^\circ/\text{s}$ | APF giữ vị trí hover |
| **FOLLOW** | Target detected ổn định | Dynamic IBVS Pitch ($0^\circ \to -30^\circ$) | Dynamic IBVS Yaw ($\psi_{cmd}$) | APF né vật cản, duy trì $d_{follow} \approx 3\text{m}$ |
| **APPROACH** | Operator "LAND" ON | IBVS Pitch gập sâu dần ($-30^\circ \to -60^\circ$) | IBVS Yaw căn thẳng tâm $u \to u_0$ | APF giảm gain đẩy 50%, tiến về H-Pad |
| **LAND** | Alignment $e < 30\text{cm}, z < 0.5\text{m}$ | Lock Gimbal Pitch $-90^\circ$ (thẳng xuống) | Lock Drone Yaw, IBVS fine alignment | APF OFF, bay thẳng đứng xuống |

---

### 2.6. Module 6: Tích Hợp IBVS Với APF Planner Gửi PX4 Offboard

Để kết hợp mượt mà giữa **IBVS (hướng đầu drone)** và **APF Planner (hướng di chuyển tịnh tiến 3D)**:

#### 1. Dữ liệu gửi đến PX4:
Chạy ở Offboard Mode qua ROS 2 topic `px4_msgs/msg/TrajectorySetpoint`.

```cpp
// Cấu trúc gói tin gửi PX4 micro-XRCE-DDS
px4_msgs::msg::TrajectorySetpoint setpoint_msg;

// Vector vận tốc 3D từ APF Planner (World Frame / Local NED)
setpoint_msg.velocity[0] = v_apf_x; // North
setpoint_msg.velocity[1] = v_apf_y; // East
setpoint_msg.velocity[2] = v_apf_z; // Down

// Góc Yaw từ IBVS Controller
setpoint_msg.yaw = psi_ibvs_cmd;    // Heading Angle (rad)
setpoint_msg.yawspeed = yaw_rate_cmd; // (Tùy chọn feedforward)
```

#### 2. Điểm ưu việt của phương án:
- **Tách biệt nhiệm vụ**: APF lo điều khiển vị trí/vận tốc tịnh tiến $[v_x, v_y, v_z]$ để né vật cản; IBVS lo xoay góc Yaw $\psi$ để giữ camera luôn hướng về mục tiêu.
- **Tính nhất quán**: PX4 Autopilot hỗ trợ hoàn hảo việc nhận đồng thời `velocity` 3D và `yaw` setpoint trong cùng một message `TrajectorySetpoint`.

---

## 3. HƯỚNG DẪN MÔ PHỎNG VÀ KIỂM NGHIỆM TRÊN MATLAB SIMULINK

### 3.1. Cấu trúc các file mã nguồn

Dự án sẽ tạo các file tại thư mục `sim_data/` và `simulink/`:

```
e:\VDT_project\VDT_project\
├── sim_data/
│   └── generate_target_sim_data.m    # Script sinh dữ liệu mô phỏng tọa độ mục tiêu 3D
├── simulink/
│   ├── ibvs_config.m                 # Cấu hình thông số camera, gain, giới hạn servo
│   ├── ekf_target_estimator.m        # MATLAB function khối EKF ước lượng & dự đoán
│   ├── ibvs_gimbal_yaw_controller.m  # MATLAB function tính luật điều khiển IBVS & PWM
│   ├── gimbal_drone_ibvs_sim.slx     # Mô hình Simulink kiểm nghiệm toàn bộ hệ thống
│   └── run_ibvs_verification.m       # Script chạy tự động & vẽ đồ thị đánh giá KPI
└── docs/
    └── IBVS_Implementation_Guide.md  # File hướng dẫn chi tiết này
```

---

### 3.2. Script sinh dữ liệu mô phỏng (`generate_target_sim_data.m`)

Mô phỏng 3 kịch bản:
1. Target di chuyển tuyến tính + ziczac tốc độ $3-5 \text{ m/s}$.
2. Drone bị dao động nhiễu thái độ Roll/Pitch/Yaw $\pm 5^\circ$.
3. **Kịch bản Occlusion**: Tắt tín hiệu `is_detected = false` trong khoảng thời gian $t = 10\text{s} \to 13\text{s}$ để test bộ EKF Predict.

---

### 3.3. Quy trình chạy kiểm nghiệm Simulink

1. Mở MATLAB và `cd` đến thư mục `e:\VDT_project\VDT_project\simulink`.
2. Chạy script `run_ibvs_verification.m`.
3. Script sẽ tự động:
   - Nạp thông số từ `ibvs_config.m`.
   - Sinh dữ liệu test bằng `generate_target_sim_data.m`.
   - Simula mô hình `gimbal_drone_ibvs_sim.slx`.
   - Hiển thị đồ thị đáp ứng: Pixel error $(e_u, e_v)$, Góc Gimbal Pitch $\theta_g$, Tín hiệu Xung PWM, Góc Drone Yaw $\psi$, và Khoảng thời gian EKF Predict khi che khuất.

---

## 4. CHUẨN ĐÓNG GÓI CHO THÀNH VIÊN APF PLANNER (TUÂN)

Để chuyển giao cho Tuân (chạy node APF Planner), module IBVS sẽ được đóng gói thành một C++ / Python Class hoặc ROS 2 Node độc lập với giao diện API rõ ràng:

### Struct Giao Diện Header (`ibvs_interface.hpp`)

```cpp
#ifndef IBVS_INTERFACE_HPP_
#define IBVS_INTERFACE_HPP_

struct CameraTargetMeasurement {
    bool is_detected;       // True nếu ArUco detect thành công
    double x_c, y_c, z_c;   // Tọa độ 3D tâm ArUco trong Camera Frame (m)
    double timestamp;       // Thời gian nhận dữ liệu
};

struct DroneState {
    double roll, pitch, yaw; // Thái độ drone (rad)
    double pos_x, pos_y, pos_z; // Tọa độ drone trong World Frame (m)
};

struct IBVSOutput {
    double gimbal_pitch_cmd_rad; // Tín hiệu góc Pitch đặt cho Gimbal (rad)
    uint16_t gimbal_pitch_pwm;   // Tín hiệu PWM điều khiển Servo MG90S (1000 - 2000 us)
    double drone_yaw_cmd_rad;    // Tín hiệu góc Yaw đặt cho Quadrotor (rad)
    bool is_aligned;             // True nếu sai số pixel < 30px & alt < 0.5m (dùng cho FSM)
    bool is_predicting;          // True nếu EKF đang chạy chế độ Predict do bị che khuất
};

class IBVSGimbalYawController {
public:
    IBVSGimbalYawController();
    void init(double focal_length, double u0, double v0);
    IBVSOutput update(const CameraTargetMeasurement& target, const DroneState& drone);
};

#endif // IBVS_INTERFACE_HPP_
```

---

## 5. TIÊU CHÍ ĐÁNH GIÁ VÀ KPI NGHIỆM THU

1. **Tracking Accuracy**:
   - Sai số góc nhìn camera luôn giữ mục tiêu trong FOV ($e_u < 40\text{px}, e_v < 40\text{px}$) khi target di chuyển $\le 5\text{m/s}$.
2. **Khả năng chống che khuất (Occlusion Robustness)**:
   - Duy trì ước lượng vị trí và góc nhìn mượt mà trong thời gian mất tín hiệu target $\le 3\text{s}$.
3. **Chất lượng điều khiển Servo PWM**:
   - Tín hiệu PWM không bị rung giật dải tần cao, tốc độ biến đổi góc $\le 180^\circ/\text{s}$.
4. **Thời gian tính toán**:
   - Thời gian thực thi 1 vòng lặp IBVS + EKF $< 2\text{ms}$ trên C++ / Python (đáp ứng tốt tần số $30-50\text{Hz}$ trên Pi 5).

---
*Tài liệu này là căn cứ kỹ thuật chính thức để tiến hành lập mã nguồn mô phỏng Simulink và đóng gói ROS 2 Node.*
