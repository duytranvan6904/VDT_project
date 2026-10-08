# 🛬 Kế hoạch tích hợp Precision Landing — Covariance-Aware SMC Guidance

> **Dự án**: VDT Quadrotor Autonomous Landing on ArUco H-Pad  
> **Ngày lập**: 05/10/2026  
> **Tác giả**: Duy (Vision/Estimation Lead)  
> **Cơ sở thuật toán**: Terminal-Angle-Constrained Sliding Mode Guidance (Tuân)  
> **Scope**: Tích hợp Covariance từ cả Target EKF + PX4 EKF2 vào pipeline Landing + Touchdown Detection

---

## Mục lục

1. [Tổng quan vấn đề & Mục tiêu](#1-tổng-quan-vấn-đề--mục-tiêu)
2. [Kiến trúc tổng thể hệ thống Landing](#2-kiến-trúc-tổng-thể-hệ-thống-landing)
3. [Mô hình sai số tương đối kép (Dual-EKF Covariance)](#3-mô-hình-sai-số-tương-đối-kép-dual-ekf-covariance)
4. [Tích hợp Covariance vào SMC Guidance](#4-tích-hợp-covariance-vào-smc-guidance)
5. [Máy trạng thái Landing 5 pha](#5-máy-trạng-thái-landing-5-pha)
6. [Touchdown Detection & Motor Shutdown](#6-touchdown-detection--motor-shutdown)
7. [Giao diện ROS 2 Topics & Messages](#7-giao-diện-ros-2-topics--messages)
8. [Mapping Module → File code](#8-mapping-module--file-code)
9. [Roadmap triển khai theo Sprint](#9-roadmap-triển-khai-theo-sprint)
10. [Phụ lục — Tham số khuyến nghị](#10-phụ-lục--tham-số-khuyến-nghị)

---

## 1. Tổng quan vấn đề & Mục tiêu

### 1.1. Bản chất vấn đề sai số kép

Hệ thống drone sử dụng **hai bộ EKF độc lập** để ước lượng vị trí:

| Đối tượng | Bộ lọc | State Vector | Nguồn đo | Covariance Output |
|---|---|---|---|---|
| **Drone** | PX4 EKF2 (firmware) | `[x,y,z, vx,vy,vz, ...]` 24 states | IMU + GPS + Baro + Mag | `eph`, `epv` trong `VehicleLocalPosition` |
| **H-Pad** | TargetStateEKF (code mình) | `[x,y,z, vx,vy,vz]` 6 states | ArUco PnP qua camera | Ma trận `P` 6×6 trong `/ekf/target_state` |

Khi thuật toán landing tính sai lệch vị trí `Δr = r̂_pad − r̂_drone`, sai số thực tế là **tổ hợp** của cả hai nguồn ước lượng:

```
                    ┌──────────────────────────────────────────────┐
                    │    Vị trí thực drone    Vị trí thực H-Pad    │
                    │         ●                      ●             │
                    │        ╱                        ╲            │
                    │   σ_drone                   σ_target         │
                    │      ╱                          ╲           │
                    │  x̂_drone ─── Δr ước lượng ─── x̂_target     │
                    │                                              │
                    │  σ_relative = √(σ²_drone + σ²_target)       │
                    └──────────────────────────────────────────────┘
```

> [!CAUTION]
> Nếu `σ_relative > bán kính bãi đáp / 2`, drone có thể "nghĩ" mình đã align nhưng thực tế nằm ngoài H-Pad!

### 1.2. Mục tiêu thiết kế

| # | Mục tiêu | Tiêu chí đánh giá |
|---|---|---|
| 1 | Hạ cánh mềm trên H-Pad **đứng yên** | Sai lệch tiếp đất < 10cm, vận tốc chạm < 0.3 m/s |
| 2 | Camera **luôn giữ marker** trong FOV suốt quá trình landing | Góc tiếp cận 45° đảm bảo ArUco luôn nằm trong khung hình |
| 3 | **Không cho phép landing** khi covariance quá lớn | Tự động hold/abort nếu σ_relative > ngưỡng an toàn |
| 4 | Phát hiện chạm đất **đáng tin cậy** với phần cứng hiện có | Kết hợp ≥2 tín hiệu: altitude + gia tốc + marker size |
| 5 | Tương thích firmware PX4 hiện tại | Sử dụng Offboard velocity mode, LAND/Disarm qua MAVLink |

---

## 2. Kiến trúc tổng thể hệ thống Landing

### 2.1. Sơ đồ kiến trúc module

```mermaid
flowchart TD
    subgraph SENSORS["🔧 SENSORS & ESTIMATION"]
        CAM["📷 RealSense D435<br/>Camera + Depth"]
        IMU["⚡ PX4 IMU/GPS/Baro"]
        ARUCO["ArUco Detector<br/>DICT_6X6_50 #42"]
        EKF_T["Target EKF<br/>6-state CV model"]
        EKF_D["PX4 EKF2<br/>24-state fusion"]
    end

    subgraph COVARIANCE["📐 COVARIANCE FUSION"]
        COV_EXTRACT["Covariance Extractor<br/>P_target + P_drone"]
        COV_GATE["Covariance Gate<br/>σ_relative < threshold?"]
    end

    subgraph GUIDANCE["🧭 SMC GUIDANCE (từ Tuân)"]
        LOS["LOS Geometry<br/>Rxy, ψ, Rz"]
        SURFACE["Sliding Surface<br/>S = [S1, S2, S3]"]
        GUIDE["GuidanceLaw<br/>SMC + Power Reaching"]
        INTEGRATORS["3× Integrator<br/>Vp, αp, γp"]
        VEL["VelComponents<br/>→ NED velocity"]
    end

    subgraph FSM["🎯 LANDING STATE MACHINE"]
        APPROACH["APPROACH<br/>IAPF + Visual Servo"]
        GLIDE["GLIDE_SLOPE<br/>SMC 45° descent"]
        FINAL["FINAL_DESCENT<br/>Vertical slow down"]
        TOUCH["TOUCHDOWN<br/>Contact detect"]
        DISARM_STATE["DISARM<br/>Motor shutdown"]
    end

    subgraph CONTROL["⚙️ CONTROL & ACTUATION"]
        SWITCH["Phase Switch<br/>APF ↔ Landing"]
        OFFBOARD["Offboard Commander<br/>PX4 velocity setpoint"]
        SERVO["Servo Gimbal<br/>Camera pitch"]
    end

    CAM --> ARUCO --> EKF_T
    IMU --> EKF_D
    EKF_T -->|P_target 6×6| COV_EXTRACT
    EKF_D -->|eph, epv| COV_EXTRACT
    COV_EXTRACT --> COV_GATE

    EKF_T -->|x̂_target| LOS
    EKF_D -->|x̂_drone, v_drone| LOS

    LOS --> SURFACE --> GUIDE
    LOS -->|Rxy| GUIDE
    EKF_D -->|Vp, αp, γp| GUIDE

    GUIDE --> INTEGRATORS --> VEL

    COV_GATE -->|safe_to_land| FSM
    VEL -->|V_cmd Landing| SWITCH
    APPROACH -->|V_cmd APF| SWITCH

    APPROACH --> GLIDE
    GLIDE --> FINAL
    FINAL --> TOUCH
    TOUCH --> DISARM_STATE

    SWITCH --> OFFBOARD --> IMU
    FSM --> SERVO
```

### 2.2. Phần cứng hệ thống hiện tại

| Thành phần | Model | Vai trò trong Landing |
|---|---|---|
| Flight Controller | Pixhawk 6C (PX4 v1.14+) | EKF2 fusion, Offboard mode, Arm/Disarm |
| Camera | Intel RealSense D435 | ArUco detection + Depth sensing |
| Companion Computer | Jetson Nano / NUC | ROS 2 nodes, EKF, Guidance |
| Servo Gimbal | 2-axis (pitch + yaw) | Camera pointing control (IBVS) |
| ArUco Marker | **A0 landscape dual-scale board**<br/>• ID 42 coarse: 52cm, đặt lệch trái (dải xa)<br/>• ID 43 fine: 5cm, đặt đúng tâm pad (dải gần) | Hai tag không che mã của nhau; PnP quy đổi cả hai về tâm pad |
| H-Pad | 118.9cm × 84.1cm (A0 landscape) | Landing surface; ID 43 trùng tâm bãi đáp |
| Frame | Holybro X500 Depth | Quadrotor platform |

---

## 3. Mô hình toán học sai số tương đối và cơ chế Covariance Gating

### 3.1. Mô hình không gian trạng thái và hiệp phương sai ước lượng bãi đáp ($P_{target\_pos}$)

Trạng thái động học của bãi đáp được mô tả bởi vector trạng thái 6 chiều:
$$\mathbf{x}_T = [\mathbf{p}_T^T, \mathbf{v}_T^T]^T \in \mathbb{R}^6, \quad \text{với } \mathbf{p}_T = [x_T, y_T, z_T]^T, \; \mathbf{v}_T = [\dot{x}_T, \dot{y}_T, \dot{z}_T]^T$$

Bộ lọc Kalman mở rộng (EKF) duy trì ma trận hiệp phương sai sai số trạng thái $\mathbf{P}_T \in \mathbb{R}^{6 \times 6}$:
$$\mathbf{P}_T(t) = \mathbb{E}\left[(\mathbf{x}_T(t) - \hat{\mathbf{x}}_T(t))(\mathbf{x}_T(t) - \hat{\mathbf{x}}_T(t))^T\right]$$

Chu trình đệ quy EKF:
- **Pha dự báo (Time Update):**
  $$\hat{\mathbf{x}}_T^-(k) = \mathbf{F} \hat{\mathbf{x}}_T(k-1), \quad \mathbf{P}_T^-(k) = \mathbf{F} \mathbf{P}_T(k-1) \mathbf{F}^T + \mathbf{Q}(\Delta t)$$
- **Pha cập nhật đo lường (Measurement Update):**
  $$\mathbf{K}_k = \mathbf{P}_T^-(k) \mathbf{H}^T \left(\mathbf{H} \mathbf{P}_T^-(k) \mathbf{H}^T + \mathbf{R}_k\right)^{-1}$$
  $$\mathbf{P}_T(k) = (\mathbf{I} - \mathbf{K}_k \mathbf{H}) \mathbf{P}_T^-(k) (\mathbf{I} - \mathbf{K}_k \mathbf{H})^T + \mathbf{K}_k \mathbf{R}_k \mathbf{K}_k^T$$

Khối con hiệp phương sai vị trí 3D $\mathbf{P}_{target\_pos} \in \mathbb{R}^{3 \times 3}$ được trích xuất trực tiếp:
$$\mathbf{P}_{target\_pos} = \mathbf{P}_T[0:3, 0:3] = \begin{bmatrix} \sigma_{tx}^2 & \sigma_{txy} & \sigma_{txz} \\ \sigma_{txy} & \sigma_{ty}^2 & \sigma_{tyz} \\ \sigma_{txz} & \sigma_{tyz} & \sigma_{tz}^2 \end{bmatrix}$$

- **Giao diện dữ liệu:** Trích xuất qua ROS 2 topic `/ekf/target_state` (`nav_msgs/msg/Odometry`), đọc trường `msg.pose.covariance[0..35]`:
  $$\sigma_{tx}^2 = \text{cov}[0], \quad \sigma_{txy} = \text{cov}[1], \quad \sigma_{ty}^2 = \text{cov}[7], \quad \sigma_{tz}^2 = \text{cov}[14]$$

### 3.2. Mô hình hiệp phương sai ước lượng trạng thái Drone ($\mathbf{P}_{drone\_pos}$)

Vị trí drone $\mathbf{p}_D = [x_D, y_D, z_D]^T$ trong hệ quy chiếu cục bộ NED được ước lượng bởi PX4 EKF2 (24-state estimation). Sai số vị trí 1-sigma được PX4 cung cấp qua topic `/fmu/out/vehicle_local_position`:
- $eph = \sigma_h = \sqrt{\sigma_N^2 + \sigma_E^2}$: Độ lệch chuẩn vị trí theo phương ngang (Horizontal Position Error Metric, đơn vị: mét).
- $epv = \sigma_v = \sigma_D$: Độ lệch chuẩn vị trí theo phương thẳng đứng (Vertical Position Error Metric, đơn vị: mét).

Mô hình ma trận hiệp phương sai vị trí drone được thiết lập theo giả định phân bố đẳng hướng trên mặt phẳng ngang (Isotropic horizontal error model):
$$\mathbf{P}_{drone\_pos} = \begin{bmatrix} eph^2 & 0 & 0 \\ 0 & eph^2 & 0 \\ 0 & 0 & epv^2 \end{bmatrix} \in \mathbb{R}^{3 \times 3}$$

- **Đặc tính động học:** $eph$ và $epv$ biến thiên thời gian phụ thuộc vào chất lượng thu tín hiệu GNSS/RTK, độ rung cảm biến IMU và động lực học bay:
  - Điều kiện RTK Fix / Visual Odometry: $eph \in [0.03, 0.10]\text{ m}$.
  - Điều kiện GNSS tiêu chuẩn ngoài trời: $eph \in [0.30, 0.80]\text{ m}$.
  - Môi trường GNSS suy biến / Multipath: $eph \in [1.00, 2.50]\text{ m}$.

### 3.3. Mô hình hiệp phương sai tương đối toàn cục (Global Relative Covariance)

Vector sai lệch vị trí tương đối giữa tâm bãi đáp và trọng tâm drone:
$$\Delta \mathbf{r} = \hat{\mathbf{p}}_T - \hat{\mathbf{p}}_D = [\Delta x, \Delta y, \Delta z]^T$$

Do bộ lọc Target EKF và bộ lọc PX4 EKF2 hoạt động độc lập (hai quá trình ngẫu nhiên không tương quan, $\mathbb{E}[(\mathbf{x}_T - \hat{\mathbf{x}}_T)(\mathbf{x}_D - \hat{\mathbf{x}}_D)^T] = \mathbf{0}$):
$$\mathbf{P}_{relative} = \mathbf{P}_{target\_pos} + \mathbf{P}_{drone\_pos} \in \mathbb{R}^{3 \times 3}$$

$$\mathbf{P}_{relative} = \begin{bmatrix} 
\sigma_{tx}^2 + eph^2 & \sigma_{txy} & \sigma_{txz} \\ 
\sigma_{txy} & \sigma_{ty}^2 + eph^2 & \sigma_{tyz} \\ 
\sigma_{txz} & \sigma_{tyz} & \sigma_{tz}^2 + epv^2 
\end{bmatrix}$$

Chiếu ma trận hiệp phương sai lên mặt phẳng ngang tiếp xúc $(x, y)$:
$$\mathbf{P}_{xy} = \mathbf{P}_{relative}[0:2, 0:2] = \begin{bmatrix} \sigma_{tx}^2 + eph^2 & \sigma_{txy} \\ \sigma_{txy} & \sigma_{ty}^2 + eph^2 \end{bmatrix}$$

Phân rã phổ (Spectral Decomposition): $\mathbf{P}_{xy} = \mathbf{V} \mathbf{\Lambda} \mathbf{V}^T$ với $\mathbf{\Lambda} = \text{diag}(\lambda_1, \lambda_2)$. Giá trị riêng cực đại:
$$\lambda_{max} = \max(\lambda_1, \lambda_2) = \frac{\text{tr}(\mathbf{P}_{xy}) + \sqrt{(\text{tr}(\mathbf{P}_{xy}))^2 - 4\det(\mathbf{P}_{xy})}}{2}$$

- **Độ lệch chuẩn theo phương bất lợi nhất (Worst-Case Directional Uncertainty):**
  $$\sigma_{relative\_xy} = \sqrt{\lambda_{max}(\mathbf{P}_{xy})}$$
- **Bán kính tin cậy 3-sigma ($99.7\%$ Confidence Ellipse):**
  $$R_{conf\_3\sigma} = 3 \cdot \sigma_{relative\_xy} = 3 \sqrt{\lambda_{max}(\mathbf{P}_{xy})}$$

### 3.4. Chuẩn an toàn hạ cánh: Cơ chế Covariance Gating

Để đảm bảo toàn bộ diện tích tiếp xúc của khung chân drone (Landing Gear Footprint) nằm trọn trong mặt phẳng bãi đáp khi tiếp đất:

1. **Ràng buộc biên hình học (Geometric Boundary):**
   Gọi $R_{pad}$ là bán kính hữu ích của bãi đáp ($R_{pad} = D_{pad}/2$), $r_{gear}$ là bán kính giới hạn của khung chân tiếp xúc drone, và $\mathbf{e}_{track} = \|\Delta \hat{\mathbf{r}}_{xy}\|$ là sai số bám điều khiển.
   Khoảng cách cực đại từ tâm bãi đáp đến điểm xa nhất của chân tiếp xúc với độ tin cậy $99.7\%$ là:
   $$r_{max\_contact} = \|\Delta \hat{\mathbf{r}}_{xy}\| + R_{conf\_3\sigma} + r_{gear}$$

2. **Điều kiện an toàn hạ cánh (Covariance Gate Formulation):**
   Hệ thống cho phép tiếp tục hạ cánh khi và chỉ khi:
   $$r_{max\_contact} \le R_{pad} \iff \|\Delta \hat{\mathbf{r}}_{xy}\| + 3\sqrt{\lambda_{max}(\mathbf{P}_{xy})} \le R_{pad} - r_{gear}$$

3. **Chuẩn hóa hệ số an toàn cho thiết kế:**
   Với cấu hình drone kích thước $r_{gear} \approx R_{pad}/2$, không gian dung sai cho phép co về:
   $$\boxed{\|\Delta \hat{\mathbf{r}}_{xy}\| + 3\sqrt{\lambda_{max}(\mathbf{P}_{xy})} \le \frac{R_{pad}}{2}}$$
   - **Quy tắc quyết định:**
     $$\text{Landing\_Authorization} = \begin{cases} 
     \text{TRUE (Tiến hành hạ cánh)} & \text{khi } \|\Delta \hat{\mathbf{r}}_{xy}\| + 3\sqrt{\lambda_{max}(\mathbf{P}_{xy})} \le \frac{R_{pad}}{2} \\
     \text{FALSE (Duy trì Hover / Abort)} & \text{khi } \|\Delta \hat{\mathbf{r}}_{xy}\| + 3\sqrt{\lambda_{max}(\mathbf{P}_{xy})} > \frac{R_{pad}}{2}
     \end{cases}$$

### 3.5. Chuyển đổi hệ quy chiếu cục bộ Marker-Relative (Triệt tiêu sai số GPS)

Trong điều kiện bay ngoài trời sử dụng GPS dân sự thông thường, sai số vị trí toàn cầu $eph \approx 0.5 - 1.5\text{ m} > R_{pad}$, khiến điều kiện kiểm tra toàn cầu không thể thỏa mãn. Để giải quyết triệt để vấn đề này, hệ thống áp dụng cơ chế chuyển hệ quy chiếu **Marker-Relative**:

1. **Thiết lập hệ tọa độ gốc bãi đáp (Marker Frame):**
   Gán tâm bãi đáp làm gốc quy chiếu cục bộ $\mathcal{F}_{pad}$: $\mathbf{p}_{origin} \equiv [0, 0, 0]^T$.

2. **Bản chất đo lường quang học PnP và vai trò của thông tin độ cao Drone:**
   Khi camera giải bài toán PnP trên ArUco marker, vector tịnh tiến thu được $\mathbf{t}_c = [x_c, y_c, z_c]^T$ nằm trong hệ quang học (`camera_optical_frame`). Đối với thị giác đơn sắc (monocular camera), cự ly dọc trục quang học $z_c$ có phương sai tăng theo bình phương khoảng cách ($\sigma_{z_c} \propto z_c^2$) và dễ bị nhiễu do góc nhìn nghiêng (perspective ambiguity). Do đó, $\mathbf{t}_c$ đơn lẻ không phản ánh khoảng cách 3D thực tế với độ tin cậy tuyệt đối ở cự ly xa.
   
   Thay vào đó, vector hướng tia nhìn (Line-of-Sight Bearing Vector) $\mathbf{u}_c = \frac{\mathbf{t}_c}{\|\mathbf{t}_c\|}$ mang độ chính xác góc rất cao. Để xác định chính xác khoảng cách 3D thực tế, hệ thống kết hợp **thông tin độ cao tương đối thực tế của Drone** $h = z_{drone} - z_{pad}$ (được ước lượng tin cậy từ cảm biến đo cao Lidar/Barometer hoặc trạng thái độ cao của bộ lọc EKF PX4):
   $$\mathbf{u}_n = \mathbf{R}_b^n \cdot \mathbf{R}_g \cdot \mathbf{R}_c^{cam} \cdot \mathbf{u}_c = \begin{bmatrix} u_{nx} \\ u_{ny} \\ u_{nz} \end{bmatrix}$$
   Khoảng cách thực tế dọc theo tia nhìn được xác định bởi giao điểm của tia nhìn với mặt phẳng bãi đáp tại độ cao $h$:
   $$\rho = \frac{h}{|u_{nz}|}$$
   Vector vị trí tương đối từ trọng tâm Drone đến Marker trong hệ tọa độ cục bộ (NED/ENU):
   $$\mathbf{r}_{rel} = \rho \cdot \mathbf{u}_n + \mathbf{R}_b^n \mathbf{p}_{cam}^b = \frac{h}{|u_{nz}|} \mathbf{u}_n + \mathbf{R}_b^n \mathbf{p}_{cam}^b = \begin{bmatrix} \Delta x_{MR} \\ \Delta y_{MR} \\ \Delta z_{MR} \end{bmatrix} \quad (18)$$
   trong đó $\mathbf{p}_{cam}^b$ là vector cánh tay đòn (lever-arm offset) từ trọng tâm Drone tới quang tâm Camera.

3. **Độ tin cậy và Lan truyền Hiệp phương sai có tính đến sai số bộ lọc EKF PX4:**
   Khác với hệ tọa độ toàn cầu:
   - Cơ chế Marker-Relative **triệt tiêu hoàn toàn sai số trôi dạt toạ độ ngang toàn cầu của GPS ($eph$)**, vì vị trí tương đối không phụ thuộc vào kinh độ/vĩ độ toàn cầu của Drone.
   - Tuy nhiên, do công thức (18) sử dụng góc xoay tư thế thân $\mathbf{R}_b^n(\phi, \theta)$ và độ cao $h$ từ PX4, **độ tin cậy của vị trí tương đối bắt buộc phải tính cả sai số của bộ lọc EKF2 PX4** (bao gồm hiệp phương sai góc nghiêng $\mathbf{P}_{att}^{EKF} = \text{diag}(\sigma_\phi^2, \sigma_\theta^2)$ và phương sai độ cao $\sigma_{h, EKF}^2 = epv^2$).

   Áp dụng định luật lan truyền sai số Gauss bậc một (First-Order Error Propagation):
   $$\mathbf{P}_{relative}^{MR} = \mathbf{J}_{vision} \mathbf{R}_{vision} \mathbf{J}_{vision}^T + \mathbf{J}_{att} \mathbf{P}_{att}^{EKF} \mathbf{J}_{att}^T + \mathbf{J}_{alt} \sigma_{h, EKF}^2 \mathbf{J}_{alt}^T \quad (19)$$
   trong đó:
   - $\mathbf{J}_{vision} = \frac{\partial \mathbf{r}_{rel}}{\partial \mathbf{t}_c}$: Ma trận Jacobian theo vector đo thị giác.
   - $\mathbf{J}_{att} = \frac{\partial \mathbf{r}_{rel}}{\partial [\phi, \theta]^T}$: Ma trận Jacobian theo góc nghiêng drone từ EKF (sai số góc $\sigma_\theta \approx 0.5^\circ \approx 0.0087\text{ rad}$, gây độ lệch ngang $h \cdot \sigma_\theta \approx 3\text{m} \times 0.0087 \approx 0.026\text{ m}$).
   - $\mathbf{J}_{alt} = \frac{\partial \mathbf{r}_{rel}}{\partial h}$: Ma trận Jacobian theo độ cao EKF (sai số $epv \approx 0.05 - 0.10\text{ m}$).

   Nhờ triệt tiêu hoàn toàn sai số $eph$ của GPS ($0.5 - 1.5\text{ m}$), tổng độ lệch chuẩn tương đối co về:
   $$\sigma_{relative, xy}^{MR} \approx \sqrt{\sigma_{vision, xy}^2 + (h \cdot \sigma_\theta)^2} \approx \sqrt{0.03^2 + 0.026^2} \approx 0.04\text{ m} \ll R_{pad}$$
   Điều này đảm bảo điều kiện an toàn hạ cánh (Covariance Gate) được thỏa mãn một cách chặt chẽ và nhất quán về mặt toán học.

### 3.6. Cấu hình Nested Dual-Scale ArUco Board và Tính liên tục của PnP

Để mở rộng dải độ cao hoạt động từ $5.0\text{ m}$ xuống tới $0.05\text{ m}$ mà không gây suy biến nghiệm PnP do hiện tượng cắt biên ảnh (Edge Clipping), bãi đáp được thiết kế dạng **Nested ArUco Board**:

1. **Mô hình hình học bãi đáp:**
   - **Outer Marker:** ID 42, kích thước $L_{outer} = 20\text{ cm}$, vùng hoạt động $z \in [0.8\text{m}, 5.0\text{m}]$.
   - **Inner Marker:** ID 43, kích thước $L_{inner} = 4\text{ cm}$, đồng tâm với ID 42, vùng hoạt động $z \in [0.05\text{m}, 0.8\text{m}]$.

2. **Mô hình toán học ArUco Board:**
   Tập hợp 8 đỉnh 3D của hai marker được định nghĩa trên cùng mặt phẳng $Z=0$ với gốc tọa độ đặt tại tâm bãi đáp:
   $$\mathcal{M} = \{\mathbf{P}_i^{(outer)}\}_{i=1}^4 \cup \{\mathbf{P}_j^{(inner)}\}_{j=1}^4, \quad \mathbf{P} \in \mathbb{R}^3$$

3. **Hàm tối ưu hóa đa điểm (Multi-point PnP Optimization):**
   Nghiệm tư thế $(\mathbf{R}^*, \mathbf{t}^*)$ được xác định qua bài toán phi tuyến:
   $$(\mathbf{R}^*, \mathbf{t}^*) = \arg\min_{\mathbf{R}, \mathbf{t}} \sum_{k \in \mathcal{V}} \left\| \mathbf{u}_k - \pi(\mathbf{K}, \mathbf{R} \mathbf{P}_k + \mathbf{t}) \right\|^2$$
   Trong đó:
   - $\mathcal{V}$ là tập hợp các điểm góc quan sát được trong khung hình ảnh.
   - $\mathbf{u}_k \in \mathbb{R}^2$ là tọa độ pixel của góc $k$.
   - $\pi(\cdot)$ là hàm chiếu phối cảnh ma trận nội suy camera $\mathbf{K}$.

4. **Tính chất liên tục và chống nhảy bước (Continuous Transition):**
   Do cả hai marker cùng quy chiếu về gốc tọa độ tâm $(0,0,0)$, vector nghiệm $\mathbf{t}^*(z)$ đạt tính chất liên tục $C^0$ trên toàn bộ dải độ cao chuyển tiếp $z \in [0.4\text{m}, 0.8\text{m}]$. Hệ thống không xuất hiện hiện tượng gián đoạn tín hiệu điều khiển khi chuyển vùng quan sát giữa marker lớn và marker nhỏ.

---

## 4. Tích hợp Covariance vào SMC Guidance

### 4.1. Sửa đổi thuật toán Tuân cho hệ thống ROS 2

Thuật toán gốc của Tuân trong Simulink sử dụng vị trí pad lý tưởng (`posHpad = [-25, -25, 0]`). Khi tích hợp vào ROS 2 thực tế, cần thay đổi:

```mermaid
flowchart TD
    subgraph ORIGINAL["Simulink gốc (Tuân)"]
        O1["posHpad = const"]
        O2["posUAV = mô hình đlh"]
        O3["Không kiểm tra covariance"]
        O1 --> O4["Guidance tính trực tiếp"]
        O2 --> O4
    end

    subgraph MODIFIED["ROS 2 thực tế (tích hợp)"]
        M1["posHpad = EKF target ± P_target"]
        M2["posUAV = EKF2 drone ± P_drone"]
        M3["Covariance Gate"]
        M1 --> M3
        M2 --> M3
        M3 -->|"OK"| M4["Guidance tính với<br/>marker-relative Δr"]
        M3 -->|"NOT OK"| M5["HOLD hover<br/>chờ reacquire"]
    end
```

### 4.2. Covariance-Aware Rswitch (Adaptive Phase Transition)

Trong code gốc, `Rswitch = 15m` là hằng số. Với covariance awareness:

```python
def compute_adaptive_Rswitch(self, sigma_relative: float) -> float:
    """Adaptive switching distance based on current estimation quality.
    
    Idea: Chỉ bắt đầu SMC landing khi đã đủ gần VÀ ước lượng đủ tốt.
    Nếu covariance lớn, thu hẹp Rswitch để buộc drone phải tiến gần hơn
    (→ marker lớn hơn trong FOV → PnP chính xác hơn → P giảm).
    """
    R_max = 15.0   # Rswitch tối đa khi covariance rất tốt (m)
    R_min = 3.0    # Rswitch tối thiểu (m), ít nhất cần khoảng cách này
    
    # Ngưỡng covariance "lý tưởng" để dùng Rswitch tối đa
    sigma_ideal = 0.05   # σ < 5cm → tin cậy cao
    # Ngưỡng covariance "tệ" để thu hẹp Rswitch tối thiểu
    sigma_bad = 0.30     # σ > 30cm → ước lượng chưa tốt
    
    if sigma_relative <= sigma_ideal:
        return R_max
    elif sigma_relative >= sigma_bad:
        return R_min
    else:
        # Nội suy tuyến tính
        ratio = (sigma_relative - sigma_ideal) / (sigma_bad - sigma_ideal)
        return R_max - ratio * (R_max - R_min)
```

### 4.3. Covariance-Aware Sliding Surface Weight

Khi covariance cao, các hệ số mặt trượt nên "mềm" hơn (tiếp cận chậm hơn, an toàn hơn):

```python
def adaptive_gains(self, sigma_relative: float):
    """Scale SMC gains based on estimation confidence.
    
    Khi σ nhỏ → tự tin → gains lớn → tiếp cận nhanh
    Khi σ lớn → không chắc chắn → gains nhỏ → tiếp cận thận trọng
    """
    confidence = max(0.0, 1.0 - sigma_relative / 0.20)  # [0,1]
    confidence = np.clip(confidence, 0.3, 1.0)           # Không giảm quá 30%
    
    ka = 0.2 * confidence    # Tốc độ khép Rxy
    kb = 0.6 * confidence    # Tốc độ áp góc dốc
    kc = 0.4 * confidence    # Tốc độ căn chỉnh phương vị
    
    return ka, kb, kc
```

### 4.4. Sơ đồ luồng dữ liệu Covariance trong Guidance Loop

```mermaid
sequenceDiagram
    participant CAM as Camera ArUco
    participant EKF_T as Target EKF
    participant EKF_D as PX4 EKF2
    participant COV as Covariance Gate
    participant GUIDE as SMC Guidance
    participant CTRL as Offboard Cmd

    loop Mỗi 20ms (50Hz)
        CAM->>EKF_T: PnP position measurement
        EKF_T->>EKF_T: predict() + update()
        EKF_T->>COV: x̂_target, P_target

        EKF_D->>COV: x̂_drone, eph, epv

        COV->>COV: P_rel = P_target + P_drone
        COV->>COV: σ_rel = √(max_eig(P_rel[:2,:2]))

        alt σ_rel < threshold AND validGuidance
            COV->>GUIDE: Δr = x̂_target − x̂_drone, σ_rel
            GUIDE->>GUIDE: LOS → Sliding Surface → GuidanceLaw
            GUIDE->>GUIDE: Integrators → VelComponents
            GUIDE->>CTRL: V_cmd Landing (NED)
        else σ_rel ≥ threshold
            COV->>CTRL: V_cmd = [0,0,0] (HOLD hover)
            Note over CTRL: Chờ marker reacquire<br/>hoặc GPS stabilize
        end

        CTRL->>EKF_D: TrajectorySetpoint → PX4
    end
```

---

## 5. Máy trạng thái Landing 5 pha

### 5.1. Tổng quan FSM

Mở rộng FSM hiện tại từ `IDLE → SEARCH → FOLLOW → APPROACH → LAND` thành hệ thống landing chi tiết hơn trong pha APPROACH → LAND:

```mermaid
stateDiagram-v2
    [*] --> IDLE
    IDLE --> SEARCH: Takeoff complete
    SEARCH --> FOLLOW: Target acquired + EKF converged
    FOLLOW --> APPROACH: Operator LAND command

    state APPROACH {
        [*] --> APF_APPROACH
        APF_APPROACH --> GLIDE_SLOPE: Rxy ≤ Rswitch AND<br/>covGate OK AND<br/>validGuidance
        GLIDE_SLOPE --> FINAL_DESCENT: Rxy ≤ 1.0m AND<br/>altitude ≤ 1.5m
        FINAL_DESCENT --> TOUCHDOWN: touchdownDetected
        TOUCHDOWN --> DISARMED: confirmCount ≥ N
    }

    APPROACH --> FOLLOW: Marker lost > timeout<br/>OR Operator cancel
    GLIDE_SLOPE --> APF_APPROACH: covGate FAIL > 3s<br/>(revert to APF)

    note right of GLIDE_SLOPE
        SMC Guidance active
        θ_des = 45° glide slope
        Covariance monitored
    end note

    note right of FINAL_DESCENT
        Vertical descent
        V_z = -0.2 m/s
        Lock XY position
    end note

    note right of TOUCHDOWN
        Motor ramp-down
        → PX4 LAND mode
        → Disarm
    end note
```

#### Tracking invariants carried into the landing pipeline

- APF publishes obstacle/goal guidance on `/apf/guidance_velocity_cmd` and target-motion compensation on `/apf/target_velocity_ff`. The FSM reduces only the guidance component during yaw alignment and caps the combined horizontal command at `follow_speed_limit`.
- Target-motion compensation remains enabled at the standoff goal while estimated target speed is at least `0.20 m/s`; otherwise the drone would stop at the goal, fall behind, and repeatedly accelerate back toward the marker.
- FOLLOW yaw uses pixel feedback plus a bounded line-of-sight bearing lead. Both IBVS and Offboard allow up to `0.80 rad/s`; lead activates above `0.30 m/s` and goes to zero for a stationary target.
- SEARCH holds its yaw when a detection arrives. FOLLOW is entered only after a bbox newer than that trigger, accepted EKF `TRACKING`, an in-frame marker, and settled vehicle yaw remain valid for the confirmation interval. A pre-trigger bbox cannot confirm the lock.
- FOLLOW translation accepts only fresh bbox/detection input. Its vertical velocity is always zero; Offboard altitude hold is the only FOLLOW altitude controller. APF's `0.30 m` goal deadband suppresses small stand-off corrections caused by stationary-target estimate noise.

### 5.2. Chi tiết từng pha

#### Pha 1: APF_APPROACH (IAPF tiến gần H-Pad)

**Mục đích**: Sử dụng IAPF Planner hiện có để đưa drone tiến gần H-Pad từ vị trí Follow (3.5m follow distance).

**Điều kiện vào**: Operator bấm LAND command.

**Hành vi**:
- IAPF Planner tạo velocity command tiến về phía target
- IBVS giữ camera hướng vào marker (servo gimbal pitch)
- Target EKF chạy liên tục, P_target giảm khi liên tục nhìn thấy marker
- **Covariance Gate chạy ngầm** để kiểm tra readiness cho SMC

**Điều kiện chuyển → GLIDE_SLOPE**:
```python
can_enter_glide = (
    Rxy <= Rswitch_adaptive              # Đủ gần (3–15m tùy σ)
    and sigma_relative < sigma_max_glide  # σ < 0.10m (adjustable)
    and valid_guidance                     # SMC có nghiệm hợp lệ
    and Vp_actual > 0.1                   # Drone đang bay (không hover chết)
    and abs(cos(gamma_actual)) > 0.1      # Không cắm đầu
    and marker_visible                     # ArUco đang detect
)
```

#### Pha 2: GLIDE_SLOPE (SMC Descent theo góc 45°)

**Mục đích**: Hạ cánh theo quỹ đạo cong mượt, giữ marker trong FOV camera.

**Hành vi**:
- LandingModeManager chốt initial state từ trạng thái thực tại thời điểm chuyển pha
- GuidanceLaw tính `[dVp, dα, dγ]` mỗi cycle
- 3 Integrators tạo reference velocity
- VelComponents đổi sang NED → gửi Offboard Commander
- **Gimbal servo**: chuyển sang chế độ nhìn xuống chéo (pitch tùy theo γ_p)

**Covariance monitoring trong pha này**:
```python
# Mỗi cycle kiểm tra:
if sigma_relative > sigma_abort_glide:  # Ví dụ: σ > 0.15m
    abort_timer += dt
    if abort_timer > 3.0:  # Cho phép 3s mất marker trước khi abort
        revert_to_APF_APPROACH()
        abort_timer = 0
else:
    abort_timer = 0  # Reset nếu σ trở lại OK
```

**Adaptive gains**: `ka, kb, kc` điều chỉnh theo σ_relative realtime (xem mục 4.3).

**Điều kiện chuyển → FINAL_DESCENT**:
```python
can_enter_final = (
    Rxy <= 1.0               # Sát bên trên pad (ngang)
    and abs(Rz) <= 1.5       # Độ cao tương đối < 1.5m
    and sigma_relative < 0.06 # σ < 6cm (rất tin cậy ở gần)
)
```

#### Pha 3: FINAL_DESCENT (Hạ thẳng đứng chậm)

**Mục đích**: Từ ~1.5m trên pad, hạ thẳng đứng với vận tốc rất chậm.

**Hành vi**:
- **Lock vị trí XY** trên đỉnh pad (PID position hold hoặc marker-relative XY correction)
- **Vận tốc hạ**: `Vz = -0.2 m/s` (giảm từ từ về -0.15 m/s khi Rz < 0.5m)
- Camera gimbal nhìn thẳng xuống (pitch = -90°) để theo dõi marker ở gần
- **Theo dõi marker size**: khi marker chiếm >50% FOV → chuyển dùng inner marker (nếu có)

**Covariance gate**:
```python
if sigma_relative > 0.10:  # Mất marker hoặc noise
    Vz = 0.0  # DỪNG hạ, hover tại chỗ
    # Chờ marker reacquire
```

**Điều kiện chuyển → TOUCHDOWN**:
```python
touchdown_detected = touchdown_detector.check(
    altitude_agl=altitude_agl,
    accel_z=accel_z_body,
    marker_size_px=marker_size_px,
    Rxy=Rxy, Rz=Rz,
    vel_relative=vel_relative
)
```

#### Pha 4: TOUCHDOWN (Phát hiện chạm đất)

**Chi tiết cơ chế**: Xem [Mục 6](#6-touchdown-detection--motor-shutdown).

#### Pha 5: DISARMED (Ngắt động cơ)

**Hành vi**:
1. Gửi lệnh `MAV_CMD_NAV_LAND` (hoặc `MAV_CMD_COMPONENT_ARM_DISARM` param1=0)
2. Chờ PX4 xác nhận `VehicleStatus.arming_state == DISARMED`
3. Chuyển FSM về IDLE
4. Log kết quả landing (sai lệch cuối cùng, vận tốc chạm, thời gian landing)

---

## 6. Touchdown Detection & Motor Shutdown

### 6.1. Thách thức phát hiện chạm đất

| Thách thức | Giải thích |
|---|---|
| Không có landing gear sensor | Drone X500 không có contact switch |
| Barometer drift | Áp suất thay đổi theo gió cánh quạt (ground effect) |
| GPS altitude noise | ±1–3m, vô dụng cho touchdown detection |
| PnP breakdown ở gần | **Đã giải quyết bằng Inner Marker (ID 43)**: Cho phép PnP tin cậy xuống tới 5cm |
| Nảy pad (bounce) | Drone có thể chạm rồi nảy lên, cần confirm đa tầng |

### 6.2. Chiến lược Touchdown Detection 3 tầng

```mermaid
flowchart TD
    subgraph L1["Tầng 1: Kinematics Check"]
        ALT{"altitude_agl<br/>< 0.15m?"}
        VEL{"‖v_relative‖<br/>< 0.3 m/s?"}
        ALT -->|Yes| VEL
    end

    subgraph L2["Tầng 2: Inertial Check"]
        ACC{"accel_z_body<br/>∈ [8.5, 11.5]<br/>m/s²?"}
        JERK{"Δaccel spike<br/>detected?"}
        VEL -->|Yes| ACC
        ACC -->|Yes| JERK
    end

    subgraph L3["Tầng 3: Confirmation"]
        COUNT{"confirmCount<br/>≥ 10 cycles<br/>(0.5s)?"}
        JERK -->|Yes or Skip| COUNT
    end

    COUNT -->|Yes| LAND["✅ TOUCHDOWN<br/>CONFIRMED"]
    COUNT -->|"Reset nếu<br/>altitude tăng"| ALT

    ALT -->|No| CONTINUE["Tiếp tục descent"]
    VEL -->|No| CONTINUE
    ACC -->|No| CONTINUE
```

### 6.3. Chi tiết từng tầng detection

#### Tầng 1 — Kinematics (Vị trí & Vận tốc)

```python
class TouchdownDetector:
    def __init__(self):
        self.confirm_count = 0
        self.confirm_threshold = 10     # 10 cycles × 50ms = 0.5s
        self.alt_threshold = 0.15       # m (AGL)
        self.vel_threshold = 0.3        # m/s (relative)
        self.accel_z_min = 8.5          # m/s² (≈ 1g - margin)
        self.accel_z_max = 11.5         # m/s² (≈ 1g + margin)
        self.jerk_threshold = 5.0       # m/s³ (spike khi chạm)
        self.prev_accel_z = 9.81
        
    def check(self, altitude_agl, vel_relative, accel_z_body, dt):
        """Multi-layer touchdown detection.
        
        Args:
            altitude_agl: Altitude above ground level (m), from:
                          - Depth camera pointing down, OR
                          - Rz from target EKF (posUAV_z - posHpad_z)
            vel_relative: 3D velocity relative to pad (m/s)
            accel_z_body: Body-frame Z acceleration (m/s²)
                          from /fmu/out/vehicle_acceleration
            dt: Time step (s)
        """
        speed = np.linalg.norm(vel_relative)
```

**Nguồn `altitude_agl`** (theo thứ tự ưu tiên):
1. **RealSense Depth camera** nhìn xuống: đo trực tiếp khoảng cách tới mặt đất, rất chính xác ở <2m
2. **Rz từ SMC** (posUAV_z − posHpad_z): tin cậy khi marker còn visible
3. **PX4 distance sensor** (nếu có rangefinder): `VehicleLocalPosition.dist_bottom`

#### Tầng 2 — Inertial (Gia tốc body)

Khi drone chạm mặt pad, gia tốc body-frame thay đổi đặc trưng:

```
            Đang bay (hover)          Vừa chạm pad
            ──────────────          ──────────────
  accel_z   ≈ 9.81 m/s²            spike → ≈ 12-15 m/s²
            (bù trọng lực           (phản lực mặt đất
             bằng lực đẩy)            cộng thêm)
                                    
            Sau đó ổn định → ≈ 9.81 m/s² (trọng lực thuần túy,
                                           motor tắt/giảm)
```

```python
        # Tầng 2: Inertial check
        jerk = abs(accel_z_body - self.prev_accel_z) / dt
        self.prev_accel_z = accel_z_body
        
        accel_near_1g = (self.accel_z_min <= accel_z_body <= self.accel_z_max)
        jerk_spike = jerk > self.jerk_threshold
```

**Cách lấy `accel_z_body` từ PX4:**
```python
# Topic: /fmu/out/vehicle_acceleration (px4_msgs/VehicleAcceleration)
# Hoặc: /fmu/out/sensor_combined (SensorCombined.accelerometer_m_s2[2])
def accel_cb(self, msg):
    # msg.xyz[2] là accel theo trục Z body (hướng xuống ở NED)
    self.accel_z_body = msg.xyz[2]  # m/s²
```

#### Tầng 3 — Temporal Confirmation (Chống nảy)

```python
        # Tầng 3: Confirmation with hysteresis
        kinematics_ok = (altitude_agl < self.alt_threshold 
                         and speed < self.vel_threshold)
        inertial_ok = accel_near_1g  # or jerk_spike cho detect nhanh hơn
        
        if kinematics_ok and inertial_ok:
            self.confirm_count += 1
        elif altitude_agl > self.alt_threshold * 1.5:
            # Nếu nảy lên → reset hoàn toàn
            self.confirm_count = 0
        else:
            # Giảm dần (không reset ngay) để chống flicker
            self.confirm_count = max(0, self.confirm_count - 1)
        
        return self.confirm_count >= self.confirm_threshold
```

### 6.4. Quy trình Motor Shutdown

Sau khi Touchdown được xác nhận, cần **dừng motor an toàn** theo đúng quy trình PX4:

```mermaid
sequenceDiagram
    participant TD as TouchdownDetector
    participant FSM as Landing FSM
    participant OBC as Offboard Commander
    participant PX4 as PX4 Autopilot

    TD->>FSM: touchdown_confirmed = True
    FSM->>FSM: Phase → TOUCHDOWN

    Note over FSM: Giảm thrust 2s

    FSM->>OBC: velocity_setpoint = [0, 0, 0]

    alt Phương án A: PX4 LAND mode
        OBC->>PX4: MAV_CMD_DO_SET_MODE<br/>main=AUTO, sub=LAND
        PX4->>PX4: Internal landing controller<br/>→ motor ramp-down → disarm
    else Phương án B: Direct Disarm
        OBC->>PX4: MAV_CMD_COMPONENT_ARM_DISARM<br/>param1=0 (disarm)
        PX4->>PX4: Motor cut → Disarmed
    end

    PX4->>FSM: VehicleStatus.arming_state = DISARMED
    FSM->>FSM: Phase → IDLE
    Note over FSM: Log landing result
```

> [!WARNING]
> **Khuyến nghị dùng Phương án A (PX4 LAND mode)** thay vì Direct Disarm. Lý do:
> - PX4 LAND mode có motor ramp-down nội bộ (giảm thrust từ từ)
> - Direct Disarm cắt motor đột ngột → drone rơi từ 10–15cm → có thể nảy hoặc lật
> - LAND mode cũng handle được trường hợp drone chưa thực sự chạm đất

**Lệnh MAVLink cụ thể:**

```python
# Phương án A: Chuyển sang LAND mode
def switch_to_land_mode(self):
    """Switch PX4 to AUTO.LAND mode for safe motor shutdown."""
    if HAS_PX4_MSGS:
        cmd = VehicleCommand()
        cmd.command = 176  # MAV_CMD_DO_SET_MODE
        cmd.param1 = 1.0   # MAV_MODE_FLAG_CUSTOM_MODE_ENABLED
        cmd.param2 = 4.0   # PX4_CUSTOM_MAIN_MODE_AUTO
        cmd.param3 = 6.0   # PX4_CUSTOM_SUB_MODE_AUTO_LAND
        cmd.target_system = 1
        cmd.target_component = 1
        cmd.source_system = 1
        cmd.source_component = 1
        cmd.from_external = True
        self.command_pub.publish(cmd)
    elif self.mav_conn:
        self.mav_conn.set_mode_apm(216)  # AUTO.LAND
```

### 6.5. Gimbal Servo trong quá trình Landing

| Pha Landing | Gimbal Pitch | Lý do |
|---|---|---|
| APF_APPROACH | IBVS theo pixel + hình học EKF; slew pitch tối đa 0.5 rad/s | Giữ marker trong FOV trong lúc APF tiến ngang |
| GLIDE_SLOPE | Tiếp tục IBVS khi marker đang detect | Gimbal không nhảy góc theo lệnh chuyển pha; mất ảnh thì giữ pitch hiện tại |
| FINAL_DESCENT | Slew dần tới −90° với giới hạn 0.5 rad/s, chỉ khi có marker mới | Nhìn xuống gần pad mà không tạo bước gập đột ngột |
| TOUCHDOWN | −90° (lock) | Không di chuyển gimbal khi chạm |

Trong `APPROACH` và `GLIDE_SLOPE`, IBVS chỉ dùng EKF feedforward khi vừa có detection ảnh mới vừa có tracking mode `TRACKING`. Khi ảnh stale/mất, controller giữ pitch và yaw ở trạng thái hiện tại; EKF prediction không kéo camera khỏi vị trí nhìn cuối. `FINAL_DESCENT` mới cho phép đặt pitch về −90°, qua rate limit và lọc EMA.

`/landing/uncertainty_radius` là confidence radius theo `confidence_sigma` (mặc định 2σ). Vì các ngưỡng trong phần landing được biểu diễn theo σ, với cấu hình 2σ phải dùng radius `0.20 m` để vào GLIDE (`σ ≤ 0.10 m`), `0.30 m` để tiếp tục GLIDE (`σ ≤ 0.15 m`) và `0.12 m` để vào FINAL (`σ ≤ 0.06 m`).

---

## 7. Giao diện ROS 2 Topics & Messages

### 7.1. Topics mới cần tạo

| Topic | Message Type | Publisher | Subscriber | Tần suất |
|---|---|---|---|---|
| `/landing/phase` | `std_msgs/String` | Landing FSM | Offboard Cmd, Gimbal | 20 Hz |
| `/landing/covariance_status` | `custom_msgs/CovarianceStatus` | Covariance Gate | Landing FSM, Logger | 10 Hz |
| `/landing/velocity_cmd` | `geometry_msgs/Twist` | SMC Guidance | Phase Switch → Offboard | 50 Hz |
| `/landing/touchdown` | `std_msgs/Bool` | Touchdown Detector | Landing FSM | 20 Hz |
| `/fmu/out/vehicle_acceleration` | `px4_msgs/VehicleAcceleration` | PX4 | Touchdown Detector | 50 Hz |
| `/fmu/out/vehicle_local_position` | `px4_msgs/VehicleLocalPosition` | PX4 | Covariance Extractor | 50 Hz |

### 7.2. Dữ liệu trong Covariance Status message

```python
# Proposal: custom message hoặc dùng DiagnosticStatus
# Nội dung publish mỗi cycle:
covariance_status = {
    'sigma_target_xy': float,       # σ ngang target EKF (m)
    'sigma_target_z': float,        # σ đứng target EKF (m)
    'sigma_drone_xy': float,        # eph từ PX4 (m)
    'sigma_drone_z': float,         # epv từ PX4 (m)
    'sigma_relative_xy': float,     # σ tổng hợp ngang (m)
    'sigma_relative_z': float,      # σ tổng hợp đứng (m)
    'confidence_radius_3sigma': float,  # Bán kính tin cậy 99.7% (m)
    'safe_to_land': bool,           # True nếu confidence_radius + |Δr| < pad_radius
    'Rswitch_adaptive': float,      # Ngưỡng chuyển pha adaptive (m)
}
```

### 7.3. Sơ đồ Topics tổng thể

```mermaid
flowchart LR
    subgraph PX4["PX4 Firmware"]
        VLP["/fmu/out/<br/>vehicle_local_position"]
        VA["/fmu/out/<br/>vehicle_acceleration"]
        TS["/fmu/in/<br/>trajectory_setpoint"]
        VC["/fmu/in/<br/>vehicle_command"]
    end

    subgraph PERCEPTION["Perception Layer"]
        ARUCO_NODE["aruco_sim_node"]
        EKF_ADAPTER["ekf_ros_adapter"]
    end

    subgraph LANDING["Landing Layer (MỚI)"]
        COV_GATE["covariance_gate_node"]
        SMC_GUIDE["smc_guidance_node"]
        LAND_FSM["landing_fsm_node"]
        TD_DETECT["touchdown_detector_node"]
    end

    subgraph CONTROL["Control Layer"]
        MISSION_FSM["mission_fsm_node"]
        OFFBOARD["offboard_commander"]
    end

    ARUCO_NODE -->|"/hpad/position_camera"| EKF_ADAPTER
    EKF_ADAPTER -->|"/ekf/target_state<br/>(pos + cov)"| COV_GATE
    EKF_ADAPTER -->|"/ekf/target_state"| SMC_GUIDE
    VLP -->|"eph, epv"| COV_GATE
    VLP -->|"pos, vel"| SMC_GUIDE

    COV_GATE -->|"/landing/covariance_status"| LAND_FSM
    COV_GATE -->|"/landing/covariance_status"| SMC_GUIDE
    SMC_GUIDE -->|"/landing/velocity_cmd"| LAND_FSM

    VA -->|"accel_z"| TD_DETECT
    VLP -->|"dist_bottom"| TD_DETECT
    EKF_ADAPTER -->|"Rxy, Rz"| TD_DETECT
    TD_DETECT -->|"/landing/touchdown"| LAND_FSM

    LAND_FSM -->|"/landing/phase"| MISSION_FSM
    MISSION_FSM -->|"/mission/velocity_setpoint"| OFFBOARD
    OFFBOARD -->|TrajectorySetpoint| TS
    OFFBOARD -->|VehicleCommand| VC
```

---

### 8.1. Các file đã điều chỉnh và tối ưu hóa

| File | Chức năng điều chỉnh | Trạng thái |
|---|---|---|
| [mission_fsm_node.py](file:///home/duy/VDT_project/simulation/core/mission_fsm_node.py) | Quản lý chuyển pha GLIDE_SLOPE, LAND, TOUCHDOWN; xử lý wave-off abort trên không và khóa wave-off khi đã tiếp đất | **Hoàn tất** |
| [offboard_commander.py](file:///home/duy/VDT_project/simulation/core/offboard_commander.py) | Gửi MAVLink Force-Disarm (`param2=21196.0`), failsafe ngắt động cơ khi tiếp đất $z \le 0.12\text{m}$ duy trì $\ge 0.8\text{s}$ | **Hoàn tất** |
| [ekf_ros_adapter.py](file:///home/duy/VDT_project/simulation/perception/ekf_ros_adapter.py) | Điều chỉnh `min_marker_distance=0.05m`, publish covariance ma trận đầy đủ $P_T$ | **Hoàn tất** |
| [ibvs_controller.py](file:///home/duy/VDT_project/simulation/control/ibvs_controller.py) | Duy trì gimbal pitch trong LAND, tự động nadir tilt ($-85^\circ$) ở cự ly gần ($z < 0.8\text{m}$), nâng slew limit lên $1.8\text{ rad/s}$ | **Hoàn tất** |
| [mission_params.yaml](file:///home/duy/VDT_project/simulation/config/mission_params.yaml) | Căn chỉnh tham số glide slope, conical gate $30^\circ$, adaptive $R_{switch} \in [4.5\text{m}, 10.0\text{m}]$, soft landing | **Hoàn tất** |

### 8.2. Các module mới đã triển khai thành công

| File mới | Vị trí thực tế | Chức năng | Trạng thái |
|---|---|---|---|
| [covariance_gate.py](file:///home/duy/VDT_project/simulation/control/covariance_gate.py) | `simulation/control/` | Thu thập $P_{target} + P_{drone}$, tính bán kính tin cậy $2\sigma$, kiểm tra nón chấp nhận $30^\circ$, điều biến thích ứng $R_{switch}$ | **Hoàn tất** |
| [smc_guidance.py](file:///home/duy/VDT_project/simulation/control/smc_guidance.py) | `simulation/control/` | Dẫn đường SMC quỹ đạo dốc trượt $45^\circ$, điều khiển vận tốc hạ độ cao $\max(v_{touch}, 0.20 \cdot r_z)$ chống hover stall | **Hoàn tất** |
| [touchdown_detector.py](file:///home/duy/VDT_project/simulation/control/touchdown_detector.py) | `simulation/control/` | Phát hiện chạm đất qua động học ($z \le 0.18\text{m}, v_z \approx 0$) và setpoint vận tốc hạ, chốt tín hiệu sau 0.35s | **Hoàn tất** |
| [test_covariance_gate.py](file:///home/duy/VDT_project/tests/test_covariance_gate.py) | `tests/` | 13 unit tests kiểm chứng ma trận hiệp phương sai, adaptive switch, conical gate | **100% PASS** |
| [test_smc_guidance.py](file:///home/duy/VDT_project/tests/test_smc_guidance.py) | `tests/` | 10 unit tests kiểm chứng mặt trượt SMC, power reaching law, soft descent profile | **100% PASS** |
| [test_dual_scale_board.py](file:///home/duy/VDT_project/tests/test_dual_scale_board.py) | `tests/` | 11 unit tests kiểm chứng phân giải tọa độ Dual-scale ArUco Board | **100% PASS** |

### 8.3. Cấu trúc module thực tế trong Repository

```
simulation/
├── core/
│   ├── mission_fsm_node.py         # FSM hoàn chỉnh (APPROACH → GLIDE_SLOPE → LAND → TOUCHDOWN)
│   ├── offboard_commander.py       # Offboard interface + MAVLink Force Disarm
│   └── takeoff.py
├── control/
│   ├── apf_planner.py              # Improved APF 3D tránh vật cản
│   ├── ibvs_controller.py          # IBVS yaw & gimbal pitch (hỗ trợ nadir tilt khi land)
│   ├── covariance_gate.py          # Dual-EKF Covariance Fusion & Conical Gate
│   ├── smc_guidance.py             # SMC Glide Slope & Soft Landing Profile
│   └── touchdown_detector.py       # Drift-invariant Touchdown Detection
├── perception/
│   ├── aruco_sim_node.py           # Nested Dual-Scale ArUco Board solver (ID 42 + ID 43)
│   └── ekf_ros_adapter.py          # Target EKF adapter với dải quan sát 0.05m - 10m
└── config/
    └── mission_params.yaml         # Cấu hình tập trung toàn bộ hệ thống
```

---

## 9. Tiến độ thực tế theo Sprint (TẤT CẢ ĐÃ HOÀN TẤT)

### Sprint 1: SMC Core & Math Guidance (Đã hoàn thành - 100%)
- [x] Triển khai mặt trượt SMC $S = [S_1, S_2, S_3]^T$ và Power reaching law trong `smc_guidance.py`.
- [x] Tạo bộ profile soft-landing giảm chấn động học khi tiếp cận mặt đất.
- [x] 10 unit tests trong `tests/test_smc_guidance.py` đạt 100% PASS.

### Sprint 2: Covariance Gate & Touchdown Detection (Đã hoàn thành - 100%)
- [x] Triển khai `covariance_gate.py`: tính toán $P_{relative} = P_{target} + P_{drone}$, nón chấp nhận $30^\circ$, adaptive $R_{switch} \in [4.5\text{m}, 10.0\text{m}]$.
- [x] Triển khai `touchdown_detector.py`: lọc đa tầng kết hợp độ cao $z \le 0.18\text{m}$, vận tốc $v_z \le 0.15\text{m/s}$, chốt nhận diện sau 0.35s xác nhận.
- [x] Loại bỏ triệt để topic race condition bằng cách đăng ký duy nhất `/mission/velocity_setpoint`.
- [x] 13 unit tests trong `tests/test_covariance_gate.py` đạt 100% PASS.

### Sprint 3: Tích hợp ROS 2 Pipeline & SITL (Đã hoàn thành - 100%)
- [x] Tích hợp FSM trong `mission_fsm_node.py` với các trạng thái GLIDE_SLOPE, LAND, TOUCHDOWN.
- [x] Tích hợp Dual-scale ArUco Board (52.5cm outer + 10cm inner) đồng gốc tọa độ tại tâm H-Pad.
- [x] Tích hợp cơ chế ngắt MAVLink Force-Disarm an toàn trong `offboard_commander.py`.
- [x] Khởi chạy thành công toàn bộ pipeline trong Gazebo Harmonic.

### Sprint 4: Tuning, Edge Cases & Kiểm chứng Thực nghiệm (Đã hoàn thành - 100%)
- [x] Thử nghiệm tiếp đất trên PX4 SITL Gazebo: drone hạ cánh chính xác vào tâm H-Pad với sai số $R_{xy} \approx 0.01\text{m}$ (1 cm), vận tốc tiếp đất $\approx 0.15\text{m/s}$.
- [x] Kiểm chứng logic Wave-off abort khi mất dấu trên không ($z > 0.4\text{m}$) và khóa wave-off an toàn khi đã tiếp xúc mặt bãi đáp ($z \le 0.18\text{m}$).
- [x] Tự động Disarm an toàn, drone dừng hẳn trên bãi đáp, hoàn thành nhiệm vụ và chuyển về IDLE.

---

## 10. Phụ lục — Tham số khuyến nghị

### 10.1. Tham số SMC Guidance (từ Tuân, giữ nguyên cho validation)

```yaml
landing_guidance:
  ros__parameters:
    # SMC Sliding Surface gains
    ka: 0.2          # Tốc độ khép Rxy (s⁻¹)
    kb: 0.6          # Tốc độ áp góc dốc (s⁻¹)
    kc: 0.4          # Tốc độ căn chỉnh phương vị (s⁻¹)
    
    # SMC Power reaching law
    k1: 0.1395       # Gain mặt trượt S1
    k2: 0.1784       # Gain mặt trượt S2
    k3: 0.0442       # Gain mặt trượt S3
    m: 5             # Mẫu số lũy thừa (odd, positive)
    n: 3             # Tử số lũy thừa (odd, 0 < n < m)
    
    # Terminal angle constraints
    theta_des: 0.7854   # π/4 = 45° góc tiếp cận đứng
    zeta_des: 0.0       # 0° góc phương vị mong muốn
    
    # Safety limits
    Rmin: 0.1           # Khoảng cách tối thiểu LOS (m)
    dVp_max: 10.0       # Giới hạn gia tốc dọc (m/s²)
    dalpha_max: 1.5708  # π/2 giới hạn tốc độ quay hướng (rad/s)
    dgamma_max: 1.5708  # π/2 giới hạn tốc độ đổi dốc (rad/s)
```

### 10.2. Tham số Covariance Gate

```yaml
covariance_gate:
  ros__parameters:
    # Ngưỡng σ cho phép chuyển pha
    sigma_max_enter_glide: 0.10    # σ_relative < 10cm mới cho vào GLIDE
    sigma_max_continue_glide: 0.15 # σ_relative < 15cm mới tiếp tục GLIDE
    sigma_max_final_descent: 0.06  # σ_relative < 6cm mới cho vào FINAL
    
    # Adaptive Rswitch
    Rswitch_max: 15.0      # m (khi σ tốt nhất)
    Rswitch_min: 3.0       # m (khi σ tệ nhất)
    sigma_ideal: 0.05      # m
    sigma_bad: 0.30        # m
    
    # Pad geometry
    pad_half_width: 0.4205 # m (nửa cạnh ngắn A0; inscribed landing radius)
    confidence_level: 3.0  # n-sigma cho confidence ellipse
    
    # Abort timing
    abort_timeout: 3.0     # s (mất marker bao lâu thì abort GLIDE)
```

### 10.3. Tham số Touchdown Detector

```yaml
touchdown_detector:
  ros__parameters:
    alt_threshold: 0.15        # m (AGL)
    vel_threshold: 0.3         # m/s (relative speed)
    accel_z_min: 8.5           # m/s² (≈ 1g − 1.3)
    accel_z_max: 11.5          # m/s² (≈ 1g + 1.7)
    jerk_threshold: 5.0        # m/s³ (impact spike)
    confirm_cycles: 10         # × 50ms = 0.5s confirmation
    bounce_reset_factor: 1.5   # Reset nếu alt > threshold × 1.5
```

### 10.4. Tham số Final Descent

```yaml
final_descent:
  ros__parameters:
    descent_speed_initial: -0.20   # m/s (bắt đầu)
    descent_speed_final: -0.15     # m/s (gần chạm)
    descent_slow_altitude: 0.5     # m (giảm tốc ở 0.5m)
    xy_hold_kp: 2.0               # Gain giữ vị trí XY
    gimbal_pitch_final: -90.0      # độ (nhìn thẳng xuống)
```

---

> [!IMPORTANT]
> **Ưu tiên cao nhất**: Sprint 1 (port + validate SMC core) và Sprint 2 (covariance gate) có thể chạy **song song** vì hai module này độc lập. Sprint 3 mới cần kết quả của cả hai.

> [!TIP]
> **Validation nhanh**: Trước khi chạy SITL, có thể viết script Python mô phỏng offline: tạo trajectory drone bay từ xa về pad, đưa qua SMC → kiểm tra quỹ đạo có bám 45° không, vận tốc chạm có < 0.3 m/s không. Dùng `matplotlib` vẽ 3D trajectory.
