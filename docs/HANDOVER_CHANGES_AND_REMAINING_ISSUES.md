# BÁO CÁO TỔNG HỢP THAY ĐỔI THUẬT TOÁN, THÔNG SỐ VÀ PHÂN TÍCH CÁC VẤN ĐỀ TỒN ĐỌNG
## Dự án: Quadrotor Bám mục tiêu di động, Né vật cản 3D & Hạ cánh chính xác trên H-Pad (VDT Project)

> **Mục đích tài liệu**: Tổng hợp chi tiết toàn bộ các thay đổi về logic thuật toán và thông số đã cập nhật ngày 21/10/2026; đồng thời cung cấp phân tích chuyên sâu (Root Cause Analysis - RCA) về các vấn đề tồn đọng (rung lắc follow, trễ yaw, điều khiển H-Pad giật cục, và vi phạm khoảng cách an toàn vật cản) để **Codex / Embedded Team** tiếp tục đọc, phân tích và triển khai giải pháp xử lý.  
> **Repository Branch**: `feature/precision-landing`  
> **Commit Hash**: `08e94d291151a9b19ed5dcf1496bf791e8d40159`  
> **Ngày cập nhật**: 21/10/2026

---

## PHẦN 1: TỔNG HỢP NỘI DUNG ĐÃ THAY ĐỔI NGÀY HÔM NAY

### 1.1. Cập nhật Logic Thuật toán (Algorithm Logic Changes)

#### A. Thuật toán Dẫn đường Hạ cánh SMC (`simulation/control/smc_guidance.py` & `embedded_handover/algorithms/smc_guidance.py`)
1. **Bổ sung Vùng đệm Treo Giữ Độ cao (Buffer-hold Zone)**:
   * *Nguyên nhân*: Khi bãi đáp di chuyển vận tốc trung bình đến cao ($0.5\,\text{m/s}$), drone tiếp cận theo góc chéo và có xu hướng lao chéo thẳng xuống sàn mà chưa kịp hội tụ sai số ngang $r_{xy}$, dẫn đến việc tiếp đất với vận tốc ngang lớn, làm nghiêng thân và lật cánh khi chạm đất.
   * *Logic mới*: Tại độ cao chuyển tiếp `final_descent_alt_m` ($0.60\,\text{m}$), nếu $r_{xy} > \text{final\_descent\_rxy\_m}$ ($0.80\,\text{m}$), thuật toán ép $v_z = 0.0\,\text{m/s}$ (giữ nguyên độ cao đệm) và dành toàn bộ quyền điều khiển vận tốc ngang để kéo tâm drone vào marker. Chỉ khi $r_{xy} \le 0.80\,\text{m}$ drone mới được phép tiếp tục hạ thẳng đứng.
2. **Làm mượt biên dạng hạ thẳng đứng (Smooth Altitude Tapering)**:
   * *Nguyên nhân*: Cơ chế cũ chuyển trạng thái nhảy bậc thô bạo giữa $v_{\text{descend\_fast}}$ ($0.50\,\text{m/s}$) và $v_{\text{descend\_touch}}$ ($0.22\,\text{m/s}$) gây rung lắc gimbal và giật góc nhìn camera.
   * *Logic mới*: Nội suy tuyến tính trơn giữa cận trên khoảng đệm (`cushion_high = 0.50m`) và cận dưới tiếp đất (`touchdown_low = 0.25m`).
3. **Cơ chế hạ bò phá bế tắc cổng hiệp phương sai (`FINAL_GATE_WAIT Creep`)**:
   * *Nguyên nhân*: Khi bãi đáp di động chạy liên tục, độ bất định EKF ($\sigma$) thường tăng cao khiến Covariance Gate chặn lệnh hạ ($v_z = 0$). Drone bay lơ lửng bám theo marker suốt 18m chiều dài map mà không thể hạ cánh.
   * *Logic mới*: Thay vì treo đứng $v_z = 0$, cho phép hạ bò chậm $v_z = -0.15\,\text{m/s}$ với bộ đếm timeout $0.50\,\text{s}$. Nếu độ cao tương đối $r_z \le 0.35\,\text{m}$, hệ thống bypass cổng an toàn để ưu tiên tiếp đất chốt hạ.

#### B. Cơ chế Nhận diện Tiếp đất & Ngắt Động cơ (`touchdown_detector.py` & `offboard_commander.py`)
1. **Khắc phục xung đột hình học càng đáp x500/S500**:
   * Chiều cao chân càng thực tế của quadrotor x500 là $0.18 - 0.24\,\text{m}$. Ngưỡng nhận diện tiếp đất và chốt bảo vệ khẩn cấp cũ bị fix cứng ở mức $0.18\,\text{m}$ và $0.12\,\text{m}$, khiến drone dù đã nằm trên mặt bãi đáp vẫn bị coi là đang bay lơ lửng, không thể kích hoạt DISARM.
   * Đã nâng trần nhận diện `altitude_ceiling_m` và ngưỡng ngắt động cơ khẩn cấp lên **$0.35\,\text{m}$**.
2. **Tối ưu thời gian xác nhận tiếp đất (`confirmation_duration_s`)**:
   * Rút ngắn thời gian xác thực từ $0.35\,\text{s} \to \mathbf{0.15\,\text{s}}$ giúp lệnh ngắt động cơ phát ra ngay lập tức khi chân càng chạm mặt bãi đáp, triệt tiêu hoàn toàn hiệu ứng phản lực mặt đất (ground effect) thổi lật thân drone.
3. **Chống kích hoạt Waveoff giả sát sàn (`mission_fsm_node.py`)**:
   * Tăng ngưỡng nhận diện drone nằm nghỉ trên mặt sàn `is_resting_on_ground` từ $0.12\,\text{m} \to \mathbf{0.28\,\text{m}}$ (hold $0.3\,\text{s}$).
   * Chặn lệnh hủy hạ cánh (waveoff) khẩn cấp khi drone đã ở độ cao dưới $0.35\,\text{m}$ hoặc sai số $h_{\text{err}} < 0.95\,\text{m}$.

---

### 1.2. Bảng Thông số đã Hiệu chỉnh (Tuning Parameters)

| Tham số cấu hình | Giá trị gốc | Giá trị mới | File cấu hình | Lý do kỹ thuật |
| :--- | :---: | :---: | :--- | :--- |
| `apf_planner.v_max` | $1.80\,\text{m/s}$ | **$1.70\,\text{m/s}$** | `mission_params.yaml` | Giảm quán tính vọt lố khi bãi đáp dừng lại đột ngột |
| `apf_planner.target_velocity_alpha` | $0.35$ | **$0.18$** | `mission_params.yaml` | Bộ lọc EMA mạnh hơn, khử nhiễu ước lượng vận tốc marker |
| `apf_planner.target_lead_time` | $0.35\,\text{s}$ | **$0.25\,\text{s}$** | `mission_params.yaml` | Giảm khoảng cách ảo đón đầu, chống dao động goal |
| `apf_planner.max_target_lead` | $1.00\,\text{m}$ | **$0.50\,\text{m}$** | `mission_params.yaml` | Khống chế điểm đích không nhảy quá xa khi marker đổi hướng |
| `apf_planner.target_velocity_deadband` | $0.18\,\text{m/s}$ | **$0.15\,\text{m/s}$** | `mission_params.yaml` | Tăng độ nhạy kích hoạt feedforward ở dải tốc độ thấp |
| `offboard_commander.max_accel` | $0.90\,\text{m/s}^2$ | **$0.75\,\text{m/s}^2$** | `mission_params.yaml` | Giảm gia tốc tối đa qua bộ lọc slew-rate để triệt tiêu chattering |
| `smc_guidance.glide_centering_kp` | $0.70$ | **$0.50$** | `mission_params.yaml` | Giảm độ nhạy phản hồi vị trí ngang trong pha Glide Slope |
| `smc_guidance.final_centering_kp` | $0.70$ | **$0.60$** | `mission_params.yaml` | Ổn định tâm mượt mà ở pha Final Descent |
| `smc_guidance.target_velocity_ff_gain` | $0.90$ | **$0.80$** | `mission_params.yaml` | Feedforward 80% vận tốc tránh quá điều khiển (over-compensation) |
| `smc_guidance.v_final_xy_max` | $0.45\,\text{m/s}$ | **$0.40\,\text{m/s}$** | `mission_params.yaml` | Giới hạn vận tốc dồn tâm ngang sát mặt đất |
| `smc_guidance.final_descent_alt_m` | $0.50\,\text{m}$ | **$0.60\,\text{m}$** | `mission_params.yaml` | Độ cao bắt đầu kích hoạt buffer zone kiểm tra tâm |
| `smc_guidance.final_descent_rxy_m` | $0.60\,\text{m}$ | **$0.80\,\text{m}$** | `mission_params.yaml` | Nới rộng điều kiện bán kính chấp nhận hạ cánh động |
| `smc_guidance.final_uncertainty_max_2sigma_m` | $0.50\,\text{m}$ | **$0.90\,\text{m}$** | `mission_params.yaml` | Tránh bị treo lơ lửng khi covariance của marker di động mở rộng |
| `touchdown_detector.altitude_ceiling_m` | $0.18\,\text{m}$ | **$0.35\,\text{m}$** | `mission_params.yaml` | Phù hợp độ cao chân càng x500 khi đậu trên nóc bãi đáp |
| `touchdown_detector.confirmation_duration_s` | $0.35\,\text{s}$ | **$0.15\,\text{s}$** | `mission_params.yaml` | Ngắt động cơ tức thì sau khi chạm bãi đáp |

---

## PHẦN 2: PHÂN TÍCH CHUYÊN SÂU CÁC VẤN ĐỀ TỒN ĐỌNG (RCA)

Dưới đây là 4 vấn đề kỹ thuật lớn người vận hành ghi nhận trong quá trình bay thử thực tế, kèm phân tích bản chất thuật toán và mã nguồn:

### Vấn đề 1: Hiện tượng Lắc Lư Thân (Tilt Oscillation / Chattering) trong pha Follow
* **Triệu chứng**: Khi bám theo bãi đáp, quadrotor liên tục chao đảo góc nghiêng thân (Roll/Pitch hunting), cảm giác bay không mượt, có hiện tượng chattering tần số cao.
* **Nguyên nhân gốc rễ (Root Cause)**:
  1. **Khớp nối gia tốc PX4 Attitude Controller**: PX4 chuyển đổi lệnh vận tốc $v_{xy}^{\text{cmd}}$ sang góc nghiêng $\phi_{\text{cmd}}, \theta_{\text{cmd}} \approx \frac{1}{g} \dot{v}_{xy}$. Bất kỳ bước nhảy hoặc nhiễu tần số cao nào trên $v_{xy}^{\text{cmd}}$ đều dẫn đến vi phân gia tốc lớn, làm góc nghiêng thân giật liên tục.
  2. **Gradient lực đẩy APF từ 30 trụ vật cản**: Trong `apf_planner.py`, lực đẩy tính theo $\nabla U_{\text{rep}} \propto \frac{1}{d^3}$. Khi drone bay gần các trụ vật cản, chỉ cần một biến thiên nhỏ của tọa độ $d$ cũng tạo ra một xung lực $\Delta f_{\text{rep}}$ cực lớn đẩy lệch vector vận tốc.
  3. **Xung đột giữa Stand-off Distance và Target Lead**: Vector đích đến `target_goal` được tính từ vị trí marker cộng với lượng đón đầu $v_{\text{target}} \cdot t_{\text{lead}}$. Khi vận tốc marker từ EKF bị rung nhẹ hoặc bãi đáp đổi hướng, điểm `target_goal` dịch chuyển giật cục khiến bộ điều khiển PD liên tục đảo chiều vector gia tốc.

### Vấn đề 2: Điều khiển H-Pad bằng phím A-W-S-D đổi hướng đột ngột (Step Input phi thực tế)
* **Triệu chứng**: Thử nghiệm bằng phím bấm `hpad_keyboard_controller.py` gây ra các bước nhảy vận tốc vuông góc $90^\circ$ tức thời ($v_x = \text{speed}, v_y = 0 \to v_x = 0, v_y = \text{speed}$).
* **Nguyên nhân gốc rễ (Root Cause)**:
  * Trong file `simulation/utils/hpad_keyboard_controller.py`:
    ```python
    if key == 'w':
        self.vx, self.vy = self.speed, 0.0
    elif key == 'a':
        self.vx, self.vy = 0.0, self.speed
    ```
  * Đây là **hàm bước nhảy lý tưởng (Step function với gia tốc $\ddot{x} = \infty$)**, hoàn toàn không phản ánh đúng động học của các phương tiện chở bãi đáp thực tế (xe tự hành UGV, tàu không người lái USV, robot kéo). Xe thực tế luôn có quán tính, khối lượng, gia tốc hữu hạn và góc bẻ lái mượt (Ackermann / Unicycle kinematics).
  * Bước nhảy $90^\circ$ tức thời làm đứt gãy mô hình chuyển động vận tốc không đổi (Constant Velocity) của bộ lọc Kalman EKF, tạo ra lượng thặng dư đo (Innovation) cực lớn làm EKF bị trễ và covariance phình to đột biến.

### Vấn đề 3: Điều khiển góc Yaw quay chậm, không theo kịp mục tiêu khi rẽ hướng
* **Triệu chứng**: Khi H-Pad rẽ góc, góc Yaw của drone quay quá chậm, không bắt kịp hướng di chuyển của mục tiêu, làm marker lệch khỏi tâm ảnh và có nguy cơ trôi khỏi FOV camera.
* **Nguyên nhân gốc rễ (Root Cause)**:
  1. **Khống chế giới hạn tốc độ quay Yaw quá chặt (`yaw_rate_limit`)**:
     * Trong `simulation/config/mission_params.yaml`, `yaw_rate_limit: 0.80 rad/s` ($\approx 45.8^\circ/\text{s}$). Khi bãi đáp rẽ vuông góc $90^\circ$, drone cần ít nhất:
       $$t = \frac{\pi / 2}{0.80} \approx 1.96\,\text{giây}$$
       chỉ để quay xong góc thân! Trong gần $2\,\text{giây}$ này, camera hướng lệch sang một bên, làm marker trượt ra sát mép FOV ($69^\circ$ ngang của RealSense D435i).
  2. **Trần Feedforward vận tốc góc quá nhỏ**:
     * Trong `simulation/control/tracking_control.py` (hàm `update_tracking_yaw_command`):
       ```python
       feedforward_max_rate: float = 0.15  # rad/s (~8.6 deg/s)
       ```
       Trần feedforward $0.15\,\text{rad/s}$ quá bé so với tốc độ đổi góc phương vị (Line-of-Sight rate) khi bãi đáp rẽ ở cự ly gần ($d \approx 2.2\,\text{m}$), khiến thành phần feedforward gần như không phát huy tác dụng kéo góc yaw.
  3. **Hiệu ứng Coup Roll-to-Yaw**: Khi drone nghiêng thân để tăng tốc ngang đuổi theo H-Pad đang rẽ, trục quang học camera bị nghiêng theo mặt phẳng Roll/Pitch, biến đổi sai số pixel ngang $u$ thành sai số phi tuyến, khiến bộ điều khiển Yaw bị trôi điểm đặt.

### Vấn đề 4: Vi phạm Khoảng Cách An Toàn tới Vật Cản trong APF Planner
* **Triệu chứng**: Drone dù đã phát hiện vật cản trong tầm nhìn nhưng vẫn lao tới rất sát vật cản (vi phạm cự ly an toàn $d_0$) trước khi bắt đầu né hoặc thậm chí có nguy cơ va chạm.
* **Nguyên nhân gốc rễ (Cực kỳ quan trọng trong công thức IAPF)**:
  * Trong file `simulation/control/iapf_core.py` (dòng 176–199), thuật toán IAPF sử dụng kỹ thuật Sigmoid Weighting để giải quyết bài toán GNRON (Mục tiêu không tới được khi gần vật cản):
    ```python
    exp_goal = math.exp(-min(d_goal, 50.0))
    weight_rep = 1.0 / (1.0 + exp_goal) - 0.5
    sigmoid_deriv = exp_goal / ((1.0 + exp_goal) ** 2)

    # Lực đẩy né vật cản:
    coeff1 = 2.0 * k_rep * (1.0 / d_safe - 1.0 / d0) * (1.0 / (d_safe ** 2)) * weight_rep
    f_rep1 = coeff1 * dir_away

    # Lực hút hướng về mục tiêu:
    coeff2 = k_rep * ((1.0 / d_safe - 1.0 / d0) ** 2) * sigmoid_deriv
    f_rep2 = coeff2 * dir_goal
    ```
  * **LỖI THIẾT KẾ THUẬT TOÁN**:
    1. Khi drone đang ở pha FOLLOW bám mục tiêu di động, khoảng cách tới đích ảo $d_{\text{goal}} = \|\text{drone} - \text{goal}\|$ **thường xuyên rất nhỏ** ($0.2\,\text{m} - 0.5\,\text{m}$) vì drone đã bám sát điểm stand-off.
    2. Khi $d_{\text{goal}} \to 0$, số hạng $\exp(-d_{\text{goal}}) \to 1.0$, dẫn đến:
       $$\text{weight\_rep} = \frac{1}{1.0 + 1.0} - 0.5 = 0.0$$
       **Lực đẩy né vật cản $f_{\text{rep1}}$ bị triệt tiêu gần như hoàn toàn (suy giảm đến 90–95%)!**
    3. Nguy hiểm hơn, thành phần thứ hai $f_{\text{rep2}} = \text{coeff2} \cdot \vec{u}_{\text{goal}}$ lại **kéo thẳng drone về phía mục tiêu**. Nếu vật cản nằm chắn giữa drone và bãi đáp, thành phần này sẽ **hút drone lao thẳng vào vật cản!**
    4. **Thiếu Bán Kính Bao Drone (Collision Margin)**: Hiện tại khoảng cách $d$ được tính tới bề mặt vật cản (`cylinder_nearest_point`), nhưng chưa cộng thêm bán kính an toàn của khung drone ($R_{\text{drone}} \approx 0.35\,\text{m}$ cho cánh quạt x500) và khoảng đệm phanh động học ($d_{\text{braking}} \approx \frac{v^2}{2 a_{\text{max}}}$).

---

## PHẦN 3: ĐỀ XUẤT HƯỚNG XỬ LÝ CHO THÀNH VIÊN EMBEDDED / CODEX

Dưới đây là các khuyến nghị kỹ thuật cụ thể để nhóm triển khai khắc phục triệt để các vấn đề trên:

### 1. Khắc phục Lực đẩy Vật cản trong IAPF (`iapf_core.py`)
* Không áp dụng hàm suy giảm GNRON Sigmoid Weighting đối với bám mục tiêu động (`dynamic_tracking_mode`). Trong chế độ bám mục tiêu, lực repulsive phải luôn giữ nguyên $100\%$ cường độ:
  $$\text{weight\_rep} = 1.0$$
  $$f_{\text{rep2}} = 0$$
* Bổ sung bán kính an toàn vật lý của drone:
  $$d_{\text{effective}} = d_{\text{measured}} - R_{\text{safety\_margin}} \quad (R_{\text{safety\_margin}} \ge 0.50\,\text{m})$$
* Bổ sung cơ chế giảm tốc độ trần $v_{\text{max}}$ phụ thuộc vào khoảng cách tới vật cản gần nhất:
  $$v_{\text{limit}} = \min\left(v_{\text{max}}, \sqrt{2 \cdot a_{\text{decel}} \cdot \max(0, d_{\text{obs}} - d_{\text{stop}})}\right)$$

### 2. Khắc phục Bộ điều khiển H-Pad Teleop (`hpad_keyboard_controller.py`)
* Bổ sung mô hình động học xe mượt mà (First-Order Lag hoặc Ramp Rate Limiter) cho vận tốc bàn phím:
  $$v_{x}(k+1) = v_{x}(k) + \text{clip}(v_{x,\text{cmd}} - v_{x}(k), -a_{\text{max}} \Delta t, a_{\text{max}} \Delta t)$$
  với $a_{\text{max}} \approx 0.4 - 0.6\,\text{m/s}^2$.
* Bổ sung chế độ lái vi sai (Unicycle: tiến/lùi bằng W/S, quay góc Heading bãi đáp mượt bằng A/D) thay vì gán trực tiếp bước nhảy vuông góc.

### 3. Tối ưu hóa Điều khiển Bám Yaw (`tracking_control.py` & `mission_params.yaml`)
* Tăng `yaw_rate_limit` từ $0.80\,\text{rad/s} \to 1.40 - 1.80\,\text{rad/s}$ ($\approx 80^\circ - 100^\circ/\text{s}$) để drone có đủ khả năng bẻ góc nhanh khi bãi đáp chuyển hướng.
* Nâng trần feedforward vận tốc góc `feedforward_max_rate` từ $0.15\,\text{rad/s} \to 0.80\,\text{rad/s}$.
* Áp dụng bộ bù góc nghiêng (Tilt Compensation): Dùng ma trận quay IMU để chiếu tọa độ tâm marker từ Camera Optical Frame sang Horizontal Plane trước khi tính góc Yaw, giúp triệt tiêu hiện tượng lệch góc Yaw khi drone đang nghiêng thân bay nhanh.

### 4. Triệt tiêu Rung lắc Thân Follow
* Áp dụng bộ lọc bẫy vận tốc (Velocity Slew-rate Limiter) 2 trục độc lập có tính đến góc nghiêng tối đa của PX4.
* Đặt vùng chết (Deadband) cho lực tổng hợp APF: Khi sai số vị trí bám đuổi nằm trong khoảng $\pm 0.15\,\text{m}$, gán $F_{\text{att}} = 0$ để tránh drone liên tục điều chỉnh góc nghiêng quanh vị trí cân bằng.
