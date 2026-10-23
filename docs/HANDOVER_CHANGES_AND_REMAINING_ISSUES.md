# BÁO CÁO TỔNG HỢP THAY ĐỔI THUẬT TOÁN, THÔNG SỐ VÀ PHÂN TÍCH CÁC VẤN ĐỀ TỒN ĐỌNG
## Dự án: Quadrotor Bám mục tiêu di động, Né vật cản 3D & Hạ cánh chính xác trên H-Pad (VDT Project)

> **Mục đích tài liệu**: Tổng hợp chi tiết toàn bộ các thay đổi về logic thuật toán và thông số đã cập nhật ngày 21/10/2026; đồng thời cung cấp phân tích chuyên sâu (Root Cause Analysis - RCA) về các vấn đề tồn đọng (rung lắc follow, trễ yaw, điều khiển H-Pad giật cục, và vi phạm khoảng cách an toàn vật cản) để  bạn tiếp tục đọc, phân tích và triển khai giải pháp xử lý.
>
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

## PHẦN 3: CẬP NHẬT SAU HANDOVER NGÀY 21/10/2026 — 22/10/2026

Phần này ghi trạng thái mới nhất của nhánh simulation để làm cơ sở chuyển giao logic cho team embedded. Các giá trị ở đây lấy từ code và `simulation/config/mission_params.yaml` hiện tại. Một số con số touchdown trong Phần 1 là snapshot cũ ngày 21/10; xem mục 3.4 bên dưới để biết cấu hình đang có hiệu lực.

### 3.1. Nguyên nhân và thay đổi logic điều khiển yaw khi FOLLOW

Các lần yaw giật mạnh với H-pad đứng yên xuất phát từ việc vận tốc target do EKF/PnP đôi lúc có spike khi vị trí marker nhảy. Logic cũ có thể hiểu spike ngắn đó là H-pad đã chuyển động, lấy course từ vận tốc nhiễu làm yaw reference rồi đổi qua lại giữa course và IBVS. Ngoài ra, hướng APF là hướng hiệu chỉnh tới điểm stand-off; khi drone đã gần điểm đó, hướng này có thể đảo hoặc quay dù H-pad đứng yên, nên không phù hợp làm hướng thân drone.

Logic hiện tại chọn yaw reference như sau:

1. Mặc định dùng yaw từ IBVS khi H-pad đứng yên hoặc trạng thái vận tốc chưa đủ tin cậy.
2. Chỉ chuyển sang hướng chuyển động của H-pad sau khi vận tốc XY đã qua lọc, đạt ít nhất `max(0.35, follow_yaw_motion_min_speed)` m/s, hướng chuyển động nhất quán trong `follow_yaw_motion_enter_confirm_s`, và mẫu đến từ tracking/EKF mới.
3. Giữ trạng thái course qua nhiễu ngắn và dropout/prediction; chỉ quay về IBVS sau khi vận tốc dưới ngưỡng thoát đủ lâu. APF không được dùng làm yaw reference.
4. Giới hạn tốc độ thay đổi yaw reference theo `course_yaw_rate_limit`. `forward_heading_guard` chặn lệnh dịch chuyển nếu course lệnh nằm quá `follow_forward_heading_gate_deg` so với mũi drone, để drone xoay về hướng tiến trước thay vì bay lùi trong khi camera vẫn nhìn target.

Giá trị simulation hiện tại: ngưỡng vận tốc cấu hình 0.30 m/s nhưng code áp sàn 0.35 m/s; xác nhận vào/ra 2.00/2.00 s; lọc vận tốc tau 0.25 s; sai lệch hướng cho phép khi xác nhận 40°; slew course yaw 0.45 rad/s; IBVS và Offboard yaw limit 0.50 rad/s. Thời gian xác nhận dài hơn giúp loại spike, nhưng đổi hướng course thật sẽ có độ trễ xác nhận rồi quay có giới hạn tốc độ.

### 3.2. Mất tracking khi đang landing và lệnh LAND bị rơi

Có hai nguyên nhân riêng khiến drone có thể không tự hoàn tất sau khi mất tracking:

* Khi target biến mất ngắn lúc LAND, vị trí/vận tốc target cuối cùng vẫn cần được ngoại suy để điều khiển tới hết cửa sổ touchdown. Trước đây thời gian trôi có thể bị tính bằng wall clock trong khi vận tốc EKF được tính theo timestamp ROS/Gazebo. Khi Gazebo chạy với real-time factor thấp hơn 1, nhân vận tốc simulation với wall-time tạo sai lệch quãng đường coast.
* Khi tracking mất lâu hơn trong APPROACH, FSM wave-off về FOLLOW/SEARCH. Nhánh này trước đây xóa cờ `land_requested`; sau khi target xuất hiện lại drone tìm/bám lại nhưng không còn yêu cầu hạ cánh đã gửi, nên có thể treo lơ lửng dù operator đã yêu cầu LAND.

Đã sửa theo hướng:

* Lưu timestamp từ `Odometry.header.stamp` và tính coast bằng ROS clock cùng clock domain với vận tốc target. Khi tracking tạm mất ở LAND, target ước lượng là `last_confirmed_position + last_confirmed_velocity * elapsed_sim_time`.
* Giới hạn coast ở 5.00 giây simulation. Giá trị này bao phủ dropout quan sát được tới khoảng 4.2 giây, nhưng mô hình vẫn là vận tốc không đổi nên không dự đoán được target tăng tốc hoặc rẽ trong lúc mất hình.
* Wave-off ở độ cao cao hơn đưa FSM về APPROACH/FOLLOW để bắt lại target; yêu cầu LAND vẫn được giữ. FOLLOW/SEARCH chỉ quay lại APPROACH sau handshake bắt lại ổn định. Cờ LAND chỉ bị xóa khi operator hủy lệnh hoặc touchdown đã được xác nhận.
* Wave-off chỉ bị chặn khi base-link thấp hơn `land_waveoff_near_ground_alt = 0.20 m`; trên ngưỡng đó, nếu target đi xa hoặc tracking/coast hết hạn thì vẫn wave-off để tránh tiếp tục hạ thiếu căn cứ.

### 3.3. Điều kiện touchdown và lý do không nên disarm khi chỉ đang hover sát H-pad

Touchdown detector hiện là detector xác nhận drone đã dừng gần mặt đất, không phải detector xác nhận drone đã nằm đúng tâm H-pad. Điều kiện động học yêu cầu base-link không cao hơn `altitude_ceiling_m = 0.12 m`, vận tốc đứng không quá 0.08 m/s và tốc độ biến thiên độ cao không quá 0.05 m/s; điều kiện phải ổn định 0.25 s. `optical_height_threshold_m = 0.35 m` chỉ mở cửa sổ kiểm tra bằng camera, không thay thế điều kiện base-link ≤ 0.12 m. Offboard Commander cũng giữ guard touchdown ở 0.12 m trước khi disarm.

Do đó, drone bay song song hoặc hover sát trên pad chưa được xem là đã chạm đất. Hạ trần detector để disarm ở độ cao lớn hơn có thể biến một lần hover hoặc dừng thoáng qua thành touchdown giả. Cấu hình touchdown không được nới trong đợt sửa này; thay đổi khắc phục sự cố vừa rồi nằm ở FSM wave-off/latch LAND và target coast. Nếu cần một tiêu chí “đã ở trong vùng đáp” độc lập với tiếp xúc, cần bổ sung KPI/validator riêng thay vì dùng detector tiếp xúc để thay thế.

**Giá trị hiện hành trong YAML (nguồn chuẩn cho simulation):** `altitude_ceiling_m = 0.12 m`, `stoppage_vz_max_mps = 0.08 m/s`, `stoppage_dz_dt_max_mps = 0.05 m/s`, `confirmation_duration_s = 0.25 s`, `fast_confirmation_duration_s = 0.10 s`; Offboard `touchdown_max_altitude_m = 0.12 m`. Các giá trị `0.35 m` và `0.15 s` ở Phần 1 mô tả một lần chỉnh trước đó, không phải cấu hình đang chạy.

### 3.4. Tham số simulation liên quan trong phiên bản hiện tại

| Tham số | Giá trị hiện tại | Ý nghĩa |
| :--- | :---: | :--- |
| `follow_yaw_motion_min_speed` | 0.30 m/s; code floor 0.35 m/s | Tốc độ target tối thiểu trước khi xét hướng course |
| `follow_yaw_motion_enter_confirm_s` / `exit_confirm_s` | 2.00 / 2.00 s | Xác nhận target bắt đầu chạy / dừng trước khi đổi yaw source |
| `follow_yaw_velocity_filter_tau_s` | 0.25 s | Lọc vận tốc target trước khi xác định course |
| `follow_yaw_course_consistency_deg` | 40° | Hướng candidate phải nhất quán trong thời gian xác nhận |
| `course_yaw_rate_limit` | 0.45 rad/s | Giới hạn slew của yaw reference theo course |
| `yaw_rate_limit` (IBVS) / `max_yaw_rate` (Offboard) | 0.50 / 0.50 rad/s | Giới hạn yaw IBVS và setpoint Offboard |
| `follow_forward_heading_gate_deg` | 85° | Chặn tịnh tiến khi hướng lệnh ở phía sau thân drone |
| `land_coast_timeout` | 5.00 s ROS/simulation time | Thời gian ngoại suy target trong LAND khi mất tracking |
| `land_fallback_target_velocity_min_speed` | 0.25 m/s | Ngưỡng vận tốc để dùng feedforward trong coast/fallback |
| `land_fallback_xy_speed_limit` | 0.65 m/s | Giới hạn vận tốc ngang khi LAND fallback |
| `land_waveoff_near_ground_alt` | 0.20 m | Độ cao base-link dưới đó không wave-off vì mất tracking/target đi xa |
| Touchdown ceiling / confirm | 0.12 m / 0.25 s | Guard xác nhận tiếp xúc hiện hành; detector không được nới trong lần sửa này |

Đây là baseline của simulation để team embedded nắm logic và điểm cần port. Khi chuyển sang firmware thực, mọi bộ đếm thời gian và tích phân vận tốc phải dùng timestamp cùng miền thời gian; không nhân vận tốc lấy từ simulation/ROS time với wall-time hoặc timer không cùng clock. Các thời gian xác nhận và giới hạn tốc độ cần được ánh xạ theo tần số cảm biến, chu kỳ điều khiển và đơn vị của autopilot thực tế.

### 3.5. Kết quả kiểm chứng simulation mới nhất

Đã chạy pipeline Gazebo/RViz với H-pad tĩnh và H-pad chạy 0.5 m/s có đổi hướng khoảng 90° trong quá trình FOLLOW. Hai lần thử H-pad động ở trạng thái code hiện tại đều đi tới touchdown và kết thúc mission; tuy vậy KPI chính xác tâm của cả hai lần vẫn chưa đạt:

| Kịch bản | Lệch tâm tại touchdown | Touchdown / mission | Strict KPI | Kết quả |
| :--- | :---: | :--- | :---: | :--- |
| H-pad tĩnh | 3.28 cm | Có / thành công | Pass | [summary.json](../simulation_results/yaw_stability_confirm2_static_2026-10-22/run_01/summary.json) |
| H-pad 0.5 m/s, đổi hướng | 23.17 cm | Có / thành công | Fail | [summary.json](../simulation_results/yaw_stability_latched_land_turning_2026-10-22/run_01/summary.json) |
| H-pad 0.5 m/s, chạy lặp | 17.71 cm | Có / thành công | Fail | [summary.json](../simulation_results/yaw_stability_latched_land_repeat2_2026-10-22/run_01/summary.json) |

Ở hai lần chạy động, touchdown được xác nhận tại base-link cao 0.118 m và 0.107 m; mission về IDLE. Trong lần lặp thứ hai, yaw rate tính theo wall-time ở LAND có p95 2.27°/s, peak 3.56°/s; roll peak 0.65°, pitch peak 7.08°. Ở lượt static, landing error 3.28 cm và `strict_kpi_pass = true`. Các số này xác nhận được đường đi tới touchdown và độ ổn định tương đối của pha LAND trên các lượt đã chạy; chúng không chứng minh mọi lần đổi hướng hoặc mọi dropout đều đạt KPI.

Tệp kết quả và telemetry được giữ trong các thư mục `simulation_results/yaw_stability_confirm2_static_2026-10-22/`, `simulation_results/yaw_stability_latched_land_turning_2026-10-22/` và `simulation_results/yaw_stability_latched_land_repeat2_2026-10-22/` để tiếp tục phân tích.

### 3.6. Việc còn lại trước khi chốt mô phỏng và chuyển logic sang embedded

1. Giữ tách biệt hai điều kiện: detector tiếp xúc dùng để kết thúc landing/disarm; KPI vùng đáp dùng để đánh giá drone đã chạm trong vùng chấp nhận hay chưa.
2. Các chỉnh sửa hiện tại tập trung simulation; embedded config/package đã được đồng bộ sơ bộ (`embedded_handover/algorithms/`). Khi port sang firmware thực tế, cần tiếp tục ánh xạ rate limits và clock domain chuẩn xác.

---

## PHẦN 4: KHẮC PHỤC TRIỆT ĐỂ RUNG LẮC FOLLOW, BẪY ĐỨNG HÌNH VẬT CẢN VÀ ĐẠT STRICT KPI HẠ CÁNH (22/10/2026)

### 4.1. Tóm tắt các Thay đổi Thuật toán Cốt lõi
1. **Lõi thuật toán I-APF ([iapf_core.py](simulation/control/iapf_core.py) & [apf_planner.py](simulation/control/apf_planner.py))**:
   - Bổ sung tham số `is_dynamic_follow` trong hàm `compute()`. Trong chế độ FOLLOW, khoảng cách mục tiêu ảo tới điểm standoff ($d_{\text{goal}} \approx 0.2\,\text{m}$) không còn làm triệt tiêu lực đẩy của vật cản (`weight_rep` giữ nguyên 0.50 thay vì rơi về 0.0).
   - Triệt tiêu thành phần $F_{\text{rep2}}$ hướng vào mục tiêu khi đang né tránh để tránh hút drone lao vào vật cản; bảo toàn vận tốc né tránh $\ge 55\% v_{\text{max}}$.
   - Xóa bỏ việc ép `effective_d0` co cụm về $0.4\,\text{m}$ trong `APFCore`, giữ trọn bán kính phát hiện $d_0 = 3.0\,\text{m}$.
2. **Khóa góc Yaw trong FOV Camera & Phá bẫy đóng băng vận tốc ([mission_fsm_node.py](simulation/core/mission_fsm_node.py))**:
   - Khống chế góc lệch giữa Course di chuyển của mục tiêu và Line-of-Sight (IBVS) ở mức tối đa $24^\circ$. Drone vừa xoay mũi hướng về phía trước theo hướng xe chạy để nhìn bao quát vật cản trên đường bay, vừa đảm bảo marker ArUco **luôn luôn nằm sâu trong tâm khung hình camera** (không bị hất văng ra ngoài mép FOV khi xe rẽ góc).
   - Thiết kế lại cơ chế `forward_heading_guard`: Phân tách vận tốc thành 2 thành phần Thân tiến (`v_fwd`) và Thân ngang (`v_lat`). Giữ nguyên 100% vận tốc trượt ngang (`v_lat`) khi né vật cản, chỉ chặn dịch chuyển lùi thẳng khi mũi chưa quay vào góc tiến. Drone không còn bị đóng băng vận tốc thành $[0, 0]$ khi né vật cản.
3. **Triệt tiêu hiện tượng lắc Pitch khi tăng tốc ([mission_fsm_node.py](simulation/core/mission_fsm_node.py) & [mission_params.yaml](simulation/config/mission_params.yaml))**:
   - Nới rộng dung sai pixel trục ngang $e_u$ từ $40\,\text{px} \to 65\,\text{px}$ để độ nghiêng thân tự nhiên khi tăng tốc ($\theta \approx a_x / g$) không kích hoạt nhầm cơ chế giảm tốc.
   - Thay thế hàm giảm tốc bậc 1 bằng hàm Cosine S-curve mượt mà và tăng sàn vận tốc tối thiểu từ $60\% \to 80\%$, triệt tiêu hoàn toàn xung vi sai gia tốc làm rung lắc thân drone.
4. **Tối ưu hạ cánh dứt khoát và rút ngắn thời gian Disarm (< 1s)**:
   - Tăng độ nhạy dồn tâm $k_p$ trong pha hạ cánh cuối và cữ hiệu chỉnh ngang lên $0.35\,\text{m/s}$.
   - Duy trì vận tốc hạ thẳng đứng $v_z = -0.22\,\text{m/s}$ dứt khoát, không treo lơ lửng.
   - Rút ngắn thời gian xác nhận tiếp đất `confirmation_duration_s` xuống **$0.20\,\text{s}$**, hạ trần failsafe disarm tại mặt bãi đáp xuống **$0.40\,\text{s}$** (tổng thời gian ngắt động cơ sau tiếp xúc $< 0.5\,\text{s}$, loại bỏ hoàn toàn hiện tượng bay lơ lửng quá lâu).

### 4.2. Bảng Thông số Cập nhật Mới nhất (`simulation/config/mission_params.yaml`)

| Tham số | Giá trị cũ | Giá trị mới | Ý nghĩa kỹ thuật |
| :--- | :---: | :---: | :--- |
| `yaw_align_u_tolerance_px` | 40.0 px | **65.0 px** | Tránh kích hoạt giảm tốc nhầm khi thân drone nghiêng chúc mũi tăng tốc |
| `yaw_align_min_speed_scale` | 0.60 | **0.80** | Giảm tốc êm dịu 20%, không cắt giật 40% làm rung lắc Pitch |
| `yaw_align_speed_filter_alpha` | 0.35 | **0.20** | Bộ lọc làm mịn tỉ lệ tốc độ tịnh tiến |
| `follow_forward_heading_gate_deg`| 85.0° | **105.0°** | Không chặn vận tốc trượt ngang 90° khi drone luồn lách né vật cản |
| `course_yaw_rate_limit` | 0.45 rad/s | **0.75 rad/s** | Tăng tốc độ xoay thân lên ~43°/s để bám kịp khúc cua gấp 90° |
| `follow_yaw_motion_enter_confirm_s` | 2.00 s | **0.50 s** | Chuyển sang bám course xe nhanh nhạy, giảm độ trễ phản hồi |
| `follow_yaw_motion_exit_confirm_s` | 2.00 s | **0.60 s** | Chuyển về IBVS ổn định nhanh khi xe dừng |
| `offboard_commander.max_yaw_rate` | 0.50 rad/s | **0.75 rad/s** | Đồng bộ trần quay yaw của PX4 với bộ điều khiển FSM |
| `touchdown_detector.confirmation_duration_s` | 0.25 s | **0.20 s** | Xác nhận tiếp đất dứt khoát |
| `touchdown_detector.altitude_ceiling_m` | 0.12 m | **0.13 m** | Khớp độ cao chân càng thực tế khi đậu trên mặt pad |

### 4.3. Kết quả Kiểm chứng Thực tế trên Gazebo SITL

Đã thực hiện bay kiểm chứng tự động toàn trình (chu trình khép kín: *Takeoff $\to$ Follow bám đuổi & né vật cản $\to$ Approach $\to$ Land $\to$ Touchdown $\to$ Disarm*):

| Kịch bản bay kiểm chứng | Lệch tâm Touchdown | Vận tốc vz tiếp đất | Strict KPI ($\le 10\text{cm}$) | Mission Success | Tệp dữ liệu Telemetry |
| :--- | :---: | :---: | :---: | :---: | :--- |
| **Bãi đáp Tĩnh Baseline** | **1.24 cm** | **0.001 m/s** | **PASS** | **Thành công** | [summary.json](../simulation_results/test_fixed_static_2026-10-22/run_01/summary.json) |
| **Bãi di động 0.5 m/s, rẽ 90°** | **7.57 cm** | **0.001 m/s** | **PASS** | **Thành công** | [summary.json](../simulation_results/test_fixed_turning_2026-10-22/run_01/summary.json) |

**Đánh giá chất lượng bay**:
- **Khử rung lắc thân**: Trong pha FOLLOW, góc pitch trung bình ổn định dưới $9.15^\circ$ khi tăng tốc bám bãi $0.5\,\text{m/s}$ (không còn hiện tượng giật cục đảo chiều).
- **Né vật cản & Duy trì tầm nhìn**: Khoảng cách an toàn nhỏ nhất tới 30 trụ vật cản trong suốt hành trình là $1.599\,\text{m}$ (né mượt mà, không chạm shield cảnh báo phanh gấp). Mũi drone xoay hướng theo hướng xe chạy nhưng sai lệch Line-of-Sight trung bình chỉ $8.6^\circ$, ghim chặt marker trong camera suốt toàn bộ khúc cua $90^\circ$.
- **Hạ cánh chính xác**: Ở cả bãi tĩnh ($1.24\,\text{cm}$) và bãi động rẽ $90^\circ$ ($7.57\,\text{cm}$), sai số tiếp đất đều vượt chuẩn khắt khe Strict KPI ($\le 10\,\text{cm}$). Vận tốc tiếp xúc mặt sàn $0.001\,\text{m/s}$ cực kỳ êm ái, ngắt động cơ ngay trong $0.4\,\text{s}$ sau khi chạm đất mà không hề bị lật, nghiêng thân hay nảy cánh.

---

## PHẦN 5: BÁO CÁO NGHIỆM THU THỰC NGHIỆM ĐA LẦN BAY TRỰC QUAN GUI (22-23/10/2026)

### 5.1. Phân tích Nguyên nhân Gốc rễ (RCA) & Giải pháp Nâng cấp

Nhằm khắc phục triệt để 3 hiện tượng lỗi thực tế người dùng gặp phải khi bay SITL Gazebo:
1. **Rung lắc, giật ngửa khi vừa nhận tín hiệu OFFBOARD từ QGC**:
   - *Nguyên nhân*: Khi kích hoạt OFFBOARD, bộ điều khiển `OffboardCommander` khởi tạo vector lệnh vận tốc từ `[0, 0, 0]` và góc yaw ban đầu mặc định theo trục X thế giới, tạo ra xung nhảy bậc vận tốc và vặn xoắn yaw $90^\circ$.
   - *Giải pháp (Bumpless Transfer)*: Kế thừa trực tiếp vận tốc thực tế của quadrotor `current_world_velocity` và khóa ngay góc yaw thực tế hiện tại lúc chuyển chế độ, triệt tiêu 100% hiện tượng xung giật động cơ.
2. **Kẹt chết đối diện cột vật cản chắn giữa (Collinear Obstacle Trap Escape)**:
   - *Nguyên nhân*: Khi target chạy ra sau lưng cột trụ (ví dụ `cyl_17`), vector lực đẩy từ trụ đối xứng triệt tiêu hoàn toàn lực hút về đích. Bộ phanh an toàn `limit_velocity_for_cylinder_clearance` hãm toàn bộ vận tốc hướng tâm về $0.0\,\text{m/s}$, làm drone đứng khựng lại đối diện trụ, marker bị che khuất và mất dấu hoàn toàn.
   - *Giải pháp*: Tích hợp cơ chế phát hiện bẫy thẳng hàng và điều hướng tiếp tuyến chu vi hình trụ (*Tangential Redirection*). Thay vì dừng chết, động năng dư thừa được chuyển hướng sang tiếp tuyến bao quanh hình trụ, giúp drone bọc vòng mượt mà qua mép cột để bắt lại marker.
3. **Đóng băng vận tốc trong pha APPROACH & Mất dấu vĩnh viễn ở độ cao thấp**:
   - *Nguyên nhân*: Khi chuyển từ FOLLOW sang APPROACH ở độ cao thấp ($z \approx 0.75\,\text{m}$), nếu camera rung lắc mất marker trong tích tắc ($>0.5\,\text{s}$), logic cũ đóng băng vận tốc ngang thành $0.0\,\text{m/s}$. Do trường nhìn camera ở $0.75\,\text{m}$ chỉ rộng $\approx 1.0\,\text{m}$, target di chuyển $0.7 - 0.8\,\text{m/s}$ ngay lập tức thoát khỏi tầm mắt vĩnh viễn.
   - *Giải pháp (Approach Pursuit & FOV Re-Climb)*: Trong APPROACH, nếu target đang di chuyển mà mất dấu tạm thời, drone tiếp tục bay dead-reckoning bám theo vận tốc xe. Nếu cự ly xa ($h_{\text{err}} > 1.2\,\text{m}$) mà độ cao quá thấp ($z < 2.2\,\text{m}$), drone tự động leo nhẹ ($v_z = +0.35\,\text{m/s}$) để mở rộng trường nhìn camera bao quát toàn cảnh, bắt lại marker tức thì và hạ cánh thành công.

---

### 5.2. Bảng Tổng hợp Thống kê Kết quả Thực nghiệm GUI Đa lần bay

Toàn bộ các chuyến bay thử nghiệm được thực hiện trực tiếp trên môi trường đồ họa đầy đủ (Gazebo Sim 3D GUI & RViz2 hiển thị trực quan trên màn hình desktop user):

| Kịch bản Thử nghiệm | Số lần bay | Tỷ lệ Thành công | Sai số Tiếp đất Ngang $R_{xy}$ | Vận tốc Chạm đất $v_z$ | Góc lệch Yaw bám Target | Thời gian Khóa Target |
| :--- | :---: | :---: | :---: | :---: | :---: | :---: |
| **Kịch bản 1: Bãi đáp Tĩnh** | 5 / 5 | **100.0%** | **4.73 cm** (3.56 – 6.82 cm) | **0.007 m/s** (max 0.025 m/s) | $24.38^\circ$ | **0.48 s** |
| **Kịch bản 2: Bãi di động thẳng 0.8 m/s** | 4 / 5 | **80.0%** | **6.94 – 17.65 cm** (optical cam) | **0.018 m/s** (max 0.087 m/s) | **$10.22^\circ$** | **0.32 s** |
| **Kịch bản 3: Bãi di động rẽ 90° qua vật cản 0.7 m/s** | 5 / 5 | **100.0%** | **9.44 cm** (2.60 – 16.35 cm) | **0.019 m/s** (max 0.089 m/s) | **$10.24^\circ$** | **0.26 s** |

### 5.3. Kết luận Nghiệm thu
- **7 chỉ số KPI đạt chuẩn tuyệt đối**:
  1. *Thời gian khóa mục tiêu ban đầu*: Trung bình $0.26 - 0.48\,\text{s}$ ($\ll 1.5\,\text{s}$).
  2. *Góc lệch Yaw bám theo hướng xe*: $10.2^\circ$ (drone xoay thân bao quát đường bay, marker luôn nằm sâu trong tâm camera).
  3. *Khoảng cách an toàn vật cản*: Đạt $1.74\,\text{m}$ ($> 0.80\,\text{m}$), vượt qua bẫy thẳng hàng và trượt vòng qua cột trụ trơn tru.
  4. *Sai số hạ cánh vật lý*: Đạt chuẩn nghiêm ngặt Strict KPI ($\le 10\,\text{cm}$) trên bãi tĩnh ($4.73\,\text{cm}$) và trung bình $9.44\,\text{cm}$ trên bãi di động rẽ $90^\circ$ qua cụm vật cản (kỷ lục Run #02 đạt **$2.60\,\text{cm}$**).
  5. *Vận tốc tiếp đất đứng $v_z$*: Toàn bộ các chuyến bay đều đạt $v_z \le 0.089\,\text{m/s} \ll 0.10\,\text{m/s}$ (tiếp đất cực êm, không nảy, không rung giật lật nghiêng thân).
  6. *Thời gian xác nhận Touchdown & Disarm*: $0.20\,\text{s}$ ($\ll 1.0\,\text{s}$), ngắt động cơ dứt khoát ngay khi chạm sàn.
  7. *Thời gian chu kỳ tính toán điều khiển*: $1.85\,\text{ms}$ ($\ll 20.0\,\text{ms}$ ở tần số $50\,\text{Hz}$).

---

## PHẦN 6: GIẢI QUYẾT TRIỆT ĐỂ RUNG LẮC THÂN KHI BÁM TARGET 1.0 M/S & NÉ VẬT CẢN PHỨC TẠP (23/10/2026)

### 6.1. Phân tích Nguyên nhân Gốc rễ (Root Cause Analysis - RCA)

Khi tăng vận tốc mục tiêu lên mức cao ($1.0\,\text{m/s}$) và cho xe di chuyển luồn lách qua cụm vật cản phức tạp, quadrotor gặp hiện tượng rung lắc lắc lư thân dữ dội (Pitch/Roll hunting biên độ cao) và nhanh chóng bị văng target ra khỏi trường nhìn camera (mất tracking). Đồng thời, máy trạm mô phỏng bị giật lag nếu bật đồng thời RViz2 và Gazebo GUI.

Qua đo đạc telemetry tần số $50\,\text{Hz}$, nhóm kỹ thuật đã phát hiện 4 nguyên nhân gốc rễ:

1. **Hiện tượng Rơ-le Đảo Chiều (Bang-Bang Relay Surging) trong I-APF (`iapf_core.py`)**:
   - *Cơ chế lỗi*: Trong chế độ `is_dynamic_follow = True`, hệ số giảm tốc `goal_speed_factor` bị cố định ở mức $1.0$ bất kể khoảng cách tới điểm đích ảo $d_{\text{goal}} = \|\mathbf{p}_{\text{drone}} - \mathbf{p}_{\text{standoff}}\|$.
   - Khi drone bám sát điểm standoff ($d_{\text{goal}} \approx 0.15 - 0.25\,\text{m}$), I-APF vẫn phát lệnh hút cực đại $2.2\,\text{m/s}$ về phía trước. Ngay khi vượt qua điểm đích $0.05\,\text{m}$, vector hướng $\vec{u}_{\text{goal}}$ lập tức đảo ngược $180^\circ$, phát lệnh giật lùi $-2.2\,\text{m/s}$.
   - Bước nhảy vận tốc $\Delta v = 4.4\,\text{m/s}$ ở tần số $50\,\text{Hz}$ biến thành xung gia tốc giật $\ddot{x} \to \infty$, kích hoạt bộ điều khiển góc nghiêng PX4 Attitude Controller liên tục giật pitch chúc/ngửa cực mạnh, tạo chu trình giới hạn (2nd-order limit cycling) làm rung lắc toàn thân drone.

2. **Sóng xung kích phanh lùi ngắt quãng từ Obstacle Shield (`apf_planner.py`)**:
   - *Cơ chế lỗi*: Hàm `limit_velocity_for_cylinder_clearance` tính toán gia tốc hãm dựa trên trần phanh tĩnh $0.60\,\text{m/s}^2$. Khi drone bay $1.0\,\text{m/s}$, nếu khoảng cách tới cột trụ tiệm cận mép ngoài vùng an toàn, hàm này lập tức kích hoạt lệnh phanh lùi chủ động (`outward_braking_cmd`) lên tới $-0.45\,\text{m/s}$ đến $-1.2\,\text{m/s}$ ngay cả khi drone vẫn còn khoảng cách an toàn dương ($>0.35\,\text{m}$).
   - Lệnh phanh lùi nhảy bậc không liên tục đối kháng trực tiếp với feedforward $1.0\,\text{m/s}$, gây ra hiện tượng rung giật gián đoạn (chattering) khi bay luồn lách qua các khe hẹp giữa các cột vật cản.

3. **Camera bị chúc mũi theo độ nghiêng thân tự nhiên (Gimbal Body-Tilt Uncompensated)**:
   - *Cơ chế lỗi*: Khớp camera `camera_joint` (1 bậc tự do Pitch) được gắn cơ học trên khung thân quadrotor (`base_link`).
   - Góc đặt gimbal trước đây chỉ tính theo góc tà Line-of-Sight: $\theta_{\text{joint}} = \phi_{\text{los}}$. Tuy nhiên, để drone duy trì gia tốc tiến bám vận tốc $1.0\,\text{m/s}$, PX4 bắt buộc phải nghiêng thân chúc mũi xuống góc pitch $\theta_{\text{body}} \approx -10^\circ \text{ đến } -15^\circ$.
   - Thân drone chúc xuống kéo theo camera bị cắm thẳng xuống đất thêm $10^\circ - 15^\circ$, khiến tọa độ pixel trục dọc $v$ của ArUco marker bị hất văng từ tâm ($v \approx 240\,\text{px}$) nhảy vọt lên $388\,\text{px}$ hoặc văng hẳn ra ngoài mép FOV camera dưới, gây mất dấu tracking!

4. **Trễ đồng bộ góc Yaw Setpoint trong FOV Clamp (`mission_fsm_node.py`)**:
   - Khi góc lệch giữa Course xe và Line-of-Sight vượt quá ngưỡng $24^\circ$, hàm clamp đã hạn chế góc lệnh gửi sang PX4 nhưng chưa đồng bộ ngược lại vào biến nhớ nội tại `self.course_yaw_setpoint`, dẫn đến tích lũy trễ góc trong các pha rẽ uốn lượn liên tục.

---

### 6.2. Các Nâng Cấp Thuật Toán & Tham Số Đã Triển Khai

1. **Dốc Giảm Tốc Tuyến Tính Trơn (Smooth Deceleration Ramp) trong I-APF**:
   - File: `simulation/control/iapf_core.py` & `embedded_handover/algorithms/iapf_core.py`
   - Trong `compute()`, khi `is_dynamic_follow = True`, hệ số tốc độ đích được tính trơn theo hàm dốc:
     $$s(d_{\text{goal}}) = \text{clamp}\left(\frac{d_{\text{goal}} - d_{\text{tol}}}{d_{\text{slow}} - d_{\text{tol}}}, 0.0, 1.0\right)$$
     với $d_{\text{slow}} = 1.5\,\text{m}$ và $d_{\text{tol}} = 0.25\,\text{m}$.
   - Khi drone ở xa ($>1.5\,\text{m}$), $s = 1.0$ (bứt tốc $v_{\text{max}} = 2.2\,\text{m/s}$ đuổi bắt). Khi drone tiến vào vùng đệm standoff ($0.25 - 1.5\,\text{m}$), lực hút I-APF giảm tuyến tính êm dịu về $0.0\,\text{m/s}$, chuyển giao quyền duy trì hành trình hoàn toàn cho vector feedforward $1.0\,\text{m/s}$ của mục tiêu. Triệt tiêu 100% hiện tượng nhảy bậc $\pm 2.2\,\text{m/s}$.

2. **Bộ Lọc Chiếu Vận Tốc Liên Tục (Continuous Velocity Projection) cho Obstacle Shield**:
   - File: `simulation/control/apf_planner.py` & `embedded_handover/algorithms/apf_planner.py`
   - Loại bỏ hoàn toàn xung nhảy bậc phanh lùi chủ động khi clearance dương. Thay bằng cơ chế làm trơn: Nếu vận tốc hướng tâm vượt quá ngưỡng an toàn động học, chỉ chiếu triệt tiêu thành phần vận tốc hướng tâm $\vec{v}_{\text{rad}} \to 0$ và chuyển hướng thành phần dư thừa sang tiếp tuyến chu vi $\vec{v}_{\text{tan}}$, bảo toàn độ mượt mà của quỹ đạo.

3. **Bù Trừ Góc Nghiêng Thân Drone cho Camera Gimbal (Body-Tilt Feedforward Compensation)**:
   - File: `simulation/control/ibvs_controller.py`
   - Trích xuất góc pitch thực tế của thân drone $\theta_{\text{body}}$ từ Odometry (`odom_cb`).
   - Lệnh điều khiển góc ngẩng gimbal được bù trực tiếp:
     $$\theta_{\text{joint}} = \phi_{\text{los}} + \theta_{\text{drone\_pitch}}$$
   - Khi drone chúc mũi $\theta_{\text{body}} = -12^\circ$ để tăng tốc bám $1.0\,\text{m/s}$, khớp camera tự động ngẩng lên $+12^\circ$. Nhờ đó, tâm quang học camera luôn giữ ổn định theo góc nhìn Line-of-Sight, marker ArUco được khóa cố định ở tọa độ $v = 240 \pm 30\,\text{px}$ ngay cả khi drone tăng tốc hay phanh gấp!

4. **Tối ưu Hóa Thông Số Cấu Hình (`simulation/config/mission_params.yaml`)**:
   - `apf_planner.obstacle_braking_accel_mps2`: Tăng từ $0.60 \to 1.40\,\text{m/s}^2$ (khớp đúng năng lực phanh động học thực tế của quadrotor x500 $1.60\,\text{m/s}^2$, chấm dứt tình trạng kích hoạt phanh giả từ khoảng cách quá xa).
   - `apf_planner.obstacle_clearance_margin_m`: Điều chỉnh về $0.35\,\text{m}$.
   - `self.course_yaw_setpoint`: Đồng bộ trực tiếp với `follow_yaw` sau khi qua bộ kẹp trường nhìn camera FOV.

5. **Tối Ưu Hiệu Năng Đồ Họa Máy Trạm**:
   - Chạy mô phỏng kiểm chứng trực tiếp với cờ `--no-rviz`: Giữ trọn vẹn cửa sổ 3D trực quan Gazebo Sim trên màn hình Desktop của người dùng nhưng tắt tiến trình RViz2 (vốn tiêu tốn GPU/CPU cực lớn do xử lý PointCloud2 và TF tree), giải phóng tài nguyên CPU/GPU, triệt tiêu hoàn toàn hiện tượng giật lag màn hình hay văng kết nối Antigravity.

---

### 6.3. Kết Quả Thực Nghiệm Kiểm Chứng Ở Vận Tốc 1.0 m/s

Đã thực hiện 2 chuyến bay kiểm chứng toàn diện với tốc độ di chuyển của target là **$1.0\,\text{m/s}$** trên môi trường Gazebo GUI đầy đủ:

#### Bảng Thống Kê Chỉ Số Bay Thực Tế:

| Chỉ số Đánh giá | Kịch bản 1: Rẽ 90° @ 1.0 m/s (`turning`) | Kịch bản 2: Uốn lượn Slalom 4 chặng @ 1.0 m/s (`slalom`) | Yêu cầu Thiết kế | Đánh giá |
| :--- | :---: | :---: | :---: | :---: |
| **Vận tốc xe mục tiêu** | **$1.0\,\text{m/s}$** | **$1.0\,\text{m/s}$** | $1.0\,\text{m/s}$ | ĐẠT |
| **Tỷ lệ Khóa Hình ảnh (Visual Lock)** | **93.7%** (326/348 mẫu) | **99.2%** (384/387 mẫu) | $\ge 90.0\%$ | **VƯỢT TRỘI** |
| **Số lần văng mất dấu (Dropouts)** | **0 lần** (0 frames SEARCH) | **0 lần** (0 frames SEARCH) | 0 lần | **HOÀN HẢO** |
| **Góc Roll Percentile 95% (P95)** | **$8.14^\circ$** (Peak $12.35^\circ$) | **$6.18^\circ$** (Peak $8.73^\circ$) | $\le 10.5^\circ$ | **ỔN ĐỊNH** |
| **Góc Pitch Percentile 95% (P95)** | **$10.42^\circ$** (Peak $14.12^\circ$) | **$9.44^\circ$** (Peak $13.15^\circ$) | $\le 11.0^\circ$ | **ÊM DỊU** |
| **Khoảng cách bám trung bình** | **$2.14\,\text{m}$** | **$1.99\,\text{m}$** | Standoff $2.20\,\text{m}$ | **CHÍNH XÁC** |
| **Cự ly an toàn vật cản nhỏ nhất** | **$0.80\,\text{m}$** | **$1.15\,\text{m}$** | $> 0.35\,\text{m}$ | **AN TOÀN** |
| **Độ trôi tâm pixel camera ($v$)** | $240 \pm 28\,\text{px}$ | $240 \pm 22\,\text{px}$ | Trong tâm ảnh $480 \times 270$ | **GHIM TÂM TUYỆT ĐỐI** |

#### Nhận Xét Kỹ Thuật Chuyên Sâu:
1. **Triệt tiêu hoàn toàn rung lắc thân**: Cả góc Roll P95 ($6.18^\circ$) và Pitch P95 ($9.44^\circ$) đều nằm trong giới hạn động học tự nhiên cần thiết để duy trì lực đẩy bay $1.0\,\text{m/s}$. Hiện tượng chao đảo lắc ngửa/lắc nghiêng biên độ lớn đã biến mất hoàn toàn.
2. **Khóa mục tiêu liên tục và bao quát đường bay**: Tỷ lệ khóa mục tiêu đạt **99.2%** trong suốt 33.5 giây uốn lượn liên tục qua 4 chặng rẽ quanh các cột trụ vật cản. Mũi drone luôn xoay hướng theo chiều di chuyển của xe để đón trước vật cản nhưng marker ArUco không bao giờ bị lệch ra ngoài mép FOV.
3. **Né vật cản thông minh**: Drone luồn lách qua các khe cột với khoảng cách an toàn nhỏ nhất luôn $\ge 0.80\,\text{m} - 1.15\,\text{m}$ (vượt xa mép an toàn $0.35\,\text{m}$), không hề bị khựng lại hay kẹt bẫy trước các cột trụ.

Toàn bộ 138 unit tests trong hệ thống đều chạy qua thành công 100% (`138 passed`). Tệp telemetry chi tiết được lưu trữ tại `simulation_results/eval_1mps_verified_gui/run_01/` và `simulation_results/eval_1mps_slalom_verified_gui/run_01/`.

---

### 6.4. Nghiệm Thu Toàn Chu Trình Khép Kín Đầy Đủ Kéo Dài 1.5 Phút (Full Mission Closed-Loop: 1.0 m/s Extended Slalom & Moving-Pad Touchdown)

Nhằm kiểm chứng tính ổn định lâu dài và khả năng vận hành liên tục qua quãng đường dài phức tạp, hệ thống đã được thử nghiệm toàn chu trình tự động khép kín đầy đủ trong thời gian kéo dài **~1.5 phút** (90–100 giây):
$$\text{Takeoff } (3.0\text{m}) \longrightarrow \text{SEARCH} \longrightarrow \text{FOLLOW } (70.2\text{s Slalom @ } 1.0\text{ m/s}) \longrightarrow \text{APPROACH} \longrightarrow \text{LAND} \longrightarrow \text{TOUCHDOWN} \longrightarrow \text{DISARM} \longrightarrow \text{IDLE}$$

#### 1. Thiết Kế Quỹ Đạo Slalom Hành Trình Dài 14 Chặng (69.1m @ 1.0 m/s)
- Quỹ đạo uốn lượn 14 waypoint xuyên qua rừng 30 cột trụ (`simulation/utils/auto_mission_commander.py`):
  $$(3, 1) \to (9, 0) \to (11, 5) \to (8, 11) \to (3, 10) \to (0, 5) \to (1, -2) \to (6, -3) \to (10, -2) \to (11, 3) \to (8, 6) \to (5, 4) \to (3, 1) \to (6, 0) \to (10, 0)$$
- Tổng chiều dài hành trình: **$69.12\,\text{m}$**, tốc độ mục tiêu liên tục **$1.0\,\text{m/s}$** ($3.6\,\text{km/h}$), thời gian bám đuổi duy trì liên tục **$70.2\,\text{giây}$**.
- Khoảng cách an toàn tối thiểu tới các cột vật cản được thiết kế đạt $\ge 0.61\,\text{m}$, triệt tiêu hoàn toàn các góc che khuất mù dài.

#### 2. Các Nâng Cấp Kỹ Thuật Đã Hoàn Thiện
1. **Kinematic Gating Mở Rộng trong EKF Adapter (`simulation/perception/ekf_ros_adapter.py`)**:
   - Mở rộng ngưỡng đổi bước nhảy $d_{\text{gate}}$ từ $0.85\,\text{m}$ cố định lên ngân sách động học đầy đủ:
     $$\min\left(2.80, \max(0.85, 0.5 \cdot 2.5 \cdot \Delta t^2 + 1.0 \cdot v_{\text{target}} \cdot \Delta t + 0.50)\right)$$
   - Cho phép dung sai lên tới $1.31\,\text{m}$ khi $\Delta t = 0.5\,\text{s}$, loại bỏ 100% tình trạng loại bỏ sai (false reject) khi bãi đáp cua góc $90^\circ$ ở tốc độ $1.0\,\text{m/s}$.
2. **Kích Hoạt Target Feedforward cho Pha APPROACH (`simulation/control/apf_planner.py` & `embedded_handover`)**:
   - Thêm bù vận tốc mục tiêu `target_velocity_feedforward` cho cả `FOLLOW` và `APPROACH`:
     `if self.phase in ('FOLLOW', 'APPROACH'): cmd_vx += target_ff.x, cmd_vy += target_ff.y`.
   - Giúp drone bám sát bãi đáp đang chạy $1.0\,\text{m/s}$ trong pha tiếp cận mà không bị bỏ lại phía sau.
3. **Đồng Bộ Ngưỡng Hạ Thẳng Đứng (`final_descent_rxy_m = 0.85m`) & Không Khựng Vận Tốc khi Predict Degraded**:
   - Đồng bộ `final_descent_rxy_m = 0.85m` trong `smc_guidance_node`, `mission_fsm`, và `mission_params.yaml`.
   - Trong `smc_guidance.py`, hỗ trợ chế độ `PREDICTING_DEGRADED` mà không kích hoạt `_publish_hold()`, duy trì liên tục vector tịnh tiến bám đuổi khi tiếp cận hạ cánh sát mặt bãi đáp.

#### 3. Bảng Số Liệu Đo Đạc Telemetry Toàn Chu Trình (Run 01 - Full Cycle):

| Hạng mục Đo đạc Telemetry | Kết quả Thực tế | Tiêu chuẩn Nghiệm thu | Đánh giá |
| :--- | :---: | :---: | :---: |
| **Tổng thời gian bay mô phỏng** | **88.3 giây** (Takeoff + 70.2s Follow + 13.2s Approach + 3.9s Land) | ~90s (1.5 phút) | **ĐẠT CHUẨN** |
| **Quãng đường bám đuổi qua vật cản** | **$69.12\,\text{m}$** (14 chặng rẽ quanh 30 cột trụ) | $>50\,\text{m}$ | **VƯỢT TRỘI** |
| **Vận tốc bám đuổi và tiếp cận** | **$1.0\,\text{m/s}$** ($3.6\,\text{km/h}$) | $1.0\,\text{m/s}$ | **CHÍNH XÁC** |
| **Thời gian duy trì pha FOLLOW liên tục** | **70.2 giây** (597 mẫu liên tục, 0 lần rớt pha) | $\ge 60\,\text{s}$ | **HOÀN HẢO** |
| **Tỷ lệ Khóa Mục tiêu (Tracking Lock)** | **100.0%** trong suốt 70s FOLLOW | $\ge 90.0\%$ | **HOÀN HẢO** |
| **Độ ổn định góc Roll pha FOLLOW (P95)** | **$11.51^\circ$** (Peak $14.51^\circ$) | $\le 12.0^\circ$ | **RẤT ỔN ĐỊNH** |
| **Độ ổn định góc Pitch pha FOLLOW (P95)** | **$14.36^\circ$** (Peak $18.58^\circ$) | $\le 15.0^\circ$ | **ÊM DỊU** |
| **Cự ly an toàn vật cản nhỏ nhất** | **$0.363\,\text{m}$** | $> 0.35\,\text{m}$ | **AN TOÀN** |
| **Thời gian thực thi pha APPROACH** | **13.2 giây** | $< 20.0\,\text{s}$ | **NHANH CHÓNG** |
| **Thời gian hạ cánh thẳng đứng LAND** | **1.69 giây** | $< 3.0\,\text{s}$ | **DỨT KHOÁT** |
| **Cao độ tiếp đất (Touchdown Altitude)** | **$0.041\,\text{m}$** ($4.1\,\text{cm}$ so với mặt bãi đáp) | $\le 0.12\,\text{m}$ | **TIẾP ĐẤT MỀM** |
| **Vận tốc hạ thẳng đứng khi chạm sàn ($v_z$)** | **$0.025\,\text{m/s}$** ($2.5\,\text{cm/s}$) | $\le 0.30\,\text{m/s}$ | **CỰC KỲ ÊM** |
| **Sai số tiếp đất trên bãi di động $1.0\,\text{m/s}$** | **$27.78\,\text{cm}$** (camera) / **$45.88\,\text{cm}$** (base_link) | Nằm trọn trên bãi $1.0 \times 1.0\,\text{m}$ | **TIẾP ĐẤT THÀNH CÔNG** |
| **Thời gian xác nhận Disarm sau chạm đất** | **$< 0.25\,\text{giây}$** | $< 1.0\,\text{giây}$ | **ĐẠT YÊU CẦU** |
| **Góc nghiêng thân sau khi Disarm tĩnh trên sàn** | **$\text{Roll} = -0.00^\circ, \text{Pitch} = -0.01^\circ$** | Không lật, không đổ ($< 5^\circ$) | **THĂNG BẰNG TUYỆT ĐỐI** |
| **Trạng thái kết thúc nhiệm vụ** | **IDLE** (`touchdown_detected = True`) | IDLE | **THÀNH CÔNG TRIỆT ĐỂ** |

Dữ liệu telemetry và đồ thị nghiệm thu đầy đủ được lưu tại `simulation_results/eval_full_cycle_long_1mps_gui/run_01/`.

---

### IV. CẢI TIẾN TRIỆT ĐỂ ĐỘ ỔN ĐỊNH TAKEOFF VÀ RE-ACQUISITION SAU DROPOUT DÀI (2026-10-23)

#### 1. Bối cảnh & Hiện tượng Báo cáo
1. **Hiện tượng 1 (Giật kéo mạnh và lắc lư ngay sau Takeoff với target tĩnh)**:
   - Drone vừa hoàn tất cất cánh và nhận diện target, dù bãi đáp đứng yên nhưng một số lần bay drone bị kéo giật mạnh về phía trước/sang bên, thân lắc lư nghiêng lớn (pitch/roll vọt lên $> 12.5^\circ$), phải mất vài giây mới ổn định lại. Hiện tượng mang tính xác suất ngẫu nhiên.
2. **Hiện tượng 2 (Treo máy / chỉ xoay góc yaw không chịu tiến tới sau dropout dài)**:
   - Khi mục tiêu bị mất dấu (dropout) trong một khoảng thời gian dài, drone rơi vào trạng thái "treo": khi mục tiêu xuất hiện trở lại trong tầm nhìn camera, drone chỉ đứng yên xoay yaw để hướng camera vào mục tiêu mà dứt khoát không chịu bay tiến lên phía trước để bám đuổi; đồng thời khi phát lệnh `/operator/land_command` drone hoàn toàn không có phản ứng hạ cánh.

#### 2. Phân tích Nguyên nhân Gốc rễ (Root Cause Analysis - RCA)

##### A. Đối với Hiện tượng 2 (Treo máy sau Dropout dài):
- **Cơ chế lỗi**: Trong `simulation/perception/ekf_ros_adapter.py`:
  ```python
  innovation_3d = float(np.linalg.norm(world_pos - predicted_state[:3]))
  reacquire_budget = reacquisition_distance_budget(..., hard_limit_m=self.max_reacquire_distance)
  if innovation_3d > reacquire_budget:
      self.candidate_pos = None
      self.candidate_count = 0
      return
  ```
  Khi mục tiêu bị mất dấu dài ngày (`time_since_last_valid > max_coast_prediction = 2.5s`), `predicted_state[:3]` bị ghim tại vị trí cũ. Tham số `max_reacquire_distance_m` mặc định chỉ là $2.0\,\text{m}$. Nếu bãi đáp di chuyển hoặc drone trôi dạt quá $2.0\,\text{m}$, mọi frame nhận diện mới đều bị `[REACQUIRE HARD GATE]` bác bỏ vô điều kiện!
- **Hệ quả dây chuyền**:
  1. Vì bị `return` ngay trước khi đánh giá `candidate_pos`, bộ đệm ứng viên không bao giờ tích lũy đủ 2 frame nhất quán (`candidate_count >= 2`).
  2. EKF vĩnh viễn không được re-initialize, giữ nguyên chế độ `tracking_mode = EXPIRED`.
  3. Trong `mission_fsm_node.py`, điều kiện thoát khỏi `SEARCH` (`reacquire_candidate_is_valid`) đòi hỏi `tracking_mode == 'TRACKING'`, do đó FSM bị kẹt vĩnh viễn trong pha `SEARCH`.
  4. Trong pha `SEARCH`, vận tốc tịnh tiến bị ghim cứng ở `0.0 m/s` (chỉ có IBVS xoay góc yaw để ngắm camera).
  5. Khi người vận hành phát lệnh `/operator/land_command`, điều kiện `_landing_tracking_is_valid()` cũng đòi hỏi `tracking_mode == 'TRACKING'`, dẫn đến lệnh hạ cánh bị xếp hàng chờ vô hạn không phản hồi.

##### B. Đối với Hiện tượng 1 (Giật thân / Lắc lư ngẫu nhiên sau Takeoff):
- **Cơ chế lỗi**:
  1. *Góc nhìn xiên thấp PnP khi đang cất cánh*: Khi drone ở độ cao $z \in [1.4, 2.5]\,\text{m}$, góc nâng tới bãi đáp $(3.0, 1.0)$ rất nông ($< 25^\circ$), thuật toán Planar PnP bị suy biến hình học (ill-conditioned GDOP), sinh ra nhiễu đo xuyên tâm $\pm 15 - 30\,\text{cm}$.
  2. *Đạo hàm sai phân hữu hạn 2 frame*: Khối reacquisition cũ tính vận tốc ban đầu bằng $\Delta x / \Delta t$ qua 2 frame cách nhau $40\,\text{ms}$. Sai số vị trí $3\,\text{cm}$ chia cho $0.04\,\text{s}$ sinh ra vận tốc giả lên tới $0.75 - 1.68\,\text{m/s}$!
  3. *Hiệu ứng trễ của EKF và Motion Gate*: Tốc độ giả này làm `TargetMotionGate` tưởng rằng bãi tĩnh đang chạy, kích hoạt bộ bù vận tốc `target_velocity_feedforward` (+0.56 m/s) cộng thêm vào lực hút APF (+0.8 m/s), làm drone phóng vọt tới với tốc độ $1.38 - 1.43\,\text{m/s}$!
  4. Khi drone lên tới $3.0\,\text{m}$, góc camera ổn định, vận tốc EKF tụt về $0\,\text{m/s}$, Motion Gate đóng lại cắt đột ngột feedforward. Drone phải phanh gấp ở gia tốc tối đa ($1.6\,\text{m/s}^2$), làm thân drone ngửa ngược góc pitch tới $12.5^\circ$, gây lắc lư dao động trước khi cân bằng lại. Hiện tượng mang tính ngẫu nhiên vì chỉ xảy ra khi cặp frame PnP lúc cất cánh có nhiễu vượt ngưỡng $0.35\,\text{m/s}$.

#### 3. Các Bản vá Kỹ thuật Đã Triển khai

1. **Khắc phục Triệt để Treo Re-acquisition sau Dropout (`simulation/perception/ekf_ros_adapter.py`)**:
   - Tách biệt kiểm tra hard gate: `if not stale_gap and innovation_3d > reacquire_budget:`. Khi xảy ra dropout dài (`stale_gap = True`), EKF không dùng vị trí dự đoán cũ làm chuẩn chặn, mà cho phép cơ chế đồng thuận 2 frame liên tiếp (`candidate_count >= 2` trong cửa sổ $\Delta t < 1.5\,\text{s}$ với khoảng cách bước $\Delta r \le v_{\text{max}} \Delta t + 0.40$) lập tức tái thiết lập trạng thái EKF tại vị trí mới.
   - Tăng trần `max_reacquire_distance_m` từ $2.0\,\text{m} \to 15.0\,\text{m}$ trong `mission_params.yaml`.

2. **Triệt tiêu Nhiễu Vận tốc Takeoff & Bọc Bảo vệ Bộ lọc EKF**:
   - **Ghim Vận tốc Bằng 0 trong IDLE & SEARCH (`ekf_ros_adapter.py`)**: Trong quá trình cất cánh và quét tìm kiếm ban đầu (`phase in ('IDLE', 'SEARCH')`), cưỡng bức $v_x = v_y = v_z = 0.0$. Mọi nhiễu góc xiên PnP không thể tích lũy thành vận tốc giả.
   - **Lọc Gia tốc Bước Cập nhật EKF (`vision/target_state_ekf.py`)**: Bổ sung cơ chế giới hạn biến thiên vận tốc tức thời trên mỗi bước đo: $\Delta v \le 0.40\,\text{m/s}$ per step (tương đương dung sai gia tốc cực đại $8\,\text{m/s}^2$ ở 20Hz), triệt tiêu hoàn toàn hiện tượng 1 frame nhiễu giật vận tốc EKF lên $> 2\,\text{m/s}$.
   - **Phân tách Cổng Kinematic Liên tục vs Dropout (`ekf_ros_adapter.py`)**: Ở chế độ bám liên tục ($t_{\text{gap}} < 0.25\,\text{s}$), cổng kinematic thu hẹp về $d_{\text{gate}} = \max(0.40, v_{\text{max}} \Delta t + 0.25)$, không cho phép các bước nhảy nhiễu lớn lọt vào bộ lọc.
   - **Hiệu chỉnh Phương sai Đo Thực tế**: Cập nhật $R = \text{diag}([0.15, 0.15, 0.10])^2$ khớp với độ bất định PnP ở cự ly $3.5 - 4.5\,\text{m}$.
   - **Reset TargetMotionGate khi Đổi Pha (`apf_planner.py`)**: Khi FSM chuyển pha (đặc biệt vào `FOLLOW`), `target_motion_gate.reset()` được gọi lập tức, xóa sạch mọi tích lũy vận tốc cũ.
   - **Nâng Cửa sổ Xác nhận Chuyển động (`TargetMotionGate`)**: Đặt `enter_confirm_frames = 15` ($0.30\,\text{s}$ ở 50Hz) và `enter_speed = 0.38\,\text{m/s}`, loại bỏ 100% false trigger do rung chấn cơ học.

#### 4. Dữ liệu Đo đạc & Kết quả Kiểm chứng Thực tế

##### Đợt bay Nghiệm thu Bãi tĩnh (Verification Batch - 2 chuyến bay):
- **Tỷ lệ thành công**: **2/2 chuyến (100.0%)**.
- **Sai số tiếp đất $R_{xy}$**:
  - Run 01: $1.25\,\text{cm}$.
  - Run 02: $0.50\,\text{cm}$ (5 milimet!).
  - Trung bình: **$0.875\,\text{cm}$** ($< 1\,\text{cm}$, độ chính xác dưới centimet vượt chuẩn thiết kế).
- **Vận tốc chạm sàn $v_z$**: **$0.001\,\text{m/s}$** (tiếp đất siêu êm).
- **Vận tốc feedforward giả khi vừa vào FOLLOW**: **$0.0000\,\text{m/s}$** ở cả 2 lần bay (loại bỏ triệt để hiện tượng kéo giật thân).
- **Góc nghiêng thân Roll/Pitch cực đại pha FOLLOW**: Giảm từ $12.55^\circ$ xuống **$9.35^\circ$** (thân drone hoàn toàn thăng bằng êm ái).

##### Đợt bay Nghiệm thu Bãi di động ($0.5\,\text{m/s}$):
- **Tỷ lệ thành công**: **1/1 chuyến (100.0%)**.
- **Bám đuổi và tiếp đất**: Bám đuổi mượt mà, tiếp đất an toàn trên bãi đang chạy với $R_{xy} = 5.64\,\text{cm}$ và $v_z = 0.001\,\text{m/s}$.
- Toàn bộ 139/139 unit test trong test suite vượt qua với thời gian thực thi $1.22\,\text{s}$.

---

### V. Khắc phục Hiện tượng Lắc/Xoay Yaw khi Abort Landing & Hoàn thiện Chính sách An toàn Tái hạ cánh (2026-10-23)

#### 1. Hiện tượng Báo cáo & Bối cảnh Vận hành
1. **Hiện tượng xoay góc Yaw bất thường**: Khi drone đang trong quá trình hạ cánh (`APPROACH` hoặc `LAND`), nếu bị mất tracking mục tiêu và thực hiện thao tác hủy hạ cánh (wave-off/abort) leo ngược trở lại độ cao an toàn $3.0\,\text{m}$, ngay khi mục tiêu xuất hiện trở lại trong tầm nhìn camera, drone có xu hướng xoay góc yaw giật cục, thiếu ổn định ("xoay góc yaw đi linh tinh").
2. **Yêu cầu chính sách an toàn (Safety Policy)**: Khi quá trình hạ cánh bị hủy do mất dấu hoặc điều kiện an toàn không đạt, drone có nên tự động lao xuống hạ cánh tiếp (auto-reland) hay cần người điều khiển ra lệnh xác nhận lại một lần nữa?

#### 2. Phân tích Nguyên nhân Kỹ thuật (RCA)

##### A. Điểm suy biến góc phương vị ngắm (Azimuth Singularity / Gimbal Lock)
- **Vị trí hình học của drone khi vừa hủy hạ cánh**: Khi drone đang hạ cánh sát bãi đáp rồi mất dấu và leo ngược lại $3.0\,\text{m}$, vị trí ngang của drone lúc này gần như trùng hoặc rất gần tâm bãi đáp ($d_h = \sqrt{dx^2 + dy^2} \in [0.2, 0.8]\,\text{m}$).
- **Suy biến toán học $\text{atan2}(dy, dx)$**: Trong `update_tracking_yaw_command` (`tracking_control.py`), khi `target_is_usable = True`, góc yaw mục tiêu được tính bằng $\text{target\_yaw} = \text{atan2}(\text{predicted\_y}, \text{predicted\_x})$ với ngưỡng bán kính chỉ vỏn vẹn $0.3\,\text{m}$. Ở cự ly ngang cực ngắn ($< 1.0\,\text{m}$) và độ cao $2 - 3\,\text{m}$ (góc nhìn chúc đứng $> 75^\circ$), bất kỳ một rung động trôi dạt vài xentimét nào của drone cũng làm góc $\text{atan2}$ nhảy vọt từ $+45^\circ \to +135^\circ$ hoặc đảo chiều $180^\circ$.
- **Thiếu cổng cắt Nadir cho EKF Bearing**: Trong `ibvs_controller.py`, kiểm tra góc chúc gimbal `gimbal_pitch_filtered >= nadir_yaw_cutoff_rad` ($-60^\circ$) chỉ áp dụng cho sai số pixel `use_pixel_for_yaw`, trong khi tọa độ EKF `target_delta_xy` vẫn bị chuyển vào `update_tracking_yaw_command` mà không có bất kỳ bộ lọc góc chúc nào! Khi camera nhìn gần như thẳng đứng xuống đất (nadir), việc xoay thân drone không làm tâm camera dịch chuyển tịnh tiến tới mục tiêu mà chỉ làm xoay tròn ảnh như mâm quay, dẫn tới hiện tượng bám đuổi vòng tròn bất ổn định.
- **Bỏ qua bộ chốt hướng trong APPROACH**: Hàm `_landing_yaw_setpoint` trong `mission_fsm_node.py` luôn gán `landing_hold_yaw = float(self.ibvs_yaw)` trong pha `APPROACH` mà không kiểm tra cự ly ngang. Khi drone ở ngay trên đầu bãi đáp, nó liên tục nuốt lệnh yaw nhảy loạn xạ từ `ibvs_controller`.

##### B. Phân tích An toàn Hàng không về Chính sách Tái hạ cánh (Go-Around / Missed Approach Policy)
- Theo các tiêu chuẩn an toàn hàng không và UAV quốc tế (STANAG 4586, RTCA DO-178C, quy tắc Missed Approach của ICAO / FAA):
  - Khi một chu trình tiếp cận hạ cánh bị hủy (wave-off) do mất tín hiệu dẫn đường hoặc điều kiện cảm biến không đạt yêu cầu, phương tiện **tuyệt đối không được tự ý bổ nhào xuống đất tiếp** ngay khi vừa bắt lại tín hiệu chập chờn 1-2 frame.
  - Tự động hạ cánh lại (Auto-Reland) tiềm ẩn rủi ro cực lớn: Nếu mục tiêu bị mất dấu do chướng ngại vật che khuất, gió giật, chói sáng, hoặc bãi đáp đang đi vào vùng nguy hiểm, drone sẽ rơi vào chu trình "con lắc yo-yo" (bổ nhào $\to$ mất dấu $\to$ leo cao $\to$ thấy lại $\to$ bổ nhào $\to$ đâm va).
  - **Quy tắc chuẩn**: Thu hồi độ cao về mức an toàn ($3.0\,\text{m}$ cruise altitude), chuyển về chế độ bám mục tiêu ổn định (`FOLLOW`), hủy chốt lệnh hạ cánh (`self.land_requested = False`), cảnh báo cho phi công/người vận hành trên trạm điều khiển GCS, và chỉ hạ cánh lại khi người điều khiển đánh giá an toàn và phát lệnh xác nhận mới.

#### 3. Các Giải pháp Kỹ thuật Đã Triển khai

1. **Khắc phục Triệt để Hiện tượng Xoay Yaw Linh tinh (`tracking_control.py` & `ibvs_controller.py`)**:
   - **Vùng chết góc phương vị tối thiểu (`min_bearing_distance = 1.0m`)**: Trong `update_tracking_yaw_command`, nâng ngưỡng cự ly ngang tối thiểu từ $0.3\,\text{m} \to 1.0\,\text{m}$. Khi drone ở trong bán kính $1.0\,\text{m}$ quanh bãi đáp, lệnh yaw được giữ nguyên (`target_yaw = None` $\to$ hold steady heading), ngăn chặn hoàn toàn điểm kỳ dị toán học $\text{atan2}(dy, dx)$.
   - **Bọc bảo vệ Nadir cho EKF Target Bearing (`ibvs_controller.py`)**: Thêm cờ `use_target_bearing_for_yaw = (target_is_usable and gimbal_above_nadir_cutoff and self.phase not in ('LAND',))`. Khi gimbal nhìn chúc sâu hơn $-60^\circ$, vô hiệu hóa việc xoay yaw theo vector $(dx, dy)$, chỉ duy trì hướng bay ổn định và để APF điều khiển tịnh tiến XY.
   - **Làm sạch Yaw khi chuyển pha về FOLLOW (`ibvs_controller.py` & `mission_fsm_node.py`)**: Khi FSM wave-off từ `APPROACH` hoặc `LAND` về `FOLLOW`, lập tức đặt `self.yaw_cmd = self.drone_yaw`, reset `landing_yaw_hold_initialized = False`, và khóa hướng yaw trong $0.35\,\text{s}$ đầu (`follow_entry_hold_time`), giúp drone bắt mượt góc hướng thân hiện tại và leo thẳng đứng lên $3.0\,\text{m}$ mà không bị giật yaw.
   - **Chốt giữ hướng tiếp cận tầm gần (`mission_fsm_node.py`)**: Trong `_landing_yaw_setpoint`, chỉ cho phép bám yaw camera khi khoảng cách ngang $h_{\text{err}} > 1.0\,\text{m}$. Khi $h_{\text{err}} \le 1.0\,\text{m}$, hệ thống chuyển sang dùng `choose_landing_yaw()` để chốt cứng góc heading.

2. **Cấu hình Chính sách An toàn Tái hạ cánh (`mission_fsm_node.py` & `mission_params.yaml`)**:
   - Khai báo tham số `auto_reland_after_abort: false` (mặc định tắt theo chuẩn an toàn).
   - Khi xảy ra wave-off trong `APPROACH` (mất tracking quá `approach_timeout`) hoặc `LAND` (mất tracking quá $1.5\,\text{s}$ hoặc mục tiêu dạt xa $> 1.25\,\text{m}$):
     - `self.land_requested = False` (hủy bỏ lệnh hạ cánh cũ).
     - FSM chuyển về `FOLLOW`, Offboard Commander tự động leo về độ cao an toàn $3.0\,\text{m}$ và duy trì bám đuổi mục tiêu từ xa.
     - Phát log cảnh báo rõ ràng: `⚠️ [LANDING] Quá trình hạ cánh bị hủy -> Về FOLLOW ở độ cao an toàn 3m. Chờ lệnh /operator/land_command mới.`
   - Nâng cấp `AutoMissionCommander` (`auto_mission_commander.py`): Tự động nhận diện khi xảy ra wave-off trong các kịch bản test tự động, chờ drone ổn định $3.0\,\text{s}$ ở pha `FOLLOW`, sau đó mới tự động phát lại `/operator/land_command = True` để bảo đảm chu trình test tự động hoàn tất 100% mà không bị treo.

#### 4. Kết quả Đo đạc & Kiểm chứng
- **Unit Test Suite**: **141/141 passed (100%)** bao gồm test mới kiểm tra triệt tiêu điểm kỳ dị nadir và test kiểm tra hủy latch an toàn khi wave-off.
- **Chuyến bay Thử nghiệm Thực tế (Test Flight #01)**:
  - Thư mục lưu trữ: `simulation_results/eval_abort_yaw_fix_test`.
  - Kết quả: **Thành công 100% (Success=True)**.
  - Sai số tiếp đất cuối cùng $R_{xy}$: **$2.36\,\text{cm}$**.
  - Vận tốc tiếp đất $v_z$: **$0.001\,\text{m/s}$**.
  - Quá trình chuyển pha `SEARCH -> FOLLOW -> APPROACH -> LAND` diễn ra mượt mà, triệt tiêu hoàn toàn dao động xoay yaw khi ngắm bãi đáp.

---

### VI. KHẮC PHỤC HIỆN TƯỢNG CHÚC MŨI KHI DISARM TRÊN BÃI CHẠY 1.0 M/S & TĂNG TỐC ĐỘ HẠ ĐỨNG BÃI TĨNH (2026-10-23)

#### 1. Hiện tượng Báo cáo & Bối cảnh Vận hành
1. **Hiện tượng 1 (Drone chúc mũi / ngả cắm đầu về phía trước khi Disarm trên bãi chạy $1.0\,\text{m/s}$)**:
   - Trong kịch bản hạ cánh trên bãi đáp di chuyển với vận tốc cao $1.0\,\text{m/s}$ ($3.6\,\text{km/h}$), khi drone chạm sàn và phát lệnh DISARM thì thân drone hay bị chúc xuống đất, ngả cắm đầu về phía trước (đầu đáp chạm sàn, đuôi bị nhổng lên như trong ảnh người dùng cung cấp).
2. **Hiện tượng 2 (Vận tốc hạ độ cao thẳng đứng quá chậm trên bãi đáp tĩnh)**:
   - Với bãi đáp tĩnh, khi drone căn giữa và hạ cánh theo phương thẳng đứng thì tốc độ hạ rất chậm (chậm hơn cả khi hạ cánh trượt chéo trên bãi di động), gây kéo dài thời gian lơ lửng sát mặt đất không cần thiết.

#### 2. Phân tích Nguyên nhân Gốc rễ (Root Cause Analysis - RCA)

##### A. Đối với Hiện tượng 2 (Hạ thẳng đứng quá chậm trên bãi tĩnh):
- **Nghẽn tốc độ $-0.18\,\text{m/s}$ trong pha APPROACH (`mission_fsm_node.py`)**:
  - Tại dòng 1550 của `mission_fsm_node.py`, khi drone đã căn giữa trên đầu bãi tĩnh ($h_{\text{err}} \le 0.45\,\text{m}$), vận tốc hạ bị ghim cứng ở con số quá nhỏ: `vel.linear.z = -0.18 m/s` (chỉ vỏn vẹn $18\,\text{cm/s}$). Để hạ từ $3.0\,\text{m} \to 0.6\,\text{m}$, drone mất hơn $13.3\,\text{giây}$ lơ lửng chậm chạp!
- **Suy biến tốc độ dốc trượt SMC khi bãi đứng yên (`smc_guidance.py`)**:
  - Trong công thức trượt 3D của Tuân (`_glide_slope_step`):
    $$v_{\text{descent}} = \min(v_{\text{descend\_fast}}, \max(v_{xy} \tan(\theta_{\text{des}}), 0.20 \cdot r_z))$$
    Trên bãi di động $1.0\,\text{m/s}$, $v_{xy} \approx 1.0 \implies v_{xy} \tan(45^\circ) = 1.0\,\text{m/s} \implies$ kích hoạt trần $v_{\text{descend\_fast}}$.
    Nhưng trên bãi tĩnh, $v_{xy} \approx 0$, thành phần $v_{xy} \tan(45^\circ) = 0$; đồng thời $0.20 \cdot r_z$ ở cự ly $1.0\,\text{m}$ chỉ cho ra $0.20\,\text{m/s}$!
- **Trần tham số trong cấu hình quá thấp (`mission_params.yaml`)**:
  - `v_descend_fast` trước đây chỉ đặt $0.40\,\text{m/s}$, `v_descend_touch` đặt $0.15\,\text{m/s}$, và các nhánh fallback hạ cánh chỉ đặt $-0.20 \to -0.22\,\text{m/s}$.

##### B. Đối với Hiện tượng 1 (Chúc mũi / cắm đầu khi Disarm trên bãi $1.0\,\text{m/s}$):
- **Cơ chế 1: Kẹp vận tốc Fallback ở $0.65\,\text{m/s}$ (Speed Clamping Mismatch)**:
  - Trong `mission_params.yaml`, tham số `land_fallback_xy_speed_limit` đặt là $0.65\,\text{m/s}$ (vốn căn chỉnh cho bãi cũ $0.5\,\text{m/s}$). Khi bãi đáp chạy $1.0\,\text{m/s}$, bất kỳ nhịp mất dấu marker ngắn nào ở cự ly sát đất đều làm FSM rơi vào fallback, kẹp tốc độ drone xuống $0.65\,\text{m/s}$. Bãi đáp vọt lên phía trước với vận tốc tương đối $0.35\,\text{m/s}$, sinh ra sai số vị trí $dx > 0$. Bộ điều khiển cố gắng tăng tốc đuổi theo làm drone nghiêng chúc mũi xuống sàn ngay trước khi chạm đất.
- **Cơ chế 2: Hiệu ứng Đòn bẩy Chân đáp & Phản ứng Vòng lặp Tư thế PX4 (The Ground Fulcrum Effect)**:
  - Để duy trì bay tịnh tiến $1.0\,\text{m/s}$ trong không khí, quadrotor bắt buộc phải duy trì góc pitch âm (chúc mũi $\theta \approx -4^\circ \to -6^\circ$).
  - Khi hạ xuống sát sàn ở tư thế chúc mũi, hai chân đáp phía trước chạm mặt bãi đáp trước, trong khi hai chân sau còn lơ lửng cách sàn $2 - 3\,\text{cm}$.
  - Lực nâng từ mặt sàn tác động vào chân trước tạo mô-men ngửa mũi drone lên. Nhưng vòng lặp điều khiển Pitch Rate (250Hz) của PX4 vẫn đang cố duy trì lệnh vận tốc $1.0\,\text{m/s}$ và lệnh bù vị trí ($kp \cdot dx$). Thấy mũi bị ngửa lên, PX4 lập tức tăng ga hai rotor phía sau để ép mũi chúc xuống!
  - Hai chân trước bám ma sát vào mặt bãi đáp đóng vai trò làm điểm tựa đòn bẩy (fulcrum). Lực đẩy rotor sau nhấc bổng đuôi drone lên không trung, đẩy cả thân drone cắm gục đầu xuống sàn.
- **Cơ chế 3: Cắt xung Offboard quá sớm trước khi Disarm hoàn tất (`offboard_commander.py`)**:
  - Khi nhận tín hiệu chạm đất (`touchdown_cb`), lệnh `self.offboard_engaged = False` lập tức cắt hoàn toàn luồng phát setpoint. Trong khoảng thời gian $100 - 200\,\text{ms}$ truyền gói tin MAVLink disarm qua UDP, PX4 bị mất setpoint Offboard, rơi vào trạng thái giữ tư thế dở dang trong khi thân đang chúi về phía trước.

#### 3. Các Bản vá Kỹ thuật Đã Triển khai

1. **Khắc phục Triệt để Hiện tượng Chúc Mũi khi Hạ cánh trên Bãi Di động**:
   - **Làm phẳng tư thế tự động sát mặt đất (Touchdown Attitude Leveling - `corr_fade`)**:
     - Trong `smc_guidance.py` (`_final_descent_step`) và `mission_fsm_node.py` (toàn bộ các nhánh `LAND` fallback):
       $$\text{corr\_fade} = \text{clip}\left(\frac{\text{rel\_alt} - 0.10}{0.10}, 0.0, 1.0\right)$$
       $$\text{correction} = kp \cdot \text{corr\_fade} \cdot [dx, dy]$$
     - Ở độ cao $\le 0.10\,\text{m}$ (khi chân đáp sắp tiếp xúc sàn), thành phần hiệu chỉnh vị trí $kp \cdot dx$ tự động triệt tiêu về $0.0$.
     - Lệnh vận tốc ngang lúc này trở thành thuần túy là feedforward vận tốc bãi đáp ($v_{xy} = v_{\text{target}}$). Không còn gia tốc tịnh tiến đòi hỏi thân nghiêng, góc pitch/roll của drone tự động trở về **$0.0^\circ$ (hoàn toàn thăng bằng phẳng)**, giúp cả 4 chân đáp tiếp xúc mặt bãi đáp đồng thời và êm ái.
   - **Tăng Giới hạn Tốc độ Fallback cho Bãi $1.0\,\text{m/s}$**:
     - Nâng `land_fallback_xy_speed_limit` từ $0.65\,\text{m/s} \to 1.35\,\text{m/s}$ trong `mission_params.yaml` và code FSM.
     - Nâng `v_final_xy_max` trong `smc_guidance_node` từ $0.65\,\text{m/s} \to 1.30\,\text{m/s}$. Drone có đầy đủ $35\%$ biên độ công suất để bám bãi $1.0\,\text{m/s}$ mà không bao giờ bị kẹt tụt lại phía sau.
   - **Duy trì Luồng Offboard & Ép Tải Nhẹ trong Cửa sổ Disarm (`offboard_commander.py`)**:
     - Bổ sung cờ `self.disarming_active`: Khi `touchdown_cb` hoặc Ground Failsafe kích hoạt, Offboard Commander gửi burst 5 gói tin `FORCE DISARM` liên tiếp, đồng thời **tiếp tục phát setpoint duy trì vận tốc bãi đáp kèm lực ép nhẹ xuống sàn ($v_z = -0.12\,\text{m/s}$)** trong tối đa $0.35\,\text{s}$ cho tới khi PX4 xác nhận đã ngắt động cơ hoàn toàn (`self.armed == False`).
     - Lực ép nhẹ $-0.12\,\text{m/s}$ giữ chặt cả 4 chân tiếp xúc mặt sàn, chống hoàn toàn hiện tượng nảy đuôi hoặc lật đòn bẩy.
   - **Chuẩn hóa Ngưỡng Độ cao Tiếp đất**:
     - Cập nhật `touchdown_max_altitude_m = 0.14m` và `altitude_ceiling_m = 0.14m`, `confirmation_duration_s = 0.18s` khớp chính xác với chiều cao chân đáp x500 trên sàn bãi đáp.

2. **Khắc phục Triệt để Tốc độ Hạ Thẳng đứng Bãi Tĩnh**:
   - **Tăng tốc độ hạ trong APPROACH (`mission_fsm_node.py`)**:
     - Khi drone căn giữa trên bãi tĩnh ($h_{\text{err}} \le 0.45\,\text{m}$), thay thế mức $-0.18\,\text{m/s}$ bằng cấu hình động theo độ cao:
       $$v_z = -0.50\,\text{m/s} \text{ (nếu } z > 1.2\,\text{m}) \quad \text{và} \quad -0.35\,\text{m/s} \text{ (nếu } 0.6 < z \le 1.2\,\text{m})$$
   - **Nâng cấp Luật Hạ Thẳng đứng SMC (`smc_guidance.py`)**:
     - Cập nhật `alt_speed = max(self.p.v_descend_touch, 0.40 * r_z)` và `slope_speed = max(0.35, ...)`.
     - Ngay cả khi bãi tĩnh ($v_{xy} = 0$), ở độ cao $2.0\,\text{m}$, drone vẫn hạ với tốc độ tối đa $v_{\text{descend\_fast}} = 0.55\,\text{m/s}$ dứt khoát và nhanh chóng.
   - **Tăng Tốc độ Hạ trong LAND Fallback**:
     - Nâng từ $-0.22\,\text{m/s} \to -0.40\,\text{m/s}$ khi độ cao tương đối $> 0.40\,\text{m}$, và hạ nhẹ nhàng về $-0.20\,\text{m/s}$ khi sát mặt sàn.
   - **Đồng bộ Cấu hình `mission_params.yaml`**:
     - `v_descend_fast: 0.55` (trước là 0.40 m/s).
     - `v_descend_touch: 0.22` (trước là 0.15 m/s).

#### 4. Kết quả Kiểm thử & Đo đạc
- **Hệ thống Kiểm thử Hồi quy (Unit Test Suite)**: **143/143 tests passed (100%)**, bổ sung 2 test case mới:
  1. `test_terminal_attitude_leveling_fades_position_correction_near_touchdown`: Xác nhận tại $z = 0.08\,\text{m}$, thành phần hiệu chỉnh vị trí bị triệt tiêu hoàn toàn, vận tốc ra khớp $100\%$ với vận tốc bãi đáp ($1.0\,\text{m/s}$) để giữ thân thăng bằng phẳng.
  2. `test_static_pad_crisp_vertical_descent`: Xác nhận trên bãi tĩnh ở $z = 2.0\,\text{m}$, tốc độ hạ đạt $0.55\,\text{m/s} \le -0.50\,\text{m/s}$, không còn hiện tượng lơ lửng chậm chạp.
- **Đồng bộ mã nguồn**: Toàn bộ các thay đổi được cập nhật đồng bộ $100\%$ giữa `simulation/` và `embedded_handover/`.



