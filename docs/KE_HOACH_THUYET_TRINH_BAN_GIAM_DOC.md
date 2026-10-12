# 📊 KẾ HOẠCH TRÌNH BÀY SLIDE THUYẾT TRÌNH BAN GIÁM ĐỐC
## DỰ ÁN: QUADROTOR TỰ HÀNH BÁM MỤC TIÊU DI ĐỘNG, NÉ VẬT CẢN 3D & HẠ CÁNH CHÍNH XÁC (H-PAD)

> **Thời lượng báo cáo**: 7 phút (420 giây)  
> **Quy mô slide**: 15 slides (Trung bình ~28s / slide)  
> **Đối tượng khán giả**: Ban Giám đốc, Trưởng phòng Kỹ thuật, Hội đồng nghiệm thu dự án  
> **Mục tiêu then chốt**: Làm nổi bật tính đột phá công nghệ ("Điểm ăn tiền"), độ tin cậy an toàn, số liệu thực nghiệm thực tế và tiềm năng thương mại hóa.

---

## ⏱️ BẢNG PHÂN BỔ THỜI GIAN & CẤU TRÚC TỔNG QUAN

| STT | Phần nội dung | Số Slide | Thời lượng (giây) | Tỷ trọng |
|:---:|---|:---:|:---:|:---:|
| **Phần 1** | Giới thiệu đối tượng, bài toán & Hạn chế của giải pháp hiện nay | Slide 1 – 3 | 75s (~1.25 phút) | 18% |
| **Phần 2** | Đóng góp đột phá của nhóm ("Điểm ăn tiền") & Kiến trúc tổng quan | Slide 4 – 6 | 90s (~1.5 phút) | 21% |
| **Phần 3** | Công nghệ lõi từng Module (Công thức & Chức năng chính) | Slide 7 – 10 | 120s (~2.0 phút) | 29% |
| **Phần 4** | Kết quả mô phỏng (Simulink & SITL) + Thực nghiệm Hardware | Slide 11 – 13 | 95s (~1.6 phút) | 23% |
| **Phần 5** | Kết luận, Tiềm năng thương mại & Kế hoạch phát triển | Slide 14 – 15 | 40s (~0.7 phút) | 9% |
| **TỔNG** | **Toàn bộ bài thuyết trình** | **15 Slides** | **420s (7.0 phút)** | **100%** |

---

# CHI TIẾT NỘI DUNG VÀ THIẾT KẾ TỪNG SLIDE

---

## PHẦN 1: ĐỐI TƯỢNG + BÀI TOÁN + HƯỚNG GIẢI QUYẾT ĐÃ CÓ (1.25 PHÚT)

### Slide 1: Trang Tiêu Đề (Title Slide)
* **Thời lượng đề xuất**: 15 giây
* **Tiêu đề Slide**: **HỆ THỐNG QUADROTOR TỰ HÀNH: BÁM MỤC TIÊU DI ĐỘNG, NÉ VẬT CẢN 3D VÀ HẠ CÁNH CHÍNH XÁC MILIMET**
* **Phụ đề**: *Nghiên cứu làm chủ công nghệ điều khiển tự hành tích hợp Nested Dual-Scale Vision, Covariance Gating và Sliding Mode Control.*
* **Thông tin trình bày**: 
  * Đơn vị: Nhóm Nghiên cứu & Phát triển Drone Tự hành (VDT Project)
  * Người trình bày: Kỹ sư Trưởng / Nhóm nghiên cứu
* **Bố cục Visual**:
  * Bên trái: Tiêu đề lớn, phong cách thiết kế tối giản, công nghệ (Dark mode hoặc Navy/Cyan hiện đại).
  * Bên phải: Hình render 3D mô phỏng Quadrotor mang camera RealSense D435i đang khóa mục tiêu bãi đáp H-Pad.
* **Nội dung hiển thị trên slide**:
  * Tên đề tài & Nhóm tác giả.
  * 3 từ khóa định vị công nghệ: *Thị giác máy tính đa tỉ lệ* – *An toàn hiệp phương sai* – *Điều khiển trượt dốc tiếp cận*.
* **Lời thoại gợi ý (Speaker Script - 15s)**:
  > *"Kính thưa Ban Giám đốc và Hội đồng, hôm nay nhóm xin phép báo cáo kết quả nghiên cứu và thực nghiệm giải pháp Quadrotor tự hành bám mục tiêu di động và hạ cánh chính xác xuống bãi đáp H-Pad. Đây là công nghệ nền tảng quyết định sự thành bại của các ứng dụng giao hàng tự động, tiếp vận hàng hải và tác chiến cứu hộ không người lái."*

---

### Slide 2: Đặt Vấn Đề & Mục Tiêu Bài Toán (Problem Statement)
* **Thời lượng đề xuất**: 30 giây
* **Tiêu đề Slide**: **BÀI TOÁN TỰ HÀNH ĐẦU-CUỐI TRONG KHÔNG GIAN HẸP CÓ VẬT CẢN**
* **Bố cục Visual**: Chia 3 khối Card đồ họa tương ứng 3 thách thức thực tế:
  * *Card 1 (Mục tiêu cơ động)*: Bãi đáp H-Pad liên tục di chuyển tự do, có nguy cơ bị che khuất tầm nhìn bởi cây cối, địa hình.
  * *Card 2 (Môi trường phức tạp)*: Không gian chật hẹp, nhiều chướng ngại vật dạng trụ/vách đòi hỏi né tránh 3D thời gian thực mà không làm gián đoạn bám đuổi.
  * *Card 3 (Tiếp đất khắt khe)*: Kích thước bãi đáp hạn chế ($D_{pad} \approx 40\text{–}50\text{cm}$), yêu cầu sai số tiếp đất cực nhỏ ($< 10\text{cm}$), vận tốc hạ êm ái, chống lật đổ khi chạm đất.
* **Nội dung hiển thị trên slide**:
  * Thách thức: Bãi đáp cơ động ngẫu nhiên + che khuất tạm thời ($< 3\text{s}$).
  * Ràng buộc: Né vật cản 3D giữ cự ly an toàn $\ge 0.5\text{m}$.
  * Mục tiêu kỹ thuật: Tiếp đất chính xác $< 10\text{cm}$, hoàn toàn tự hành (End-to-End Autonomy) không cần can thiệp tay của phi công.
* **Lời thoại gợi ý (Speaker Script - 30s)**:
  > *"Trong các nhiệm vụ thực địa, bãi đáp thường xuyên di chuyển và nằm trong khu vực có nhiều vật cản. Mục tiêu cốt lõi của chúng ta là giải quyết đồng thời 3 bài toán khó: vừa bám đuổi mục tiêu di động, vừa chủ động né vật cản 3D thời gian thực, và quan trọng nhất là hạ cánh chính xác từng centimet xuống bãi đáp nhỏ gọn mà không làm hư hại thiết bị."*

---

### Slide 3: Hạn Chế Của Các Hướng Giải Quyết Đã Có (Current Limitations)
* **Thời lượng đề xuất**: 30 giây
* **Tiêu đề Slide**: **4 NGHẼN CỔ CHAI KỸ THUẬT CỦA CÁC GIẢI PHÁP TRUYỀN THỐNG**
* **Bố cục Visual**: 4 ô Box màu đỏ cảnh báo (Alert Boxes) chỉ rõ các "điểm chết" công nghệ:
  1. **Định vị GPS toàn cầu**: Sai số trôi dạt dân sự từ $1.0\text{–}2.5\text{m}$ $\rightarrow$ *Không thể đáp trúng bãi đáp đường kính $50\text{cm}$, nguy cơ rơi lật ra ngoài*.
  2. **Nghịch lý 1 Marker ArUco (Single Marker Paradox)**: Dùng marker lớn thì khi drone hạ thấp ($<0.4\text{m}$) sẽ bị **tràn khung ảnh (FOV)** mất dấu; dùng marker nhỏ thì bay cao ($>2\text{m}$) camera không thể nhận diện.
  3. **Hạ cánh thẳng đứng & Trôi Barometer**: Khi drone phanh chỉnh vị trí, thân nghiêng làm camera hất ra ngoài mất mục tiêu; cảm biến áp suất khí quyển trôi $0.5\text{m}$ làm drone phán đoán sai $\rightarrow$ **hiệu ứng mặt đất (Ground effect) làm nảy và lật máy bay**.
  4. **APF truyền thống**: Dễ bị kẹt thế năng cực tiểu địa phương (Local Minima) và bị hiện tượng GNRON đẩy văng drone ra xa khi mục tiêu ở gần vật cản.
* **Lời thoại gợi ý (Speaker Script - 30s)**:
  > *"Các giải pháp hiện nay trên thị trường gặp 4 rào cản lớn: GPS trôi dạt quá lớn không thể hạ cánh chính xác; thị giác 1 marker bị nghịch lý tràn ảnh khi tới gần; hạ cánh thẳng đứng dễ bị nảy lật do trôi khí áp kế; và các thuật toán né vật cản thông thường rất dễ bị kẹt chết ở góc chết thế năng. Đội ngũ chúng tôi đã giải quyết triệt để 4 vấn đề này bằng một kiến trúc hoàn toàn mới."*

---

## PHẦN 2: ĐÓNG GÓP MỚI & KIẾN TRÚC TỔNG QUAN (1.5 PHÚT)

### Slide 4: Điểm Mới & Đóng Góp Đột Phá — "Điểm Ăn Tiền" (Core Innovations)
* **Thời lượng đề xuất**: 45 giây
* **Tiêu đề Slide**: **5 ĐỘT PHÁ CÔNG NGHỆ ("ĐIỂM ĂN TIỀN" CỦA DỰ ÁN)**
* **Bố cục Visual**: 5 biểu tượng huy hiệu công nghệ (Tech Badges) nổi bật:
  1. 🎯 **Nested Dual-Scale ArUco Board**: Cấu trúc 2 marker đồng tâm chuẩn A0 (ID 42 $52.5\text{cm}$ + ID 43 $10\text{cm}$) cùng chung gốc tọa độ 3D $\rightarrow$ Duy trì visual lock liên tục từ $8.0\text{m}$ xuống tận $0.05\text{m}$ (không điểm mù).
  2. 🌐 **Marker-Relative Navigation**: Đưa gốc tọa độ về tâm bãi đáp, triệt tiêu $100\%$ ảnh hưởng của trôi dạt GPS.
  3. 🛡️ **Dual-Covariance Safety Gating**: Giám sát an toàn bằng ma trận hiệp phương sai kép $2\sigma \le 0.45\text{m}$ và phễu nón $30^\circ$, tự động hủy hạ cánh (Wave-off) nếu sai số lớn.
  4. 📐 **SMC 45° Glide Slope & Triệt Tiêu Hover-Stall**: Dẫn đường dốc trượt $45^\circ$ bằng điều khiển trượt (SMC), camera luôn khóa mục tiêu, bù vận tốc đứng độc lập triệt tiêu hiện tượng kẹt lơ lửng.
  5. 🛑 **Drift-Invariant Touchdown & MAVLink Force Disarm**: Dò chạm đất phi trôi dạt trong $0.35\text{s}$ kết hợp ngắt động cơ cưỡng bức tức thì (`param2=21196.0`), chống nảy lật $100\%$.
* **Lời thoại gợi ý (Speaker Script - 45s)**:
  > *"Điểm ăn tiền làm nên sự khác biệt của dự án gồm 5 giải pháp độc quyền: Thứ nhất, bãi đáp kép Dual-Scale xoá bỏ hoàn toàn điểm mù quang học. Thứ hai, định vị Marker-Relative độc lập với GPS. Thứ ba, cổng an toàn Covariance Gate ngăn chặn mọi rủi ro rơi tự do. Thứ tư, quỹ đạo hạ dốc nghiêng 45 độ bằng điều khiển trượt SMC triệt tiêu hiện tượng kẹt lơ lửng. Và cuối cùng là cơ chế ngắt động cơ dứt khoát trong 0.35 giây, đảm bảo tiếp đất êm ái tuyệt đối."*

---

### Slide 5: Kiến Trúc Phân Tầng Hệ Thống (System Architecture)
* **Thời lượng đề xuất**: 25 giây
* **Tiêu đề Slide**: **KIẾN TRÚC PHẦN MỀM PHÂN TẦNG ĐỒNG BỘ TRÊN NỀN TẢNG ROS 2**
* **Bố cục Visual**: Sơ đồ phân tầng rõ ràng từ trên xuống dưới (4 tầng chức năng):
  * **Tầng Nhận thức (Perception)**: Camera RealSense D435i $\rightarrow$ `aruco_detector` (PnP Board) $\rightarrow$ `ekf_ros_adapter` (Target EKF 6-state).
  * **Tầng Giám sát & Dẫn đường (Guidance & Safety)**: `covariance_gate` ($2\sigma$ & Conical Gate) $\leftrightarrow$ `smc_guidance` (Glide Slope $45^\circ$).
  * **Tầng Điều khiển & Né vật cản (Control)**: `iapf_planner` (Improved APF 3D) $\leftrightarrow$ `ibvs_controller` (Gimbal Pitch & Drone Yaw).
  * **Tầng Chấp hành & An toàn (Execution & Failsafe)**: `mission_fsm_node` $\rightarrow$ `touchdown_detector` $\rightarrow$ `offboard_commander` (MAVLink Force Disarm) $\rightarrow$ Bo Pixhawk (PX4).
* **Lời thoại gợi ý (Speaker Script - 25s)**:
  > *"Hệ thống được thiết kế theo kiến trúc 4 tầng phân lập rõ ràng trên nền tảng ROS 2 Humble. Nhận thức từ camera RealSense đi qua bộ lọc Kalman 6 trạng thái, chuyển đến tầng phân tích an toàn Covariance Gate. Từ đó, bộ điều khiển dẫn đường SMC và tránh vật cản I-APF sẽ tính toán lệnh vận tốc tối ưu và hạ lệnh trực tiếp xuống bộ điều khiển bay Pixhawk thông qua MAVLink."*

---

### Slide 6: Chu Trình FSM 7 Pha & Cơ Chế Wave-off An Toàn (Mission FSM)
* **Thời lượng đề xuất**: 20 giây
* **Tiêu đề Slide**: **CHU TRÌNH NHIỆM VỤ FSM 7 PHA VỚI CƠ CHẾ WAVE-OFF TỰ ĐỘNG**
* **Bố cục Visual**: Sơ đồ chuyển pha trạng thái (State Diagram) với mũi tên động lực:
  $$\text{IDLE} \xrightarrow{\text{Takeoff}} \text{SEARCH} \xrightarrow{\text{Visual Lock}} \text{FOLLOW} \xrightarrow{\text{Land Cmd}} \text{APPROACH} \xrightarrow{2\sigma \le 0.45\text{m}} \text{GLIDE\_SLOPE} \xrightarrow{z \le 0.4\text{m}} \text{LAND} \xrightarrow{\text{Touchdown}} \text{TOUCHDOWN / DISARM}$$
  * Nhánh rẽ cảnh báo nổi bật: **Wave-off Abort** (khi mất dấu $>1.5\text{s}$ hoặc sai số $2\sigma > 0.55\text{m}$, drone tự động hủy hạ cánh, vọt lên giữ độ cao an toàn).
* **Lời thoại gợi ý (Speaker Script - 20s)**:
  > *"Toàn bộ chu trình từ lúc cất cánh đến khi tắt máy được điều phối qua cỗ máy trạng thái hữu hạn 7 pha. Đặc biệt, hệ thống tích hợp cơ chế Wave-off tự động: nếu mục tiêu bị che khuất bất ngờ hoặc sai số vượt ngưỡng an toàn trong khi đang hạ dốc, drone sẽ lập tức ngắt hạ cánh và bay lượn an toàn để tái định vị."*

---

## PHẦN 3: KIẾN TRÚC TỪNG MODULE & CÔNG THỨC CHÍNH (2.0 PHÚT)

### Slide 7: Module Nhận Thức — Dual-Scale Vision & EKF 6 Trạng Thái
* **Thời lượng đề xuất**: 30 giây
* **Tiêu đề Slide**: **MODULE NHẬN THỨC: NESTED ARUCO BOARD & TARGET EKF 6 TRẠNG THÁI**
* **Bố cục Visual**:
  * Bên trái: Hình ảnh bãi đáp A0 với Marker ID 42 ($52.5\text{cm}$) và ID 43 ($10\text{cm}$) đồng tâm, ảnh chụp từ camera ở cự ly $3\text{m}$ và $0.1\text{m}$.
  * Bên phải: Khối hộp công thức toán học và sơ đồ bộ lọc Kalman ước lượng vận tốc.
* **Công thức toán & Chức năng chính**:
  * **Ước lượng Pose Board đa điểm chung gốc $(0,0,0)$**:
    $$(\mathbf{R}^*, \mathbf{t}^*) = \arg\min_{\mathbf{R}, \mathbf{t}} \sum_{k \in \mathcal{V}} \left\| \mathbf{u}_k - \pi(\mathbf{K}, \mathbf{R} \mathbf{P}_k + \mathbf{t}) \right\|^2$$
    *(Chuyển giao liên tục $C^0$, triệt tiêu hoàn toàn bước nhảy tọa độ khi đổi marker)*.
  * **Mô hình động học Constant Velocity (CV) 6 trạng thái**:
    $$\mathbf{x}_w = [x_w, y_w, z_w, v_{wx}, v_{wy}, v_{wz}]^T, \quad \mathbf{x}_k = \mathbf{F}\mathbf{x}_{k-1} + \mathbf{w}_k$$
  * **Cổng loại nhiễu Mahalanobis Gate**: $d_M^2 \le 16.27$ (loại bỏ $100\%$ outlier), duy trì Dead Reckoning quán tính khi mất dấu $<3\text{s}$.
* **Lời thoại gợi ý (Speaker Script - 30s)**:
  > *"Ở tầng nhận thức, chúng tôi giải bài toán PnP trên cụm Marker lồng nhau có chung gốc toạ độ. Nhờ đó, vector vị trí liên tục tuyệt đối khi chuyển đổi giữa tầm xa và tầm gần. Dữ liệu được đưa vào bộ lọc Kalman 6 trạng thái có cổng Mahalanobis loại bỏ nhiễu, cho phép dự đoán quán tính mượt mà ngay cả khi mục tiêu bị che khuất hoàn toàn trong 3 giây."*

---

### Slide 8: Module Né Vật Cản 3D — Thuật Toán Cải Tiến I-APF 3D
* **Thời lượng đề xuất**: 30 giây
* **Tiêu đề Slide**: **MODULE ĐIỀU KHIỂN NÉ VẬT CẢN: THUẬT TOÁN I-APF 3D & THOÁT BẪY KẸT**
* **Bố cục Visual**:
  * Hình minh họa vector lực: Drone né vật cản trụ và lướt theo vector tiếp tuyến 3D (Dynamic Tangent) màu tím.
  * So sánh đường bay: Standard APF (bị kẹt đứng yên) vs. I-APF (lướt thoát bẫy mượt mà).
* **Công thức toán & Chức năng chính**:
  * **Triệt tiêu hiện tượng GNRON bằng trọng số Sigmoid**:
    $$w_{rep} = \frac{1}{1 + e^{-d_{goal}}} - 0.5 \implies \lim_{d_{goal} \to 0} F_{rep} = 0$$
    *(Khi drone áp sát đích, lực cản tự triệt tiêu, đưa drone về đúng tâm bãi đáp mà không bị dội ngược)*.
  * **Thoát bẫy cực tiểu địa phương (Dynamic 3D Tangent Plane)**:
    $$\text{Khi } |F_{total}| < F_{enter} = 0.1\text{N} \implies \text{Sinh 12 tia tiếp tuyến 3D, dự báo } N_{pred}=3 \text{ bước}$$
  * **Khử rung lắc (Oscillation Suppression)**: Lọc hướng bay theo 3 phân đoạn góc $\Delta\alpha \le 30^\circ$, giữ chuyển động êm ái cho Quadrotor.
* **Lời thoại gợi ý (Speaker Script - 30s)**:
  > *"Để né vật cản trong không gian 3D, chúng tôi cải tiến thuật toán thế trường nhân tạo I-APF: Thứ nhất, điều biến lực đẩy bằng hàm Sigmoid giúp loại bỏ triệt để hiện tượng bị dội ngược khi bãi đáp ở gần vật cản. Thứ hai, cơ chế bẫy thế năng Schmitt Trigger tự động sinh 12 tia tiếp tuyến 3D dự báo trước đường bay để luồn lách thoát kẹt mà không bị rung giật máy bay."*

---

### Slide 9: Module Giám Sát An Toàn — Cổng Hiệp Phương Sai Kép (Covariance Gate)
* **Thời lượng đề xuất**: 30 giây
* **Tiêu đề Slide**: **MODULE GIÁM SÁT AN TOÀN: DUAL-COVARIANCE GATING & PHỄU NÓN 30°**
* **Bố cục Visual**:
  * Đồ họa 3D Phễu nón an toàn $30^\circ$ (Conical Funnel) và Elip sai số (Error Ellipse) co cụm lại khi drone hạ thấp.
  * Đồ thị đường cong bán kính không định $2\sigma$ giảm dần theo hàm hyperbol $1/z$.
* **Công thức toán & Chức năng chính**:
  * **Dung hợp sai số vị trí tương đối**:
    $$\mathbf{P}_{relative} = \mathbf{P}_{target\_pos} + \mathbf{P}_{drone\_pos}$$
  * **Bán kính tin cậy 2-sigma theo phương xấu nhất (Worst-Case Directional)**:
    $$\sigma_{xy} = \sqrt{\lambda_{max}(\mathbf{P}_{relative, xy})} \implies R_{conf\_2\sigma} = 2\sqrt{\lambda_{max}}$$
  * **Điều kiện an toàn cấp phép hạ cánh (Gate Decision)**:
    $$\boxed{2\sigma \le 0.45\text{m} \quad \wedge \quad \theta_{cone} = \arctan\left(\frac{R_{xy}}{\Delta z}\right) \le 30^\circ}$$
  * **Bán kính dốc trượt thích ứng**: $R_{switch} = \text{clamp}(5.0 + 2.0 \times 2\sigma, 4.5, 10.0)\text{ m}$.
* **Lời thoại gợi ý (Speaker Script - 30s)**:
  > *"Đây là chốt chặn an toàn quan trọng nhất trước khi hạ cánh: Covariance Gate dung hợp sai số của cả bộ lọc ước lượng mục tiêu và bộ lọc nội tại của drone. Bằng cách phân rã trị riêng ma trận sai số để tìm phương rủi ro cao nhất, hệ thống chỉ cho phép tiếp cận khi bán kính không định 2-sigma nhỏ hơn 45cm và drone nằm trọn trong phễu nón an toàn 30 độ."*

---

### Slide 10: Module Dẫn Đường SMC, Nadir Tilt & Dò Chạm Đất Touchdown
* **Thời lượng đề xuất**: 30 giây
* **Tiêu đề Slide**: **DẪN ĐƯỜNG SMC 45°, GIMBAL NADIR TILT (-85°) & DISARM TỨC THÌ**
* **Bố cục Visual**:
  * Hình cắt quỹ đạo tiếp cận theo dốc nghiêng $45^\circ$, camera luôn nhìn thấy bãi đáp ở trung tâm.
  * Sơ đồ logic 4 điều kiện của bộ dò tiếp đất Touchdown Detector.
* **Công thức toán & Chức năng chính**:
  * **Luật tiếp cận lũy thừa Power Reaching Law (SMC)**:
    $$\dot{S}_i = -k_i |S_i|^{3/5} \text{sgn}(S_i) \quad (i = 1, 2, 3)$$
  * **Bù vận tốc hạ độc lập triệt tiêu kẹt lơ lửng (Hover Stall Elimination)**:
    $$v_{z\_cmd} = -\max(v_{touch}, 0.20 \cdot r_z)$$
  * **Chế độ Nadir Tilt của Gimbal**: Khi độ cao $z < 0.8\text{m}$, tự động ép góc pitch về **$-85^\circ$** để khóa chặt tâm marker nhỏ ID 43.
  * **Bộ dò tiếp đất phi trôi dạt (Drift-Invariant Touchdown Latch)**:
    $$\left( z \le 0.18\text{m} \ \wedge \ |v_z| \le 0.08\text{m/s} \ \wedge \ v_{z\_cmd} \le -0.10\text{m/s} \right) \xrightarrow{\text{chốt } 0.35\text{s}} \textbf{MAVLink Force Disarm} \ (param2=21196.0)$$
* **Lời thoại gợi ý (Speaker Script - 30s)**:
  > *"Trong pha tiếp cận, bộ điều khiển trượt SMC dẫn đường drone theo góc dốc nghiêng 45 độ, giúp camera luôn nhìn chéo thấy bãi đáp. Khi áp sát dưới 80cm, gimbal tự động chúc thẳng -85 độ để khóa tâm marker nhỏ. Ngay khi chân đáp chạm sàn và vận tốc triệt tiêu trong 0.35 giây, lệnh Force Disarm được kích hoạt tức thì, triệt tiêu toàn bộ lực nâng và loại bỏ 100% hiện tượng nảy lật máy bay."*

---

## PHẦN 4: KẾT QUẢ MÔ PHỎNG (SIMULINK & SITL) + THỰC NGHIỆM HARDWARE (1.6 PHÚT)

### Slide 11: Bước 1 — Kiểm Thử & Xác Thực Thuật Toán Trên MATLAB / Simulink
* **Thời lượng đề xuất**: 30 giây
* **Tiêu đề Slide**: **BƯỚC 1: KIỂM THỬ THUẬT TOÁN TRÊN MATLAB / SIMULINK (MODEL-BASED DESIGN)**
* **Phụ đề**: *Xác thực tính ổn định toán học, động lực học 6-DOF và tối ưu hóa bộ tham số trước khi tích hợp vào ROS 2 / SITL.*
* **Bố cục Visual**: Chia 3 khối đồ thị kết quả mô phỏng toán lý tưởng:
  1. *Đồ thị 1 (Dẫn đường SMC 45°)*: 
     - Quỹ đạo tiếp cận nghiêng chuẩn xác $45^\circ$.
     - Đồ thị hội tụ của 3 mặt trượt $S = [S_1, S_2, S_3]^T \to 0$ theo thời gian; triệt tiêu hoàn toàn hiện tượng rung giật (Chattering) bằng hàm bão hòa $\text{sat}(S/\epsilon)$.
     - Vận tốc hạ $v_z$ duy trì đều $\approx -0.15\text{ m/s}$ ngay cả khi sai số ngang $R_{xy} \to 0$ $\rightarrow$ *Xác nhận triệt tiêu kẹt lơ lửng Hover Stall*.
  2. *Đồ thị 2 (Né vật cản 3D I-APF)*: 
     - Mô phỏng kịch bản bẫy đối xứng (Symmetric Obstacle Trap): I-APF kích hoạt lực tiếp tuyến $F_{tan}$ bứt phá khỏi Local Minima trong $< 0.8\text{ s}$ (tỷ lệ thành công $100\%$, trong khi Standard APF bị kẹt dừng hoàn toàn).
     - Đường cong lực cản giảm về 0 nhờ hàm Sigmoid khi áp sát đích (triệt tiêu hiện tượng GNRON).
  3. *Đồ thị 3 (IBVS Gimbal & EKF Occlusion Handling)*: 
     - Kịch bản mất dấu mục tiêu đột ngột $3.0\text{ s}$: EKF duy trì ước lượng quán tính (Dead Reckoning), sai số pixel khi xuất hiện lại $e_u, e_v < 40\text{ px}$.
     - Tín hiệu xung PWM điều khiển servo gimbal mượt mà qua bộ lọc EMA, tốc độ biến thiên góc $\le 180^\circ/\text{s}$, không rung lắc cơ khí.
* **Ý nghĩa then chốt**: Tinh chỉnh và chốt bộ tham số tối ưu ($k_{att}, k_{rep}, k_1, k_2, k_3, K_p, K_s, K_{pitch}, K_{yaw}$) giúp tiết kiệm $80\%$ thời gian debug trên môi trường SITL và ngăn chặn rủi ro va chạm.
* **Lời thoại gợi ý (Speaker Script - 30s)**:
  > *"Để đảm bảo tính khoa học và an toàn tuyệt đối, nhóm tuân thủ nghiêm ngặt quy trình Model-Based Design: Toàn bộ thuật toán được mô phỏng và kiểm chứng trên mô hình 6-DOF của MATLAB/Simulink trước. Kết quả mô phỏng chứng minh: Mặt trượt SMC hội tụ mượt mà theo dốc 45 độ mà không bị chattering; thuật toán I-APF thoát bẫy cực tiểu địa phương 100% trong vòng 0.8 giây; và bộ lọc EKF giữ vững góc ngắm gimbal ngay cả khi mục tiêu bị che khuất 3 giây. Nhờ đó, chúng tôi tối ưu hóa được toàn bộ bộ thông số trước khi đưa vào SITL."*

---

### Slide 12: Bước 2 — Kiểm Chứng Toàn Phần Trên Mô Phỏng PX4 SITL & Gazebo
* **Thời lượng đề xuất**: 35 giây
* **Tiêu đề Slide**: **BƯỚC 2: KẾT QUẢ KIỂM CHỨNG TOÀN PHẦN TRÊN PX4 SITL & GAZEBO HARMONIC**
* **Bố cục Visual**:
  * Hình chụp không gian Gazebo Harmonic và quỹ đạo bay 3D hiển thị trên RViz2 qua các trụ vật cản.
  * Bảng đối chiếu chỉ số đo đạc thực tế so với tiêu chuẩn thiết kế nghiệm thu:

| Hạng mục kiểm chứng | Tiêu chuẩn đặt ra | Kết quả đo đạc thực tế | Đánh giá |
|---|:---:|:---:|:---:|
| **Thời gian khóa mục tiêu ban đầu** | $< 3.0\text{ s}$ | **$0.77\text{ s}$** | Vượt tiến độ |
| **Sai số góc hướng (Yaw) khi bám đuổi** | $< 5.0^\circ$ | **$< 2.5^\circ$** | Rất ổn định |
| **Khoảng cách an toàn né vật cản ($d_{min}$)** | $\ge 0.5\text{ m}$ | **$\ge 0.85\text{ m}$** | Tuyệt đối an toàn |
| **Bán kính không định khi tiếp đất ($2\sigma$)** | $\le 0.35\text{ m}$ | **$< 0.05\text{ m}$** | Siêu chính xác |
| **Sai số tiếp đất lệch tâm ngang ($R_{xy}$)** | $< 10.0\text{ cm}$ | **$\approx 1.0\text{ cm}$ ($0.01\text{m}$)** | **XUẤT SẮC** |
| **Thời gian chốt ngắt động cơ (Disarm)** | $< 1.0\text{ s}$ | **$0.35\text{ s}$** | Không nảy, không trôi |

* **Lời thoại gợi ý (Speaker Script - 35s)**:
  > *"Sau khi thuật toán đạt chuẩn trên Simulink, chúng tôi chuyển thể sang ROS 2 và đưa vào mô phỏng vật lý toàn phần PX4 SITL với Gazebo Harmonic. Ở môi trường đầy đủ nhiễu cảm biến và trễ truyền thông, hệ thống vẫn thể hiện hiệu năng xuất sắc: Drone khóa mục tiêu sau 0.77 giây, né vật cản an toàn trên 0.85m. Đặc biệt, sai số tiếp đất thực tế đạt xấp xỉ 1 centimet so với tiêu chuẩn 10 centimet, và bộ dò tiếp đất ngắt động cơ dứt khoát sau đúng 0.35 giây."*

---

### Slide 13: Bước 3 — Đóng Gói C++ Nhúng & Thực Nghiệm Phần Cứng
* **Thời lượng đề xuất**: 30 giây
* **Tiêu đề Slide**: **BƯỚC 3: ĐÓNG GÓI C++ NHÚNG TRÊN RASPBERRY PI 5 & BENCH-TEST PHẦN CỨNG**
* **Bố cục Visual**:
  * Ảnh chụp phần cứng thực tế: Drone gắn Companion Computer Raspberry Pi 5 / Jetson Nano, bo PX4 Pixhawk 6C, Camera Intel RealSense D435i và Gimbal Servo.
  * Ảnh chụp màn hình Bench-test nhận diện bãi đáp A0 trên bàn Lab (cửa sổ OpenCV nhận diện, hiển thị trục 3D chuẩn tâm).
  * Sơ đồ cấu hình mạng Hotspot Unicast UDP (CycloneDDS) giữa máy tính trạm GCS và Pi 5.
* **Thông số kỹ thuật phần cứng đã xác nhận**:
  * Tần số xử lý Vision: $\ge 25\text{–}30\text{ Hz}$ trên Raspberry Pi 5.
  * Tần số điều khiển vòng kín PX4: $250\text{ Hz}$ qua kết nối UART TELEM2 ($921600\text{ baud}$).
  * Bộ mã nguồn C++20 Header-Only tối ưu hóa riêng cho CPU ARM Cortex-A76 (`-O3 -mcpu=cortex-a76`).
  * Bảng điều khiển GCS Operator Terminal: Phát lệnh Land, Emergency Abort và giám sát Telemetry thời gian thực.
* **Lời thoại gợi ý (Speaker Script - 30s)**:
  > *"Toàn bộ pipeline thuật toán đã được đóng gói hoàn chỉnh sang C++20 tối ưu cho vi xử lý ARM của Raspberry Pi 5 kết nối với Pixhawk qua UART tốc độ cao 921600 baud. Các bài bench-test trên bàn thí nghiệm với camera RealSense thực tế đã xác nhận tần số xử lý đạt 30 khung hình/giây và đo đạc khoảng cách trùng khớp với thực tế. Hệ thống đã hoàn thiện sẵn sàng cho các bài bay thực địa."*

---

## PHẦN 5: KẾT LUẬN & HƯỚNG PHÁT TRIỂN (0.7 PHÚT)

### Slide 14: Giá Trị Thực Tiễn & Tiềm Năng Thương Mại Hóa (Business Value)
* **Thời lượng đề xuất**: 20 giây
* **Tiêu đề Slide**: **GIÁ TRỊ THỰC TIỄN & TIỀM NĂNG THƯƠNG MẠI HÓA CÔNG NGHỆ**
* **Bố cục Visual**: 3 khối ứng dụng thực tế với biểu tượng minh họa sinh động:
  1. 📦 **Logistics & Giao hàng tự động chặng cuối (Last-mile Delivery)**: Tự động hạ cánh chính xác xuống trạm sạc pin thông minh hoặc thùng xe bán tải đang di chuyển.
  2. 🚢 **Tiếp vận Hàng hải & Quân sự (Maritime & Defense)**: Đáp chính xác lên boong tàu cứu hộ hoặc xe bọc thép đang cơ động dập dềnh trên sóng nước.
  3. ⚡ **Tự động hóa Kiểm tra Công nghiệp (Automated Inspection)**: Tuần tra đường dây điện, trạm biến áp, tự động quay về trạm sạc không dây với độ lệch chỉ vài centimet.
* **Nội dung nhấn mạnh**: Tự chủ $100\%$ công nghệ lõi, không phụ thuộc vào giải pháp độc quyền của nước ngoài, tiết kiệm chi phí bản quyền và dễ dàng tích hợp tùy biến.
* **Lời thoại gợi ý (Speaker Script - 20s)**:
  > *"Giải pháp này mở ra tiềm năng ứng dụng và giá trị thương mại rất lớn cho công ty: Làm chủ công nghệ hạ cánh chính xác giúp chúng ta tự động hóa hoàn toàn các trạm sạc không người lái cho drone kiểm tra công nghiệp, triển khai các giải pháp giao hàng tự động tiếp đất trên thùng xe tải, hoặc phục vụ các nhiệm vụ trinh sát tiếp vận trên tàu thuyền quân sự."*

---

### Slide 15: Lộ Trình Triển Khai Thực Địa & Đề Xuất Phê Duyệt (Roadmap)
* **Thời lượng đề xuất**: 20 giây
* **Tiêu đề Slide**: **LỘ TRÌNH TRIỂN KHAI THỰC ĐỊA & ĐỀ XUẤT PHÊ DUYỆT NGHIỆM THU**
* **Bố cục Visual**: Trục thời gian (Milestone Roadmap) với 3 giai đoạn rõ ràng:
  * **Giai đoạn 1 (Đã hoàn thành 100%)**: Nghiệm thu mô phỏng Simulink, mô phỏng SITL, Bench-test phần cứng, đóng gói C++ ROS 2 Node.
  * **Giai đoạn 2 (Tháng tới)**: Bay thử nghiệm dã ngoại ngoài trời (Outdoor Flight Test) trên bãi đáp tĩnh và bãi đáp di động vận tốc $3\text{ m/s}$.
  * **Giai đoạn 3 (Quý tới)**: Tích hợp AI Object Detection (YOLOv8-Nano) nhận diện vật cản động và hạ cánh trên mặt sàn dập dềnh sóng nước.
* **Lời kêu gọi hành động (Call-to-Action)**:
  * *"Kính đề nghị Ban Giám đốc phê duyệt kết quả nghiệm thu giai đoạn mô phỏng và cấp phép triển khai đợt bay thử nghiệm thực địa ngoài trời."*
* **Lời thoại gợi ý (Speaker Script - 20s)**:
  > *"Chúng tôi đã hoàn thành 100% mục tiêu giai đoạn 1 từ Simulink đến SITL và bench-test phần cứng. Kế hoạch tiếp theo là tổ chức đợt bay thử nghiệm ngoài trời trên bãi đáp di động trong tháng tới. Nhóm kính đề nghị Ban Giám đốc phê duyệt kết quả nghiên cứu hôm nay để chúng tôi tự tin tiến ra bãi bay thực địa. Xin trân trọng cảm ơn Ban Giám đốc!"*

---

# 📚 TỔNG HỢP CÁC CÔNG THỨC & THAM SỐ ĂN TIỀN (PHỤ LỤC Q&A)

Dành cho bạn để tự tin trả lời các câu hỏi chất vấn sâu về mặt kỹ thuật của Ban Giám đốc / Hội đồng chuyên gia:

### 1. Công thức PnP đa điểm Nested ArUco Board
$$(\mathbf{R}^*, \mathbf{t}^*) = \arg\min_{\mathbf{R}, \mathbf{t}} \sum_{k \in \mathcal{V}} \left\| \mathbf{u}_k - \pi(\mathbf{K}, \mathbf{R} \mathbf{P}_k + \mathbf{t}) \right\|^2$$
*Giải thích*: Cả 8 đỉnh của Outer Marker (ID 42 - $52.5\text{cm}$) và Inner Marker (ID 43 - $10\text{cm}$) đều được định nghĩa trên cùng một hệ tọa độ phẳng $Z=0$ với gốc $(0,0,0)$ đặt tại tâm marker nhỏ. Nhờ đó, nghiệm vị trí trả về liên tục $C^0$, không xảy ra hiện tượng nhảy giật khi chuyển giao tầm nhìn.

### 2. Dung hợp Covariance & Trị riêng cực đại
$$\mathbf{P}_{relative} = \mathbf{P}_{target\_pos} + \mathbf{P}_{drone\_pos}$$
$$\lambda_{max}(\mathbf{P}_{xy}) = \frac{\text{tr}(\mathbf{P}_{xy}) + \sqrt{(\text{tr}(\mathbf{P}_{xy}))^2 - 4\det(\mathbf{P}_{xy})}}{2}$$
$$R_{conf\_2\sigma} = 2\sqrt{\lambda_{max}(\mathbf{P}_{xy})} \le 0.45\text{m}$$
*Giải thích*: Trị riêng cực đại đại diện cho bán kính trục dài của elip sai số (hướng rủi ro sai lệch lớn nhất). Việc kiểm tra $2\sigma \le 0.45\text{m}$ đảm bảo xác suất tin cậy $95.4\%$ vị trí tiếp xúc của chân đáp nằm trọn vẹn trong đường kính bãi đáp.

### 3. Luật tiếp cận lũy thừa SMC Glide Slope
$$\dot{S}_i = -k_i |S_i|^{3/5} \text{sgn}(S_i), \quad S = [S_1, S_2, S_3]^T$$
$$v_{z\_cmd} = -\max(v_{touch}, 0.20 \cdot r_z)$$
*Giải thích*: Tỷ số lũy thừa $3/5 = 0.6$ giúp mặt trượt hội tụ nhanh khi ở xa và trơn tru không chattering khi áp sát bề mặt. Thành phần $v_{z\_cmd}$ độc lập đảm bảo drone luôn duy trì vận tốc hạ tối thiểu $0.15\text{m/s}$, loại bỏ hiện tượng bị "treo lơ lửng" (Hover Stall) khi sai số ngang $R_{xy} \to 0$.

### 4. Thuật toán thoát kẹt Local Minima của I-APF 3D
* **Ngưỡng kích hoạt**: $|F_{total}| < F_{enter} = 0.10\text{N}$, thoát ra khi $|F_{total}| > F_{exit} = 0.30\text{N}$ (cơ chế trễ Hysteresis chống chập chờn).
* **Dynamic Tangent**: Sinh 12 tia tiếp tuyến vuông góc với vector lực cản trong không gian 3D, đánh giá hàm chi phí dựa trên độ thông thoáng và góc hướng về đích sau 3 bước dự báo trước ($N_{pred}=3$).

---

# 🎯 3 KỊCH BẢN CHẤT VẤN HÓC BÚA CỦA BAN GIÁM ĐỐC & CÁCH ĐÁP

* **Câu hỏi 1**: *"Tại sao không dùng một camera có độ phân giải siêu cao (4K) để nhìn marker từ xa xuống gần thay vì phải in bãi đáp kép phức tạp?"*
  * **Trả lời**: *"Dạ thưa Ban Giám đốc, camera 4K đòi hỏi năng lực tính toán cực lớn khiến vi xử lý nhúng trên drone bị quá tải nhiệt và giảm FPS xuống dưới 10Hz, gây trễ điều khiển bay nguy hiểm. Giải pháp bãi đáp kép Nested Board cho phép dùng camera tiêu chuẩn 720p/VGA với tần số xử lý cao tới 30Hz, tiêu thụ rất ít điện năng và chi phí rẻ hơn 5 lần."*

* **Câu hỏi 2**: *"Nếu gió thổi mạnh làm drone lệch khỏi dốc trượt 45 độ thì sao?"*
  * **Trả lời**: *"Dạ thưa Ban Giám đốc, bộ điều khiển trượt SMC có đặc tính bền vững (robustness) rất cao trước nhiễu loạn gió. Ngoài ra, Covariance Gate liên tục kiểm tra phễu nón an toàn 30 độ; nếu gió đẩy drone lệch ra ngoài nón hoặc sai số 2-sigma vượt 0.55m, FSM sẽ lập tức kích hoạt Wave-off đưa drone leo cao an toàn để tiếp cận lại."*

* **Câu hỏi 3**: *"Hệ thống này mang ra ngoài trời không có Internet thì máy tính trạm GCS kết nối với Drone kiểu gì?"*
  * **Trả lời**: *"Dạ thưa Ban Giám đốc, hệ thống không cần Internet. Chúng tôi đã cấu hình giao thức CycloneDDS qua mạng Hotspot Wi-Fi Unicast trực tiếp giữa Laptop và Drone với ROS Domain ID riêng biệt, đảm bảo truyền nhận tin cậy và độc lập hoàn toàn với hạ tầng mạng bên ngoài."*
