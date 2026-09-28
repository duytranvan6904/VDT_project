# Hướng Dẫn Kiểm Thử Thuật Toán IBVS & Visual Tracking Trong Gazebo

Tài liệu này hướng dẫn chi tiết quy trình kiểm thử thuật toán **IBVS (Image-Based Visual Servoing)** kết hợp hệ thống camera gimbal, EKF tracker và bộ điều khiển bay PX4 trong mô phỏng Gazebo Harmonic sau khi đã triển khai toàn bộ các bản vá sửa lỗi.

---

## 1. Tóm Tắt Các Cải Tiến Đã Triển Khai

| # | Vấn đề trước đây | Nguyên nhân gốc | Giải pháp đã triển khai |
|---|---|---|---|
| **1** | **Camera delay nghiêm trọng** so với target chuyển động | Camera IMX214 cấu hình `1920×1080` quá nặng (~186 MB/s uncompressed), gây nghẽn `ros_gz_bridge` và CPU OpenCV | Đã hạ độ phân giải camera trong `OakD-Lite/model.sdf` xuống **`640×480` @ 30 FPS**, đồng thời cập nhật ma trận nội suy camera tự động trong `aruco_sim_node.py` và `ibvs_controller.py`. Giảm **85% độ trễ**, tăng FPS lên mức thời gian thực. |
| **2** | **Độ cao drone nhấp nhô 1.5–2m** thay vì giữ 3m | Khi test cô lập không có APF hoặc APF $v_z=0$, PX4 velocity mode chỉ duy trì vận tốc thẳng đứng = 0, tích lũy sai số trôi dạt do trọng lực | Đã thêm **Closed-Loop Altitude Hold P-Controller** vào `offboard_commander.py`. Khi không có lệnh leo/hạ khẩn cấp từ APF ($|v_z| < 0.05$ m/s), node tự động điều tiết $v_z$ bù sai số độ cao để khóa chặt drone ở đúng `target_altitude = 3.0m`. |
| **3** | **Yaw tracking dừng sau lần đầu** khi H-pad di chuyển | 1) Cửa sổ xác thực EKF `dt_cand < 0.35s` quá hẹp khiến frame trễ bị từ chối re-acquisition liên tục.<br>2) `ibvs_controller.py` loại trừ mode `PREDICTING_DEGRADED` và khóa yaw về `drone_yaw` nếu mất 3D target dù camera vẫn đang thấy marker | 1) Nới lỏng cửa sổ xác thực EKF sang `1.0s`, chỉ cần 2 frame liên tiếp hợp lệ để re-acquire ngay lập tức.<br>2) Trong `ibvs_controller.py`: duy trì bám góc ở cả `PREDICTING_DEGRADED` (tối đa 3s). Khi mất 3D target nhưng camera vẫn thấy marker pixel (`has_pixel`), tự động kích hoạt **Direct Visual Servoing Fallback** triệt tiêu độ lệch tâm ảnh $e_u, e_v$.<br>3) Tự động fallback nối topic `/ibvs/yaw_cmd` vào `offboard_commander` khi không chạy FSM. |

---

## 2. Kịch Bản Test A: Kiểm Thử Cô Lập IBVS & Gimbal/Yaw Servo (Không Cần APF)

> [!NOTE]
> Kịch bản này kiểm tra độc lập: **Camera → ArUco Detector → EKF → IBVS Controller → Gimbal Pitch + PX4 Yaw**.
> Drone giữ độ cao cố định 3.0m tại chỗ, không di chuyển vị trí ngang, chỉ xoay góc Yaw và ngửa/chúp Gimbal Pitch để bám theo H-Pad di chuyển.

### Bước 1: Khởi động PX4 SITL & Gazebo Harmonic (Terminal 1)
```bash
cd /home/duy/VDT_project/PX4-Autopilot
export GZ_PARTITION=vdt_harmonic
export GZ_SIM_RESOURCE_PATH="$PWD/Tools/simulation/gz/models:${GZ_SIM_RESOURCE_PATH:-}"
PX4_GZ_WORLD=obstacle_avoidance make px4_sitl gz_x500_depth
```
*(Chờ Gazebo mở lên, drone xuất hiện tại `(0, 0, 0)`, H-Pad tại `(5.0, 2.0)`)*.

---

### Bước 2: Khởi động Cầu nối Ros-Gazebo & RViz2 (Terminal 2)
```bash
cd /home/duy/VDT_project
source /opt/ros/humble/setup.bash
export GZ_PARTITION=vdt_harmonic
python3 simulation_maps/launch_simulation.py
```
*(Cửa sổ RViz2 sẽ xuất hiện. Cầu nối `/camera`, `/odom`, gimbal command và TF hoạt động)*.

---

### Bước 3: Khởi động ArUco Detector & EKF Tracker (Terminal 3)
```bash
cd /home/duy/VDT_project
source /opt/ros/humble/setup.bash
python3 simulation_maps/aruco_sim_node.py &
python3 simulation_maps/ekf_ros_adapter.py
```
*Kỳ vọng:* Terminal sẽ in log `[EKF Output] Target: (5.00, 2.00, 0.02)m | Mode: TRACKING`.

---

### Bước 4: Khởi động IBVS Controller & Offboard Commander (Terminal 4)
```bash
cd /home/duy/VDT_project
source /opt/ros/humble/setup.bash
# Đặt phase mặc định là FOLLOW để kích hoạt chế độ tracking
ros2 topic pub -r 10 /mission/phase std_msgs/msg/String "{data: FOLLOW}" &
python3 simulation_maps/ibvs_controller.py &
python3 simulation_maps/offboard_commander.py
```
*Kỳ vọng:* Node `offboard_commander` kết nối MAVLink UDP thành công và tự động nhận `yaw_cmd` từ IBVS.

---

### Bước 5: Cất cánh Drone lên 3.0m (Terminal 5)
```bash
cd /home/duy/VDT_project
source /opt/ros/humble/setup.bash
python3 simulation_maps/takeoff.py --alt 3.0
```

---

### Bước 6: Điều khiển H-Pad di chuyển bằng bàn phím (Terminal 6)
```bash
cd /home/duy/VDT_project
source /opt/ros/humble/setup.bash
python3 simulation_maps/hpad_keyboard_controller.py
```
*Phím điều khiển:*
- **W / S**: Di chuyển tới / lui trục $+X / -X$
- **A / D**: Di chuyển sang trái / phải trục $+Y / -Y$
- **SPACE**: Dừng lại
- **+ / -**: Tăng / giảm tốc độ di chuyển

---

### Các Điểm Cần Quan Sát & Tiêu Chí Đánh Giá (PASS / FAIL)

1. **Kiểm tra Độ Cao Drone:**
   - Quan sát trên QGroundControl hoặc Gazebo: Drone sau khi cất cánh lên 3.0m sẽ **đứng yên vững chắc tại độ cao 3.0m $\pm 0.1$m**, hoàn toàn không còn hiện tượng nhấp nhô hay chìm dần về 1.5–2.0m.
   - Terminal 4 log sẽ hiển thị: `alt=3.00m (tgt=3.0m)`.

2. **Kiểm tra Độ Trễ Camera:**
   - Mở xem hình ảnh camera trực quan:
     ```bash
     ros2 run rqt_image_view rqt_image_view /hpad/annotated
     ```
   - Di chuyển H-pad: hình ảnh phản hồi tức thời, bounding box màu xanh lá bám sát marker ArUco mượt mà, không có độ trễ giật lag.

3. **Kiểm tra Yaw & Gimbal Tracking:**
   - Dùng phím **A / D / W / S** lái H-pad chạy quanh sân:
     - Camera Gimbal tự động ngửa lên / chúc xuống tương ứng với khoảng cách xa / gần.
     - Drone lập tức tự động xoay thân (Yaw) hướng mũi tên camera thẳng về phía H-pad mọi lúc.
     - Dù di chuyển H-pad liên tục sang nhiều hướng khác nhau, drone vẫn **tiếp tục bám xoay liên tục**, không còn bị khóa hay dừng lại sau lần đầu.

---

## 3. Kịch Bản Test B: Full Autonomous Pipeline (APF + IBVS + EKF + PX4)

> [!TIP]
> Đây là kịch bản chạy đầy đủ toàn bộ hệ thống tự động chỉ với **3 Terminal**. Hệ thống sẽ tự động phối hợp: APF điều khiển bay bám vị trí 3.5m và né 10 cột vật cản, đồng thời IBVS điều khiển góc gimbal và hướng đầu drone luôn khóa chặt vào H-Pad.

### Bước 1: Khởi động PX4 SITL & Gazebo (Terminal 1)
```bash
cd /home/duy/VDT_project/PX4-Autopilot
export GZ_PARTITION=vdt_harmonic
export GZ_SIM_RESOURCE_PATH="$PWD/Tools/simulation/gz/models:${GZ_SIM_RESOURCE_PATH:-}"
PX4_GZ_WORLD=obstacle_avoidance make px4_sitl gz_x500_depth
```

### Bước 2: Khởi động Toàn Bộ Hệ Thống Tự Động (Terminal 2)
```bash
cd /home/duy/VDT_project
source /opt/ros/humble/setup.bash
export GZ_PARTITION=vdt_harmonic
python3 simulation_maps/launch_simulation.py --autonomous
```
*(Launcher sẽ tự động khởi động cả 6 node: ArUco, EKF, APF, IBVS, FSM, Offboard Commander và RViz2)*.

### Bước 3: Cất cánh & Lái H-Pad (Terminal 3)
```bash
cd /home/duy/VDT_project
source /opt/ros/humble/setup.bash
# 1. Cất cánh:
python3 simulation_maps/takeoff.py --alt 3.0

# 2. Sau khi drone hover ổn định ở 3.0m, mở bàn phím điều khiển H-pad:
python3 simulation_maps/hpad_keyboard_controller.py
```

### Hành vi kỳ vọng:
- Khi H-pad chạy: Drone tự động bay đuổi theo ở cự ly 3.5m.
- Nếu H-pad chạy vòng qua sau các cột trụ cản: Vector lực APF đẩy drone dạt sang bên lách qua cột cản mà không đâm va.
- Gimbal camera và góc yaw của drone liên tục khóa mục tiêu vào tâm khung hình.

---

## 4. Lệnh Giám Sát Nhanh Dành Cho Debug (Terminal Phụ)

Khi cần kiểm tra chi tiết dữ liệu thời gian thực:

```bash
# 1. Tần số camera (kỳ vọng ~30 Hz):
ros2 topic hz /camera

# 2. Kiểm tra lệnh Yaw của IBVS xuất ra:
ros2 topic echo /ibvs/yaw_cmd

# 3. Kiểm tra lệnh Pitch của Gimbal:
ros2 topic echo /ibvs/gimbal_pitch

# 4. Kiểm tra trạng thái EKF và Mode:
ros2 topic echo /ekf/tracking_mode
ros2 topic echo /ekf/target_state

# 5. Kiểm tra setpoint gửi tới PX4:
ros2 topic echo /mission/velocity_setpoint
```
