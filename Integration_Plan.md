# Kiến Trúc Tích Hợp Liên Module — Vision · Planner · Embedded

## 1. Phương án tích hợp

- Dùng 1 package trung gian `integration_bridge`, không sửa msg nội bộ của
  `fsm_state_machine` / `offboard_manager`.
- Vision (Duy) và Planner (Tuân) publish theo 4 msg hợp đồng, bridge convert
  sang type embedded đang chờ.
- `vision_bridge_node` ngừng publish khi state không hợp lệ (`valid_for_control
  = false`) → EKF EXPIRED tự thành timeout, `input_state_cache` xử lý bình
  thường, không cần sửa gì phía đó.
- Dùng bridge trong giai đoạn W1–W4 (format còn đổi). Tới mốc SITL (W5), nếu
  format đã ổn định → fold logic convert thẳng vào embedded, bỏ hop bridge để
  giảm latency.

## 2. Template msg hợp đồng

```
# TargetStateFiltered.msg — từ EKF
std_msgs/Header header
geometry_msgs/PoseWithCovariance pose
geometry_msgs/TwistWithCovariance twist
string mode                # TRACKING | PREDICTING | EXPIRED ...
float64 age_since_measurement
bool valid_for_control

# AlignmentStatus.msg — từ vision detector
std_msgs/Header header
bool marker_visible
float64 pixel_align_error

# AltitudeStatus.msg — nguồn TBD (xem mục 4.1)
std_msgs/Header header
float64 altitude
bool touchdown_flag

# PlannerCommand.msg — từ APF/VO-APF
std_msgs/Header header
float64 vx
float64 vy
float64 vz
float64 yaw
```

## 3. Template node bridge 

```cpp
// vision_bridge_node — pattern chung cho cả 3 luồng vision
sub(target_state_topic, TargetStateFiltered) -> {
    if (!msg.valid_for_control) return;   // im lặng = timeout tự nhiên
    pub(hpad/state_filtered, toOdometry(msg));
}
sub(alignment_status_topic, AlignmentStatus) -> {
    pub(hpad/pose, toVisionMarker(msg));
}
sub(altitude_status_topic, AltitudeStatus) -> {
    pub(alt_estimator/state, toAltEstimate(msg));
}
```

```cpp
// planner_bridge_node
sub(planner_command_topic, PlannerCommand) -> {
    pub(planner_output_topic, toPlannerOutput(msg));  // topic thật cần verify
}
```

- Mỗi topic input/output đều là ROS param, remap được không cần build lại.
- Không thêm logic xử lý nào khác ngoài convert type + gate validity — bridge
  càng mỏng càng dễ fold vào embedded sau này.

## 4. Các điểm cần chốt 

### 4.1. Nguồn publish `AltitudeStatus`

- Vấn đề: cả scope README lẫn EKF plan đều không giao node nào chịu trách
  nhiệm altitude + touchdown_flag.
- 2 phương án khả dĩ, mỗi phương án kéo theo hệ quả khác nhau:
  - PX4 `local_position.z` passthrough: đơn giản, latency thấp, nhưng
    không có `touchdown_flag` thật — phải suy ra từ ngưỡng z + velocity, dễ
    false positive khi bay thấp gần vật cản (không phải touchdown).
  - Node riêng dùng depth/accelerometer spike: đúng với mô tả "gia tốc kế
    spike" trong scope README §2.5, nhưng chưa ai code, thêm 1 node mới vào
    hệ đã đông process trên Pi 5.
- Rủi ro nếu không chốt: FSM không có input hợp lệ cho state LAND → touchdown
  detect sai, không disarm đúng lúc.

### 4.2. Tên topic thật của `PlannerOutput`

- Vấn đề: `offboard_manager` có sub kiểu `PlannerOutput{vx,vy,vz,yaw}` nhưng
  README không in tên topic — chỉ suy luận từ bảng Messages, không phải xác
  nhận từ source.
- Rủi ro nếu đoán sai: bridge publish vào topic không ai lắng nghe → hệ thống
  build thành công, chạy không lỗi, nhưng drone không di chuyển theo lệnh
  planner — loại lỗi khó debug nhất vì không có exception, chỉ là "im lặng".
- Cách chốt: đọc trực tiếp source `offboard_manager` (không có trong tài liệu
  đã đưa), hoặc `ros2 topic list` + `ros2 topic info` khi node chạy thật.

### 4.3. Đơn vị `pixel_align_error` và ngưỡng `0.3`

- Vấn đề: `VisionMarker.pixel_align_error` nghe tên là pixel, nhưng FSM dùng
  ngưỡng `0.3` để chuyển APPROACH → LAND — số này hợp lý hơn nếu là mét hoặc
  giá trị normalize [0,1], không hợp lý nếu là pixel thô (ảnh vài trăm–vài
  nghìn px, ngưỡng 0.3px là không tưởng).
- Rủi ro: nếu đơn vị sai lệch bậc, FSM sẽ hoặc không bao giờ chuyển sang LAND
  (ngưỡng quá chặt), hoặc chuyển ngay khi vừa thấy marker (ngưỡng quá lỏng)
  — cả hai đều là lỗi an toàn nghiêm trọng ở pha cuối cùng.
- Cách chốt: Duy công bố công thức tính `pixel_align_error` (pixel thô hay đã
  chia cho nửa chiều rộng ảnh), Việt Anh đối chiếu ngược lại với ngưỡng đang
  hard-code trong `fsm_state_machine`.

### 4.4. `yaw` trong `PlannerCommand` — setpoint hay rate

- Vấn đề: APF/VO-APF tính lực → velocity vector, không có mô tả rõ nó có
  bao gồm thành phần yaw hay không, và nếu có thì là góc tuyệt đối hay tốc độ
  quay.
- Rủi ro: `offboard_manager` build `TrajectorySetpoint` cho PX4 — PX4 phân
  biệt rõ yaw setpoint (rad) và yaw rate (rad/s) là 2 field khác nhau trong
  message gửi xuống. Gán nhầm loại sẽ khiến drone quay sai hướng hoặc quay
  liên tục không dừng.
- Cách chốt: xác nhận với Tuân xem APF hiện có tính yaw không (theo scope
  README, APF mới tính lực 3D vị trí, chưa thấy đề cập yaw) — nếu chưa, field
  này tạm thời nên lấy từ hướng nhìn về H-Pad (`atan2`) ở tầng bridge hoặc
  FSM, không chờ planner cấp.

### 4.5. RAM Pi 5: 8GB hay 4GB

- Vấn đề: 2 con số khác nhau trong cùng 1 tài liệu (scope README §1.2 vs
  §3.2).
- Rủi ro: đây không phải lỗi đánh máy vô hại — margin RAM quyết định có chạy
  nổi đồng thời vision (buffer RGB+Depth 30fps) + EKF + planner + 9 node
  embedded hay không. Sai lệch 4GB là gấp đôi, đủ để một plan "chạy được" trên
  giấy nhưng OOM-kill node giữa chuyến bay trên phần cứng thật.
- Cách chốt: kiểm tra `cat /proc/meminfo` trên board thật, ghi lại 1 số duy
  nhất, xoá số còn lại khỏi tài liệu.

## 5. Tránh oversubscribe thread trên Pi 5 (4 core)

- Kiểm kê toàn bộ process chạy cùng lúc: 9 node embedded + vision (2 thread
  RGB/Depth + thread nội bộ OpenCV) + EKF + planner + `MicroXRCEAgent` +
  `landing_diagnostics` logger.
- Đo số thread thực tế mỗi process bằng `ps -eLf` hoặc `htop` (per-thread
  view) khi chạy full pipeline SITL, không đoán trên giấy.
- Giới hạn thread pool OpenCV (`cv2.setNumThreads()`) nếu vision tự spawn
  nhiều thread hơn cần thiết.
- Gộp node nhẹ (bridge, `input_state_cache`, `kill_switch`) vào chung 1
  component container (intra-process) thay vì mỗi node 1 process riêng.
- Set `MultiThreadedExecutor` thread count tường minh cho từng process, không
  để mặc định tự scale theo số core rồi cộng dồn ngoài ý muốn.

## 6. Tách priority: safety-critical vs soft real-time

- 2 nhóm rõ ràng:
  - Safety-critical (cần timing chặt): `kill_switch`,
    `offboard_safety_monitor`, `offboard_manager`.
  - Soft real-time (lệch vài chục ms không nguy hiểm ngay): vision, EKF,
    planner, `landing_diagnostics`.
- Pin nhóm safety-critical vào 1–2 core riêng bằng `taskset`/cgroup CPU
  affinity.
- Set `SCHED_FIFO` + priority cao hơn cho nhóm safety-critical (cần quyền phù
  hợp / `CAP_SYS_NICE` trên Pi 5).
- Test bắt buộc trước flight test thật: cho vision+planner chạy full tải, đo
  `watchdog_timeout_sec=0.5` của `offboard_safety_monitor` có bị trễ không.

## 7. Latency tiềm ẩn cần đo, không giả định

- Đo latency end-to-end thật: camera → ArUco → EKF → planner → bridge →
  offboard_manager → PX4. So với KPI <500ms trong scope README.
- Đo riêng latency bridge cộng thêm, so với gọi thẳng không qua bridge — dùng
  để quyết fold hay không ở mốc W5.
- Đo DDS discovery/serialization overhead khi có ~15+ ROS 2 participant cùng
  lúc; cân nhắc intra-process comm cho node compose chung container.
- I/O contention: `landing_diagnostics` ghi CSV ~20Hz cùng lúc camera đọc
  RGB+Depth 30fps qua USB, chung SD card → buffer log ở RAM, flush theo batch
  thay vì ghi mỗi tick.

## 8. Các vấn đề khác

- Xác nhận `rc_parser` (UART SBUS) và `xrce_bridge_manager` (`/dev/ttyAMA0`)
  dùng 2 UART vật lý khác nhau trên Pi 5, không share port.
- Chưa có load test đo CPU/RAM thực tế khi chạy full pipeline đồng thời — rủi
  ro lớn nhất hiện tại, cần làm trước bất kỳ flight test nào.
