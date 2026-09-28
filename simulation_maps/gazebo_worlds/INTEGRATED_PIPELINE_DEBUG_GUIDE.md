# Hướng dẫn kiểm tra pipeline APF–EKF–IBVS–Gimbal–Yaw

Tài liệu này dùng để kiểm tra ba hiện tượng khi ghép toàn bộ pipeline:

1. Drone giật/lắc khi vừa nhận target và bắt đầu xoay yaw.
2. Drone tiến gần target nhưng camera không gập xuống, sau đó vào `SEARCH` và quay vòng.
3. Target di động được re-acquire trong thời gian ngắn nhưng drone giật yaw rồi mất tracking.

Phạm vi là mô phỏng Gazebo/PX4. Chưa được phép dùng các thông số này để bay thật trước khi các tiêu chí PASS bên dưới đạt đầy đủ.

---

## 1. Kết luận kiểm tra code hiện tại

Các kết luận sau là những điểm cần kiểm tra trước khi tune gain. Chúng được đối chiếu với code hiện tại trong repository.

| Mức | Điểm kiểm tra | Bằng chứng trong code | Ý nghĩa với hiện tượng |
|---|---|---|---|
| P0 | Gimbal bị reset khi vào `SEARCH` | [`ibvs_controller.py`](../ibvs_controller.py), `fallback_timer_cb()` đặt pitch về `default_pitch=-20°` ở mỗi chu kỳ | Nếu target nằm gần dưới drone, camera lại nhìn ngang/chúc rất ít và không thể re-acquire target |
| P0 | `detected=True` không đồng nghĩa phép đo đã được EKF chấp nhận | [`ekf_ros_adapter.py`](../ekf_ros_adapter.py), `detected_cb()` cập nhật `last_detection_time`; measurement bị kinematic gate từ chối vẫn có thể giữ mode cũ | Target di động có thể làm controller dùng target world-frame cũ, sau đó đổi đột ngột khi EKF re-acquire |
| P0 | Yaw được điều khiển bởi bearing world-frame và pixel trim đồng thời | [`ibvs_controller.py`](../ibvs_controller.py), `target_yaw=atan2(dy,dx)` rồi cộng hiệu chỉnh pixel | Nếu target/EKF nhảy hoặc frame lệch timestamp, yaw setpoint cũng nhảy; bộ giới hạn hiện tại là `0.6 rad/s` |
| P1 | `SEARCH`, `FOLLOW`, `APPROACH` không dùng cùng một envelope pitch | Code hiện tại: `SEARCH [-60°, -10°]`, `FOLLOW [-88°, -10°]`, `APPROACH [-88°, -20°]`, `LAND [-90°, -60°]` | Gimbal có thể nhìn thấy target trong `FOLLOW`, nhưng mất target rồi vào `SEARCH` thì vẫn có thể không quét đủ thấp |
| P1 | Servo Gazebo chậm hơn giới hạn pitch của IBVS | SDF giới hạn joint `velocity=1.5 rad/s`, trong khi YAML đặt `pitch_rate_limit=3.1416 rad/s` | Lệnh thay đổi nhanh hơn khả năng cơ khí, tạo trễ pha và có thể làm target ra khỏi FOV |
| P1 | APF yaw hiện không phải nguồn yaw cuối cùng | APF có publish `/apf/yaw_cmd`, nhưng FSM lấy `/ibvs/yaw_cmd` để publish `/mission/yaw_setpoint` | Không tune APF yaw để sửa lỗi này; cần kiểm tra riêng IBVS yaw → FSM → Offboard |
| P1 | Khoảng cách `follow_distance=3.5 m` là khoảng cách 3D | [`apf_planner.py`](../apf_planner.py), `make_follow_goal()` suy ra khoảng cách ngang từ khoảng cách xiên | Ở độ cao 3 m, target trên mặt đất cho khoảng cách ngang khoảng `1.84 m`, không phải 3.5 m |
| P1 | APF dừng sớm hơn IBVS khi mất measurement | IBVS còn chấp nhận `PREDICTING_DEGRADED`, nhưng APF chỉ chạy với `TRACKING/PREDICTING` | Camera có thể tiếp tục đổi pitch trong khi drone đã dừng; sau `EXPIRED`, FSM chuyển `SEARCH` và pitch bị reset |

### 1.1. Envelope góc thực tế

Joint `CameraJoint` trong [`x500_depth/model.sdf`](../../PX4-Autopilot/Tools/simulation/gz/models/x500_depth/model.sdf) có:

```text
lower   = -90°
upper   = +15°
velocity = 1.5 rad/s ≈ 85.9°/s
```

Quy ước của pipeline hiện tại là pitch âm hướng camera xuống. Với drone ở `z=3.0 m`, target ở `z≈0`, góc hình học cần thiết là:

```text
pitch_required = -atan2(z_drone - z_target, horizontal_distance)
```

Ví dụ follow đúng khoảng cách 3.5 m:

```text
horizontal_distance ≈ sqrt(3.5² - 2.98²) ≈ 1.84 m
pitch_required ≈ -58.3°
```

Góc này nằm trong `FOLLOW`. Nhưng nếu APF đưa drone gần thẳng đứng trên target thì góc cần gần `-90°`; `FOLLOW` hiện chỉ cho phép tới `-88°`, còn khi chuyển sang `SEARCH` chỉ cho phép tới `-60°` và vẫn kéo dần về `default_pitch=-20°`. Đây là nguyên nhân rất phù hợp với việc target đứng yên ngay dưới drone nhưng camera không tìm lại được.

### 1.2. Cần phân biệt “nhìn thấy marker” và “measurement hợp lệ”

`aruco_sim_node.py` phát `/hpad/detected=True` trước khi `ekf_ros_adapter.py` hoàn tất kiểm tra kinematic gate. Trong `ekf_ros_adapter.py`, một measurement có thể bị log:

```text
[KINEMATIC GATE] Rejected jump ...
```

nhưng controller vẫn nhận bbox pixel và cờ detected. Do đó trong lúc re-acquire:

- bbox mới có thể đã xuất hiện;
- target world-frame có thể vẫn là state cũ;
- `tracking_mode` có thể chưa phản ánh việc measurement vừa bị loại;
- yaw có thể bị kéo theo state cũ rồi đổi hướng khi xuất hiện `[EKF RE-ACQUIRED]`.

Không xem `detected=True` là điều kiện đủ để cho phép APF/yaw dùng state mới. Khi test phải ghi đồng thời `/hpad/detected`, `/hpad/position_camera`, `/ekf/tracking_mode`, `/ekf/target_state` và log `[KINEMATIC GATE]`.

---

## 2. Chuẩn bị môi trường chung & Ghi ROS 2 Bag

Mỗi khi mở một Terminal mới, luôn thiết lập môi trường:

```bash
cd /home/duy/VDT_project
source /opt/ros/humble/setup.bash
export GZ_PARTITION=vdt_harmonic
```

### Terminal Ghi Log/Bag (Khuyến nghị mở riêng cho mỗi bài test):

```bash
cd /home/duy/VDT_project
ros2 bag record -o /tmp/vdt_integrated_debug \
  /odom /hpad/detected /hpad/position_camera /hpad/bbox \
  /ekf/target_state /ekf/tracking_mode \
  /apf/velocity_cmd /apf/yaw_cmd \
  /ibvs/yaw_cmd /ibvs/gimbal_pitch \
  /mission/phase /mission/velocity_setpoint /mission/yaw_setpoint \
  /joint_states /tf /tf_static
```

Kiểm tra tần số hoạt động của các topic quan trọng:

```bash
ros2 topic hz /camera
ros2 topic hz /hpad/bbox
ros2 topic hz /odom
ros2 topic hz /ibvs/yaw_cmd
ros2 topic hz /ibvs/gimbal_pitch
```

*Nếu `/camera` hoặc `/odom` bị trễ/đứt, chưa được phép kết luận lỗi nằm ở gain.*

---

## 3. Test 0 — Xác nhận pitch Gazebo, feedback joint và TF

> **Mục tiêu:** Kiểm tra cơ cấu phần cứng ảo gimbal:
> 1. Chiều quay của lệnh pitch (âm = chúc xuống, 0 = nhìn ngang).
> 2. Feedback góc thực từ Gazebo qua `/joint_states`.
> 3. Tọa độ TF giữa `base_link` và `camera_link`.
> *(Không chạy APF, không cất cánh drone, không chạy EKF)*.

### Terminal 1: Khởi động PX4 SITL & Gazebo Harmonic
```bash
# Khởi động an toàn (tự động diệt tiến trình gz/px4 treo và nạp đầy đủ biến môi trường):
./simulation_maps/start_px4_sim.sh
```
*(Chờ Gazebo mở lên, mô hình drone x500 xuất hiện)*.


### Terminal 2: Khởi động Cầu nối Ros-Gazebo & TF (Manual Mode, không --autonomous)
```bash
cd /home/duy/VDT_project
source /opt/ros/humble/setup.bash
export GZ_PARTITION=vdt_harmonic
python3 simulation_maps/launch_simulation.py
```
*(Cửa sổ RViz2 sẽ xuất hiện. Cầu nối `/model/x500_depth_0/command/gimbal_pitch` và `/joint_states` sẵn sàng)*.

### Terminal 3: Giám sát TF và JointState
```bash
cd /home/duy/VDT_project
source /opt/ros/humble/setup.bash
# Theo dõi góc TF liên tục giữa thân drone và camera:
ros2 run tf2_ros tf2_echo base_link camera_link
```

### Terminal 4: Phát lệnh kiểm tra các góc Pitch khác nhau
Lần lượt gõ từng lệnh, quan sát cả trong cửa sổ Gazebo lẫn Terminal 3:

**1. Kiểm tra góc nhìn ngang (0°):**
```bash
ros2 topic pub --once /model/x500_depth_0/command/gimbal_pitch std_msgs/msg/Float64 "{data: 0.0}"
ros2 topic echo /joint_states --once
```

**2. Kiểm tra góc chúc nhẹ -20° (-0.35 rad):**
```bash
ros2 topic pub --once /model/x500_depth_0/command/gimbal_pitch std_msgs/msg/Float64 "{data: -0.35}"
ros2 topic echo /joint_states --once
```

**3. Kiểm tra góc chúc sâu -60° (-1.047 rad):**
```bash
ros2 topic pub --once /model/x500_depth_0/command/gimbal_pitch std_msgs/msg/Float64 "{data: -1.047}"
ros2 topic echo /joint_states --once
```

**4. Kiểm tra góc gần thẳng đứng -80° (-1.396 rad) & -90° (-1.571 rad):**
```bash
ros2 topic pub --once /model/x500_depth_0/command/gimbal_pitch std_msgs/msg/Float64 "{data: -1.396}"
ros2 topic echo /joint_states --once
ros2 topic pub --once /model/x500_depth_0/command/gimbal_pitch std_msgs/msg/Float64 "{data: -1.571}"
ros2 topic echo /joint_states --once
```

### Bảng ghi chép đánh giá Test 0:
| Tiêu chí | Kết quả quan sát | Đạt (PASS) / Không (FAIL) |
|---|---|---|
| Lệnh âm (`-0.35`, `-1.05`, `-1.57`) làm camera trong Gazebo chúc xuống đất | | |
| Lệnh `0.0` đưa camera nhìn thẳng về phía trước | | |
| Giá trị `position` trong `/joint_states` khớp với lệnh gửi ($\pm 0.05$ rad) | | |
| `tf2_echo base_link camera_link` hiển thị góc quay quanh trục Pitch tương ứng | | |
| Không bị rung lắc (jitter) liên tục tại cơ cấu joint camera | | |

*Nếu command, `JointState` và TF không cùng dấu, dừng test và sửa frame/pitch trước. Code hiện tại cố ý dùng `pitch_tf=-pitch_val` trong [`launch_simulation.py`](../launch_simulation.py); không tự ý đảo dấu lần nữa trong IBVS.*

---

## 4. Test A — Kiểm tra yaw mà không có APF

> **Mục tiêu:** Kiểm tra khả năng xoay Yaw bám target của drone mà không bị nhiễu bởi chuyển động tịnh tiến của APF. Drone cất cánh và hover cố định tại `(0, 0, 3m)`. Ta di chuyển H-Pad sang 4 hướng (+X, +Y, -X, -Y) hoặc qua biên $\pm\pi$.

Trước hết có thể chạy unit test logic có sẵn:
```bash
python3 simulation_maps/test_c3_ibvs.py
```

### Terminal 1: PX4 SITL & Gazebo
```bash
./simulation_maps/start_px4_sim.sh
```


### Terminal 2: Cầu nối ROS 2 - Gazebo & RViz2
```bash
cd /home/duy/VDT_project
source /opt/ros/humble/setup.bash
export GZ_PARTITION=vdt_harmonic
python3 simulation_maps/launch_simulation.py
```

### Terminal 3: ArUco Detector & EKF Adapter
```bash
cd /home/duy/VDT_project
source /opt/ros/humble/setup.bash
python3 simulation_maps/aruco_sim_node.py &
python3 simulation_maps/ekf_ros_adapter.py
```

### Terminal 4: IBVS Controller & Offboard Commander (Kích hoạt FOLLOW)
```bash
cd /home/duy/VDT_project
source /opt/ros/humble/setup.bash
# Đặt phase FOLLOW độc lập (không cần chạy FSM) để khóa Yaw vào IBVS
ros2 topic pub -r 10 /mission/phase std_msgs/msg/String "{data: FOLLOW}" &
python3 simulation_maps/ibvs_controller.py &
python3 simulation_maps/offboard_commander.py
```

### Terminal 5: Cất cánh Drone lên 3.0m
```bash
cd /home/duy/VDT_project
source /opt/ros/humble/setup.bash
python3 simulation_maps/takeoff.py --alt 3.0
```
*(Chờ drone đạt 3.0m và hover ổn định)*.

### Terminal 6: Điều khiển bàn phím H-Pad
```bash
cd /home/duy/VDT_project
source /opt/ros/humble/setup.bash
python3 simulation_maps/hpad_keyboard_controller.py
```
*Thao tác lái H-Pad:*
- Lái H-Pad từ vị trí ban đầu `(5, 2)` sang phía trước `+X` $\rightarrow$ Yaw kỳ vọng $\approx 0^\circ$ ($0$ rad).
- Lái H-Pad sang trái `+Y` $\rightarrow$ Yaw kỳ vọng $\approx +90^\circ$ ($+1.57$ rad).
- Lái H-Pad sang phải `-Y` $\rightarrow$ Yaw kỳ vọng $\approx -90^\circ$ ($-1.57$ rad).
- Lái H-Pad vòng ra phía sau `-X` $\rightarrow$ Kiểm tra bước chuyển biên góc $\pm\pi$ ($\pm 3.14$ rad).

### Terminal 7: Giám sát đáp ứng Yaw (Kiểm tra hiện tượng lắc lư/giật)
```bash
cd /home/duy/VDT_project
source /opt/ros/humble/setup.bash
python3 simulation_maps/monitor_yaw.py
```
*(Script sẽ in liên tục 5 Hz: `[YAW] CMD: ...° | DRONE: ...° | ERR: ...°`)*.


### Bảng ghi chép đánh giá Test A:
| Tiêu chí | Kết quả quan sát | Đạt (PASS) / Không (FAIL) |
|---|---|---|
| Khi target vừa xuất hiện, drone quay về target mượt mà, không lắc lư qua lại | | |
| Target ở `+X, +Y, -X, -Y` cho Yaw drone khớp tương ứng ($0^\circ, 90^\circ, \pm 180^\circ, -90^\circ$) | | |
| Khi target vượt qua biên $\pm\pi$, drone quay theo góc ngắn nhất, không quay tròn cả vòng | | |
| Sai số góc bám `ERR` sau khi ổn định $< 5^\circ$ | | |

*Nếu cần chẩn đoán sâu hơn, có thể dùng profile giới hạn tốc độ quay trong `ibvs_controller`: `yaw_rate_limit: 0.20`, `search_yaw_rate: 0.10`.*

---

## 5. Test B — Xác minh camera có tự gập trong FOLLOW không

> **Mục tiêu:** Kiểm tra pipeline đầy đủ khi drone bay tịnh tiến từ xa lại gần mục tiêu:
> - Target đứng yên tại `(5.0, 2.0)`.
> - APF kéo drone lại gần target (khoảng cách 3D = 3.5m, khoảng cách ngang $\approx 1.84$m).
> - Kiểm tra xem Gimbal Pitch có tự động gập sâu từ $-20^\circ$ xuống $-58^\circ$ và hướng về $-80^\circ$ khi tới gần hay không.

### Terminal 1: PX4 SITL & Gazebo
```bash
./simulation_maps/start_px4_sim.sh
```


### Terminal 2: Chạy Toàn Bộ Autonomous Pipeline
```bash
cd /home/duy/VDT_project
source /opt/ros/humble/setup.bash
export GZ_PARTITION=vdt_harmonic
python3 simulation_maps/launch_simulation.py --autonomous
```
*(Launcher sẽ tự khởi chạy ArUco, EKF, APF, IBVS, FSM, Offboard Commander và RViz2)*.

### Terminal 3: Giám Sát Chi Tiết Pha Mission, EKF Mode và Góc Gimbal Pitch
```bash
cd /home/duy/VDT_project
source /opt/ros/humble/setup.bash
python3 simulation_maps/monitor_follow.py
```
*(Script sẽ in liên tục 4 Hz: `[PHASE] EKF: ... | Dist: ...m | Gimbal CMD: ...° | Geometric Req: ...°`)*.


### Terminal 4: Ra Lệnh Cất Cánh
```bash
cd /home/duy/VDT_project
source /opt/ros/humble/setup.bash
python3 simulation_maps/takeoff.py --alt 3.0
```

### Bảng ghi chép đánh giá Test B:
| Tình huống | Góc Pitch hình học kỳ vọng | Góc Pitch thực tế quan sát | Đạt (PASS) / Không (FAIL) |
|---|---|---|---|
| Khi drone vừa cất cánh (còn ở xa target ~5.4m) | $\approx -28^\circ \dots -35^\circ$ | | |
| Khi drone bay đến cự ly follow (khoảng cách ngang $\approx 1.84$m) | $\approx -58^\circ$ | | |
| Camera có tiếp tục gập sâu bám target không, hay bị khựng lại ở $-20^\circ$? | Không bị kẹt ở $-20^\circ$ | | |
| Khi drone tiếp cận gần target, hệ thống có bị rớt sang `SEARCH` và xoay tròn không? | Không rớt sang `SEARCH` | | |

### 5.1. Phân tích run Test B đã thu được

#### Điều cần sửa trong cách đọc log

`monitor_follow.py` đặt tên trường là `Dist`, nhưng giá trị được tính bằng khoảng cách ngang:

```python
d_h = hypot(drone_x - target_x, drone_y - target_y)
```

Đây không phải khoảng cách 3D. Khi drone ở độ cao khoảng `3.0 m` và target trên mặt đất:

```text
d_h = 1.97 m  →  slant_range ≈ sqrt(1.97² + 2.98²) ≈ 3.57 m
```

Do đó `Dist=1.97…2.33 m` vẫn phù hợp với `follow_distance=3.5 m`. Chưa có bằng chứng từ log này cho thấy bộ lọc `min_detection_distance=2.2 m` là nguyên nhân trực tiếp. Không hạ ngưỡng này chỉ dựa trên trường `Dist`; cần ghi thêm khoảng cách camera–marker và log `[OUTLIER REJECTED]`.

#### Những gì đã PASS một phần

- Ở đầu `FOLLOW`, `Gimbal CMD` đi từ khoảng `-25°` tới `-64°` và bám khá sát `Geometric Req`; nhánh feedforward pitch đang hoạt động.
- Khi `PREDICTING_DEGRADED`, command vẫn ở khoảng `-50.3°` và `Geometric Req` cũng khoảng `-50.0°`; controller vẫn đang tính đúng góc pitch theo EKF state.
- Gimbal không nhảy tức thời về `-20°`; nó giảm dần từ `-50°` về `-20°` do logic SEARCH hiện tại. Chuyển động này ít jerk hơn, nhưng vẫn sai hướng quan sát vì target cần khoảng `-50°`.

#### Nguyên nhân đã được xác nhận

Đoạn cuối của log có dạng:

```text
FOLLOW / PREDICTING_DEGRADED / CMD≈-50° / Req≈-50°
→ EXPIRED
→ SEARCH / CMD -43° ... -20° / Req vẫn≈-50°
```

Khi vào `SEARCH`, controller không còn dùng target state để giữ pitch mà kéo camera về `default_pitch=-20°`. Nếu marker đã ở vùng thấp của ảnh hoặc ngay dưới drone, camera sẽ rời khỏi marker; yaw search sau đó không thể bù lỗi pitch. Đây là nguyên nhân trực tiếp của chuỗi “mất tracking → SEARCH → quay vòng”.

#### Các nguyên nhân cần phân biệt thêm

1. **Góc thực tế của joint có thể không bằng command.** Monitor chỉ đọc `/ibvs/gimbal_pitch`, chưa đọc `JointState`. Cần so sánh command với `CameraJoint` thực tế.
2. **Detector có thể mất marker dù góc hình học đúng.** Cần xem `/hpad/bbox`, `/hpad/detected` và `/hpad/annotated`; log hiện tại chưa cho biết marker ra khỏi FOV hay ArUco/PnP bị loại.
3. **State target hoặc follow-goal có thể dao động trước khi mất dấu.** Dãy khoảng cách ngang `4.35 → 5.15 → 1.97 → 2.45 m` không hội tụ đơn điệu. Cần xem `/ekf/target_state`, `/apf/velocity_cmd` và ground truth để phân biệt EKF nhảy với drone overshoot.

### 5.1.1. Phân tích bag mới `vdt_testB_simtime_01`

Bag mới có thời lượng `182.25 s`, gồm `/clock` với thời gian mô phỏng khoảng `212.920…330.632 s`. Khác với bag trước, timestamp của camera và TF hiện đã cùng miền sim-time:

```text
/hpad/position_camera header.stamp ≈ 259.876…306.572 s
/tf header.stamp                 ≈ 213.776…325.096 s
```

Khoảng cách giữa timestamp measurement và TF gần nhất chỉ khoảng `0…0.04 s`. Vì vậy lỗi lệch epoch `1.79×10^9 s` đã được loại khỏi run này; không nên tiếp tục quy toàn bộ lỗi hiện tại cho `use_sim_time`.

Các dấu hiệu chính:

| Thời gian tương đối | Quan sát |
|---:|---|
| `61.4 s` | Bắt đầu có detection và EKF `TRACKING` |
| `64.6 s` | FSM vào `FOLLOW` |
| `67.7 s` | Detection mất; EKF chuyển `PREDICTING` |
| `70.6 s` | EKF `EXPIRED`, FSM chuyển `SEARCH` |
| `76.9 s`, `114.0 s`, `135.6 s` | Re-acquire nhiều lần nhưng sau đó lại mất tracking |
| `148.9 s` | FSM lại vào `SEARCH` |
| toàn bag | Có `163` measurement camera/bbox; không có message `/hpad/ground_truth` thực sự được ghi |

Khi dùng đúng timestamp để dựng lại phép biến đổi camera → world offline, các measurement nhìn thấy khá nhất quán quanh:

```text
x ≈ 5.0 m, y ≈ 0.6 m, z ≈ 0.0 m
```

Đây là bằng chứng target được detector nhìn thấy như một vật gần như đứng yên trong run. Tuy nhiên nó **không trùng** với pose SDF dự kiến `(5.0, 2.0, 0.02)`. Vì bag không có pose model trực tiếp từ Gazebo, chưa thể kết luận đây là target đã được di chuyển trước khi ghi bag hay là sai lệch frame/pose. Cần ghi pose model Gazebo trực tiếp trong run kế tiếp.

Vấn đề nghiêm trọng hơn là EKF không bám theo measurement đã biến đổi. Ngay khi bắt đầu tracking, `/ekf/target_state` đã ở khoảng `(4.71, 0.62, -0.91)`, sau đó trôi tới `(5.16, 0.65, -2.48)` rồi nhảy nhiều mét, trong khi measurement camera → world offline vẫn gần `(5.0, 0.6, 0.0)`. State có các bước nhảy lớn, ví dụ z thay đổi khoảng `2.7 m` trong một bước và các vận tốc đạt khoảng `1…2.7 m/s` dù target đứng yên.

Vì vậy trạng thái pipeline hiện tại là:

```text
measurement camera ổn định
→ EKF output không nhất quán với measurement
→ APF nhận target giả và phát vận tốc đổi hướng/sát giới hạn
→ yaw setpoint đổi mạnh
→ drone rời vùng FOV
→ detection mất, EKF EXPIRED, FSM SEARCH
```

`CameraJoint` trong bag bám command khá sát, với actual pitch đi trong khoảng xấp xỉ `-87°…-11°`. Do đó servo không phải nguyên nhân gốc của run này, dù logic SEARCH vẫn cần sửa sau khi EKF ổn định.

### 5.1.2. Việc cần sửa trước khi chạy lại

1. **Dọn node cũ trước mỗi run.** `start_px4_sim.sh` chỉ dừng PX4/Gazebo, không dừng các Python pipeline node. Nếu chạy launcher nhiều lần, có thể tồn tại nhiều `ekf_ros_adapter`, `mission_fsm`, APF hoặc IBVS cùng publish một topic. Kiểm tra trước khi chạy:

   ```bash
   pgrep -af 'launch_simulation|aruco_sim_node|ekf_ros_adapter|apf_planner|ibvs_controller|mission_fsm_node|offboard_commander'
   ```

   Chỉ dừng các process cũ thuộc project, rồi khởi động đúng một launcher. Không chạy `launch_simulation.py` lần hai trong cùng run.

2. **Ghi trực tiếp measurement sau phép đổi TF.** Trong `ekf_ros_adapter.py`, log hoặc publish thêm `world_pos`, timestamp measurement, timestamp TF được chọn và cờ `fallback_latest_tf`. Nếu world measurement không gần pose model Gazebo thì sửa frame/rotation trước khi đụng vào EKF.

3. **Không fallback sang TF mới nhất khi lookup theo timestamp thất bại.** Với measurement có timestamp hợp lệ, nếu không tìm được TF đúng thời điểm thì bỏ measurement và giữ prediction; không dùng latest TF để cập nhật target.

4. **Dùng một miền thời gian bên trong EKF.** Đã áp dụng: `initialize`, `predict`, `last_valid_meas_time` và tracking age trong [`ekf_ros_adapter.py`](../ekf_ros_adapter.py) dùng `msg.header.stamp` và `self.get_clock().now()`; measurement out-of-order bị loại thay vì trộn hai clock.

5. **Xác minh pose target trực tiếp từ Gazebo.** Trong lúc chạy, kiểm tra pose model `hpad_aruco` bằng Gazebo Transport và lưu kết quả. Không dùng `/hpad/ground_truth` làm chuẩn nếu topic này không có publisher trong bag.

6. **Chặn điều khiển khi EKF không đáng tin.** Trong `PREDICTING_DEGRADED` hoặc `EXPIRED`, tạm thời đặt vận tốc APF về zero/giới hạn rất thấp và giữ yaw cuối cùng hoặc search chậm. Không cho APF bám theo state có z ngoài miền bay hoặc position jump vượt ngưỡng.

Chỉ sau khi world measurement và EKF state cùng ổn định quanh một pose tĩnh trong ít nhất `10 s` mới tune `k_att`, `v_max`, `K_yaw` hoặc envelope pitch. Nếu tune APF trước, controller sẽ che khuất lỗi measurement/EKF.

### 5.1.3. Đánh giá hiện tượng drone nghiêng và file IAPF

Trong bag `vdt_testB_simtime_01`, attitude của drone trong `FOLLOW` nằm khoảng roll `-12…+12°` và pitch `-17…+14°`. Điều này phù hợp với việc APF đang phát lệnh vận tốc ngang; PX4 phải nghiêng roll/pitch để tạo gia tốc ngang. Không thể vừa yêu cầu drone dịch chuyển theo target vừa giữ roll/pitch đúng bằng `0°`; chỉ có thể giữ attitude gần phẳng trong pha căn yaw với vận tốc ngang bằng `0`.

Luồng hiện tại là:

```text
EKF target state → APF velocity x/y → Mission setpoint → PX4 velocity controller
                                                       → roll/pitch để bay ngang
```

Vì vậy cần phân biệt hai chiều nhân quả:

- Nếu TF dùng đúng attitude và đúng timestamp, drone nghiêng không tự nó làm EKF sai; attitude được dùng để đổi camera-frame sang world-frame.
- Trong bag này, camera measurement sau khi dựng lại khá ổn định nhưng EKF state nhảy mạnh. Do đó khả năng chính là EKF/state lỗi làm APF đổi hướng, rồi APF mới gây nghiêng/lắc. Cần log `world_pos` sau TF để xác nhận trên runtime.

File [`IAPF_Planner_3D_MultiObs.m`](../../../Avoidance/IAPF_Planner_3D_MultiObs.m) là **Improved APF planner**, không phải EKF. Nó có thể thay thế phần `APFCore.compute()` sau khi được port sang Python, nhưng không thể thay thế `TargetStateEKF` hoặc sửa lỗi coordinate transform. IAPF cũng dùng NED `[N,E,D]`, trong khi planner ROS hiện tại dùng ENU `[x,y,z]`; nếu tích hợp trực tiếp sẽ đảo/trộn trục. Ngoài ra IAPF chỉ tạo velocity/yaw, không điều khiển trực tiếp roll/pitch.

Thứ tự tích hợp an toàn:

1. Giữ EKF hiện tại và xác minh `world_pos` với pose model Gazebo.
2. Giới hạn APF hiện tại xuống `v_max≈0.3…0.5 m/s`, `max_accel≈0.3…0.6 m/s²` để kiểm tra state mà không làm drone giật mạnh.
3. Thêm chế độ **YAW_ALIGN**: khi bắt đầu tracking hoặc yaw error lớn hơn khoảng `10…15°`, đặt `vx=vy=0`, giữ độ cao, chỉ phát yaw command và gimbal pitch.
4. Chỉ mở lại vận tốc ngang bằng ramp sau khi yaw đã ổn định; tiếp tục giới hạn tốc độ, gia tốc và jerk.
5. Khi EKF đã ổn định, port IAPF sang Python với lớp chuyển đổi ENU↔NED, reset state persistent khi đổi phase/target, rồi so sánh APF cũ và IAPF trong cùng một bag.

Không nên thay EKF bằng IAPF. Nếu muốn cải tiến EKF, cần một thay đổi riêng ở `ekf_ros_adapter.py`/`TargetStateEKF`: dùng timestamp sensor nhất quán, loại measurement out-of-order, bỏ fallback latest TF và giới hạn target state trước khi chuyển cho APF.

### 5.2. Lệnh chẩn đoán bắt buộc cho lần chạy kế tiếp

```bash
ros2 topic echo /hpad/detected
ros2 topic echo /hpad/bbox
ros2 topic echo /hpad/position_camera
ros2 topic echo /joint_states
ros2 topic echo /apf/velocity_cmd
ros2 topic echo /ekf/target_state
ros2 run rqt_image_view rqt_image_view /hpad/annotated
```

Khi mất dấu, ghi lại cùng thời điểm: `phase`, `tracking_mode`, `detected`, actual `CameraJoint pitch`, `bbox center`, `/apf/velocity_cmd` và log `[OUTLIER REJECTED]`/`[KINEMATIC GATE]`.

| Kết quả tại thời điểm mất dấu | Kết luận |
|---|---|
| `detected=False`, ảnh không có marker, actual pitch lệch command | Lỗi joint/bridge/TF hoặc servo lag |
| `detected=False`, actual pitch≈command, marker nằm ngoài ảnh | Envelope/FOV hoặc dấu pitch chưa đúng |
| Marker vẫn thấy trong `/hpad/annotated` nhưng `detected=False` | Lỗi ArUco/PnP/filter; xem log detector |
| `detected=True` nhưng không có measurement hợp lệ, có `[KINEMATIC GATE]` | Raw detection đang bị dùng thay cho accepted EKF measurement |
| `/apf/velocity_cmd` đổi dấu liên tục trước khi mất dấu | APF/follow-goal thiếu damping hoặc EKF target bị rung |

### 5.3. Đề xuất cải thiện theo thứ tự ưu tiên

#### P0 — Không reset pitch về góc ngang khi vào SEARCH

Trong SEARCH, nên giữ `last_valid_pitch` trong một khoảng ngắn, sau đó quét chậm từ góc cuối cùng nhìn thấy target về vùng thấp, ví dụ:

```text
last_valid_pitch → -60° → -45° → -30°
```

Không đưa ngay camera từ `-50°…-65°` về `-20°`. Để kiểm chứng nhanh, có thể cho SEARCH giữ `-50°` hoặc sweep `[-60°, -30°]`; đây là test chẩn đoán, chưa phải tuning cuối.

#### P0 — Chỉ coi measurement được EKF chấp nhận là measurement hợp lệ

`/hpad/detected` chỉ biểu diễn raw visual detection. Đã sửa EKF để tracking age dùng `last_valid_meas_time`, chỉ cập nhật sau distance check, TF check và kinematic gate. Khi measurement bị gate, mode không còn chuyển thành `TRACKING` chỉ vì raw `detected=True`.

#### P1 — Tắt hoặc lọc yaw velocity feedforward trong test target đứng yên

`ibvs_controller.py` hiện dùng vận tốc EKF để tạo `yaw_ff` tới `±0.3 rad/s`. Nhiễu vận tốc của target đứng yên có thể tạo rung yaw. Test lại với `yaw_ff=0`, hoặc chỉ bật khi vận tốc đã vượt ngưỡng sau low-pass filter. Trong giai đoạn chẩn đoán dùng `yaw_rate_limit=0.20…0.30 rad/s`.

#### P1 — Thêm damping cho APF trước khi tăng tốc

Để kiểm tra riêng dao động APF/Offboard, dùng profile chậm:

```yaml
apf_planner:
  ros__parameters:
    v_max: 0.6
    k_att: 4.0
    d_slow: 1.0
    goal_threshold: 0.30

offboard_commander:
  ros__parameters:
    max_accel: 0.8
```

Nếu quỹ đạo ổn định và camera không mất dấu, tăng từng tham số một. Về lâu dài nên bổ sung low-pass/hysteresis cho follow-goal và thành phần damping theo vận tốc drone; không chỉ tăng `k_att`.

#### P1 — Sửa nhãn monitor

Đổi output thành `Horizontal Dist` và bổ sung `Slant Dist`, `actual_pitch`, `detected`, `bbox_u/v`, `|v_target|`, `|v_apf|`. Với run hiện tại, chỉ nhìn `Dist` rất dễ kết luận nhầm rằng drone đã tiến quá gần marker.

### 5.4. Runbook terminal cho lần test lại

Với code hiện tại, các terminal dưới đây dùng để **xác nhận nguyên nhân và thu log**. Riêng cải thiện P0 “giữ `last_valid_pitch` khi SEARCH” và P0 “accepted measurement” chưa thể được kiểm chứng đầy đủ chỉ bằng lệnh terminal; cần triển khai hai thay đổi đó trong code trước.

#### Terminal 1 — PX4 SITL và Gazebo

Chạy một lần duy nhất:

```bash
cd /home/duy/VDT_project
bash simulation_maps/start_px4_sim.sh
```

Không chạy lại terminal này trong lúc drone đang bay vì script có bước dừng các tiến trình `px4/gz-sim` cũ.

#### Terminal 1A — Bridge clock của Gazebo

Với source hiện tại, `launch_simulation.py` chưa tự bridge `/clock`. Chạy sau khi Gazebo đã khởi động và trước khi bật `use_sim_time`:

```bash
source /opt/ros/humble/setup.bash
export GZ_PARTITION=vdt_harmonic
ros2 run ros_gz_bridge parameter_bridge \
  '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'
```

Kiểm tra ở terminal phụ:

```bash
source /opt/ros/humble/setup.bash
ros2 topic echo /clock --once
```

Nếu không nhận được `/clock`, bridge chưa hoạt động hoặc `GZ_PARTITION` không trùng với Gazebo. Không tiếp tục test trong trạng thái đó; sau khi `/clock` có dữ liệu mới bật `use_sim_time` cho các node.

#### Terminal 2 — Autonomous pipeline

Đợi Gazebo và PX4 khởi động xong rồi chạy:

```bash
cd /home/duy/VDT_project
source /opt/ros/humble/setup.bash
export GZ_PARTITION=vdt_harmonic
python3 simulation_maps/launch_simulation.py --autonomous
```

#### Terminal 2A — Kiểm tra `use_sim_time` của toàn bộ node

Pipeline hiện đã được cấu hình bật `use_sim_time=true` ngay khi khởi động. Không nên bật trễ sau khi node đã chạy vì TF buffer có thể đã chứa wall-time. Sau khi Terminal 2 khởi động, chỉ kiểm tra:

```bash
source /opt/ros/humble/setup.bash
for node in fast_tracker_style_node aruco_sim_node ekf_ros_adapter \
  apf_planner ibvs_controller mission_fsm offboard_commander; do
  echo "--- ${node} ---"
  ros2 param get "/${node}" use_sim_time
done
```

Kiểm tra:

```bash
ros2 param get /fast_tracker_style_node use_sim_time
ros2 param get /ekf_ros_adapter use_sim_time
```

#### Terminal 3 — Ghi bag

Mở trước khi cất cánh:

```bash
cd /home/duy/VDT_project
source /opt/ros/humble/setup.bash
ros2 bag record -o /tmp/vdt_testB_yaw_align_04 \
  /clock \
  /odom /hpad/ground_truth /hpad/detected /hpad/bbox /hpad/position_camera \
  /ekf/target_state /ekf/tracking_mode \
  /apf/velocity_cmd /apf/yaw_cmd \
  /ibvs/yaw_cmd /ibvs/gimbal_pitch \
  /mission/phase /mission/velocity_setpoint /mission/yaw_setpoint \
  /joint_states /tf /tf_static
```

Chạy Terminal 3 sau khi `/clock` hoạt động và `use_sim_time` đã bật, nhưng trước Terminal 8 (cất cánh). Thứ tự là: Terminal 1 → Terminal 1A → Terminal 2 → Terminal 2A → Terminal 3 → Terminal 4–7 → Terminal 8.

Việc mở terminal mất vài giây không làm sai timestamp của message. Nó chỉ làm bag thiếu dữ liệu trước thời điểm recorder bắt đầu, hoặc monitor không hiển thị được các message đầu tiên. Không để drone cất cánh trước khi Terminal 3 bắt đầu ghi.

#### Tiêu chí đánh giá ba phương án đã triển khai

Trong lần test này, cấu hình mới chỉ có hiệu lực nếu khởi động lại toàn bộ pipeline sau khi sửa file YAML. Không chạy thêm một launcher thứ hai trên cùng hệ thống.

1. **Phương án 1 — timestamp/TF/EKF:** Terminal 2 phải xuất hiện log `[WORLD_MEAS]` với `cam_stamp` và `tf_stamp` gần nhau. Không được có chuỗi `[TF REJECTED]` liên tục khi detector đang hoạt động. Với target đứng yên, `world=(x,y,z)` và `/ekf/target_state` phải hội tụ, không nhảy nhiều mét hoặc tạo vận tốc lớn kéo dài.
2. **Phương án 2 — giảm động lực APF:** `/apf/velocity_cmd` không vượt khoảng `0.4 m/s`; `offboard_commander` phải giới hạn thay đổi vận tốc theo `max_accel=0.4 m/s²`. Drone có thể nghiêng khi bay ngang, nhưng không được giật mạnh hoặc đảo vận tốc liên tục.
3. **Phương án 3 — YAW_ALIGN:** khi vừa chuyển sang `FOLLOW`, phải thấy log `[YAW_ALIGN] HOLD XY`; `/mission/velocity_setpoint` giữ `vx=vy=0` trong lúc yaw error còn lớn. Khi sai số yaw xuống dưới `7°`, phải thấy `[YAW_ALIGN] RELEASE XY` và vận tốc ngang mới tăng dần. Nếu EKF là `PREDICTING`, `PREDICTING_DEGRADED` hoặc `EXPIRED`, APF phải về zero và FSM giữ XY.

Kết quả PASS tối thiểu của Test B: target đứng yên vẫn còn trong ảnh khi drone bắt đầu FOLLOW; không chuyển SEARCH chỉ vì một cú giật yaw; attitude trong giai đoạn căn yaw gần hover; sau khi release, drone tiến chậm và không quay vòng. Nếu fail, giữ nguyên bag và log `[WORLD_MEAS]`, `[YAW_ALIGN]`, `/apf/velocity_cmd`, `/mission/velocity_setpoint`, `/odom` để đối chiếu.

#### Terminal 4 — Monitor FOLLOW

```bash
cd /home/duy/VDT_project
source /opt/ros/humble/setup.bash
python3 simulation_maps/monitor_follow.py
```

Nhớ đọc `Dist` là khoảng cách ngang. Đối chiếu thêm target/drone trên bag để tính khoảng cách xiên.

#### Terminal 5 — Xem ảnh camera

```bash
source /opt/ros/humble/setup.bash
ros2 run rqt_image_view rqt_image_view /hpad/annotated
```

Khi mất tracking, không đóng cửa sổ này; cần xác định marker còn trong ảnh hay không.

#### Terminal 6 — Joint pitch thực tế

```bash
source /opt/ros/humble/setup.bash
ros2 topic echo /joint_states
```

Theo dõi `CameraJoint`. So sánh góc này với `/ibvs/gimbal_pitch`; command `-50°` nhưng actual joint không gần `-50°` là lỗi bridge/joint/servo.

#### Terminal 7 — Detection, EKF và APF

Mở ba terminal nhỏ hoặc chạy từng lệnh ở các terminal riêng:

```bash
ros2 topic echo /hpad/detected
ros2 topic echo /ekf/tracking_mode
ros2 topic echo /apf/velocity_cmd
```

Nếu có `[OUTLIER REJECTED]` hoặc `[KINEMATIC GATE]` đúng lúc mất dấu, ghi lại timestamp tương ứng.

#### Terminal 8 — Cất cánh

Chỉ chạy sau khi Terminal 2–7 đã hoạt động:

```bash
cd /home/duy/VDT_project
source /opt/ros/humble/setup.bash
python3 simulation_maps/takeoff.py --alt 3.0
```

Với Test B, chưa chạy `hpad_keyboard_controller.py`; để target đứng yên. Chờ phase `FOLLOW`, quan sát đến khi horizontal distance gần `1.84 m`, rồi chờ hệ thống ổn định.

Sau khi chạy xong, lưu lại bag bằng lệnh:

```bash
mkdir -p /home/duy/VDT_project/recordings
cp -a /tmp/vdt_testB_yaw_align_04 \
  /home/duy/VDT_project/recordings/testB_yaw_align_04
```

#### Terminal 9 — Chỉ dùng sau khi Test B tĩnh PASS

```bash
cd /home/duy/VDT_project
source /opt/ros/humble/setup.bash   
python3 simulation_maps/hpad_keyboard_controller.py
```

Terminal này chỉ dùng cho Test D target di động, không dùng trong lần xác nhận đầu tiên của Test B.

### 5.5. Profile chẩn đoán chậm

Sau khi đã ghi một run baseline, có thể chỉnh `simulation_maps/config/mission_params.yaml` trước khi khởi động lại Terminal 2:

```yaml
apf_planner:
  ros__parameters:
    v_max: 0.6
    k_att: 4.0
    d_slow: 1.0
    goal_threshold: 0.30

ibvs_controller:
  ros__parameters:
    yaw_rate_limit: 0.25
    pitch_rate_limit: 1.0
    pitch_ema_alpha: 0.35

offboard_commander:
  ros__parameters:
    max_accel: 0.8
```

Profile này chỉ làm chuyển động chậm và giảm rung để chẩn đoán APF/yaw. Nó **không sửa** việc SEARCH kéo pitch về `-20°` và **không sửa** việc raw detection có thể bị dùng thay cho accepted measurement.

---

## 6. Test C — Mất dấu với target đứng yên

> **Mục tiêu:** Kiểm tra hành vi chuyển đổi trạng thái khi mục tiêu đột ngột bị che khuất trong 1–2 giây:
> 1. EKF có chuyển đúng trình tự: `TRACKING` $\rightarrow$ `PREDICTING` $\rightarrow$ `PREDICTING_DEGRADED` $\rightarrow$ `EXPIRED` không?
> 2. Trong lúc `PREDICTING`, gimbal có giữ nguyên góc hướng về vị trí cũ thay vì giật nảy về $-20^\circ$ không?
> 3. Khi bỏ che, drone có re-acquire target ngay tại chỗ không, hay bị kẹt ở góc pitch quét ngang của `SEARCH`?

### Cách thực hiện che khuất giả lập:
Sau khi drone đang ở pha `FOLLOW` ổn định (chạy như Bài Test B):
Tại một Terminal phụ, tạm dừng node ArUco trong 2 giây rồi cho chạy tiếp:
```bash
# Lệnh tạm dừng node aruco (SIGSTOP) để giả lập mất hình ảnh:
killall -STOP aruco_sim_node.py
# Đợi 2 giây, sau đó cho node chạy tiếp (SIGCONT):
sleep 2.0 && killall -CONT aruco_sim_node.py
```

### Terminal Giám Sát Trạng Thái (Mở theo dõi):
```bash
cd /home/duy/VDT_project
source /opt/ros/humble/setup.bash
ros2 topic echo /ekf/tracking_mode
```

### Bảng ghi chép đánh giá Test C:
| Tiêu chí | Kết quả quan sát | Đạt (PASS) / Không (FAIL) |
|---|---|---|
| Khi bị che trong 1s, EKF chuyển sang `PREDICTING` | | |
| Trong lúc `PREDICTING`, Gimbal không bị reset giật nảy về $-20^\circ$ | | |
| Khi bỏ che sau 1.5s, EKF chuyển lại ngay `TRACKING` (hoặc `[EKF RE-ACQUIRED]`) | | |
| Drone không bị rơi vào `SEARCH` quay vòng tròn khi mục tiêu vẫn ở ngay dưới | | |

*Nếu target chỉ tìm lại được khi H-Pad được đưa ra xa, đây là dấu hiệu của envelope `SEARCH [-60°, -10°]` và reset `default_pitch=-20°`.*

---

## 7. Test D — Mất dấu rồi re-acquire target di động

> **Mục tiêu:** Kiểm tra hiện tượng giật góc Yaw khi target vừa xuất hiện lại sau một khoảng thời gian ngắn bị che khuất trong khi đang di chuyển.

### Terminal 1, 2, 3, 4:
Chạy hệ thống đầy đủ như trong **Bài Test B** (Gazebo, `--autonomous`, Takeoff, và script Monitor).

### Terminal 5: Điều Khiển H-Pad Di Chuyển
```bash
cd /home/duy/VDT_project
source /opt/ros/humble/setup.bash
python3 simulation_maps/hpad_keyboard_controller.py
```

### Tiến hành Test D1 (Target di chuyển chậm):
1. Bấm giữ phím `W` để H-pad chạy thẳng với tốc độ chậm ($0.5$ m/s).
2. Khi drone đang bay bám theo, tại Terminal 6 gõ lệnh che mục tiêu trong 0.8 giây:
   ```bash
   killall -STOP aruco_sim_node.py && sleep 0.8 && killall -CONT aruco_sim_node.py
   ```
3. Ghi lại thời điểm xuất hiện các log `[KINEMATIC GATE]` và `[EKF RE-ACQUIRED]`.
4. Quan sát:
   - Terminal log EKF có in `[KINEMATIC GATE] Rejected jump` không?
   - Tại frame đầu tiên nhìn lại thấy mục tiêu, góc Yaw của drone có bị xoay giật đột ngột không?

### Tiến hành Test D2 (Target di chuyển nhanh hơn):
1. Bấm phím `+` trên bàn phím điều khiển H-pad để tăng tốc độ lên $1.2$ m/s.
2. Lặp lại thao tác che mục tiêu 0.8 giây:
   ```bash
   killall -STOP aruco_sim_node.py && sleep 0.8 && killall -CONT aruco_sim_node.py
   ```
3. Quan sát xem drone có bám mượt tiếp theo target hay bị mất dấu hoàn toàn và xoay vòng.

### Bảng ghi chép đánh giá Test D1 & D2:
| Tiêu chí | Test D1 (Chậm) | Test D2 (Nhanh) | Đạt (PASS) / Không (FAIL) |
|---|---|---|---|
| Góc Yaw không bị giật/bước nhảy đột ngột ở frame đầu tiên re-acquire | | | |
| Trong thời gian measurement bị gate, hệ thống không coi state cũ là mới | | | |
| Không bị lặp chuỗi `FOLLOW` $\rightarrow$ `SEARCH` $\rightarrow$ `FOLLOW` liên tục | | | |
| Không xuất hiện hiện tượng drone tự quay vòng tròn sau khi marker đã nằm lại trong FOV | | | |
| Quỹ đạo bay tiếp tục bám theo đường di chuyển của H-pad | | | |

Nếu thấy hiện tượng giật, phân loại theo log:

| Quan sát | Nguyên nhân ưu tiên |
|---|---|
| `[KINEMATIC GATE]` lặp, `tracking_mode` vẫn như đang tracking | Cờ detected đang được dùng thay cho accepted measurement |
| `[EKF RE-ACQUIRED]` xuất hiện cùng lúc yaw đổi mạnh | State được khởi tạo lại lệch bearing; cần giữ yaw/pixel fallback trong thời gian xác nhận |
| `target_state` nhảy khi drone/gimbal quay | TF hoặc timestamp camera không đồng bộ |
| `ibvs/yaw_cmd` mượt nhưng `/odom` yaw giật | PX4 yaw response/mapping hoặc bộ điều khiển attitude |
| `ibvs/yaw_cmd` tự nhảy | EKF/world bearing, pixel trim hoặc nguồn input không ổn định |
| Target đứng yên vẫn vào SEARCH | FOV/pitch/TF, không nên tiếp tục tune APF |

---

## 8. Kiểm tra frame và timestamp nếu yaw vẫn giật

`aruco_sim_node.py` đã truyền timestamp của ảnh vào `PointStamped`. `ekf_ros_adapter.py` cố gắng lookup TF đúng timestamp, nhưng vẫn có fallback về TF mới nhất nếu bị extrapolation. Khi drone/gimbal đang quay nhanh, fallback này có thể làm world position nhảy.

Kiểm tra:

```bash
ros2 topic echo /hpad/position_camera --once
ros2 run tf2_ros tf2_echo world camera_optical_frame
ros2 run tf2_tools view_frames
```

Trong bag, so sánh timestamp của `/hpad/position_camera` với `/tf`. Không chấp nhận:

- hai publisher cùng phát một transform;
- `camera_optical_frame` không nối qua `camera_link`;
- target world-frame thay đổi lớn khi target và drone đứng yên, chỉ vì thay đổi pitch;
- target world-frame đổi hướng mạnh hơn vận tốc target cho phép.

---

## 9. Thứ tự xử lý sau khi test

Ưu tiên theo thứ tự này:

1. **PASS Test 0:** Dấu pitch, joint feedback, TF.
2. **PASS Test A:** Yaw cardinal và wrap-around, không APF.
3. **PASS Test B:** Pitch follow đạt góc hình học và không bị giới hạn sai pha.
4. **PASS Test C:** Target đứng yên mất dấu rồi tìm lại.
5. **PASS Test D1:** Target di động chậm.
6. **PASS Test D2:** Target di động nhanh.
7. Cuối cùng mới tăng tốc độ target, giảm khoảng cách follow hoặc bật đầy đủ vật cản APF.

*Lưu ý: Không dùng `ros2 topic echo /apf/yaw_cmd` để kết luận yaw đã được gửi tới PX4; đường yaw thực tế của full pipeline là `/ibvs/yaw_cmd → FSM → /mission/yaw_setpoint`. Khi một test fail, lưu bag và ghi lại phase, tracking mode, pitch command/thực tế, yaw command/thực tế và các log EKF trước khi thay đổi tham số.*
