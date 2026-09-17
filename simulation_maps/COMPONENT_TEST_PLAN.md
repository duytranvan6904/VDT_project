# Kế hoạch kiểm thử tách rời APF và Target Tracking/Servo

## 1. Mục tiêu

Tách pipeline thành các bài test nhỏ để xác định chính xác lỗi nằm ở:

1. Tính toán APF và điểm đích bám.
2. Biến đổi tọa độ camera → world → PX4.
3. Ước lượng vị trí target bằng EKF.
4. IBVS/gimbal pitch.
5. Lệnh yaw và bộ điều khiển bay.

Không chạy toàn bộ `--autonomous` cho đến khi từng nhóm test bên dưới đạt PASS.

---

## 2. Nhận định nguyên nhân hiện tại

Hiện tượng đang thấy không nên chỉ quy về gain APF. Có bốn điểm nghi ngờ cần ưu tiên:

### P0. Measurement ArUco không được transform theo timestamp của ảnh

Trong [`aruco_sim_node.py`](aruco_sim_node.py), pose được gắn timestamp bằng thời điểm publish hiện tại thay vì timestamp của frame camera. Sau đó [`ekf_ros_adapter.py`](ekf_ros_adapter.py) gọi:

```python
lookup_transform(world, camera_optical_frame, rclpy.time.Time())
```

`Time()` nghĩa là lấy TF mới nhất, không phải TF tại thời điểm ảnh được chụp. Khi drone hoặc gimbal đang quay, target world-frame sẽ bị tính bằng pose mới hơn ảnh. Kết quả là target có thể nhảy quanh drone, làm IBVS đổi bearing liên tục và gây xoay vòng.

### P0. Dấu pitch của joint Gazebo và TF broadcaster có khả năng ngược nhau

Trong model Gazebo, `CameraJoint` có trục:

```xml
<xyz>0 -1 0</xyz>
```

Nhưng [`launch_simulation.py`](launch_simulation.py) broadcast `camera_link` bằng quaternion quay quanh `+Y` với đúng giá trị `camera_pitch`. Nếu command gimbal được hiểu theo trục `-Y`, TF ROS có thể đang mô tả camera quay ngược chiều so với camera thật trong Gazebo. EKF khi đó nhận target ở vị trí sai ngay khi pitch thay đổi.

### P0. Cần xác nhận lại ENU/NED bằng test cardinal direction

APF tạo velocity trong world ENU. [`offboard_commander.py`](offboard_commander.py) đổi:

```text
vx_world → ve_NED
vy_world → vn_NED
vz_world → -vd_NED
```

và đổi yaw bằng `yaw_ned = pi/2 - yaw_world`. Nếu giả định trục local NED của PX4 không trùng với mapping này, drone sẽ đi lệch hướng và yaw không tương ứng với camera. Không được kết luận mapping đúng chỉ từ công thức; phải kiểm thử bằng lệnh +X/+Y độc lập.

### P1. FOLLOW hiện không nhắm đúng tâm target theo thiết kế

APF hiện dùng [`make_follow_goal()`](apf_planner.py), tức là tạo một điểm bám cách target 3.5 m theo phương ngang. Vì vậy drone không được kỳ vọng bay tới đúng tọa độ H-Pad trong FOLLOW. Đây là chủ ý để giữ khoảng cách, nhưng cần phân biệt rõ với lỗi APF.

### P1. IBVS yaw đang dùng bearing world-frame của EKF

Trong [`ibvs_controller.py`](ibvs_controller.py), yaw mục tiêu được tính từ:

```python
atan2(target_world_y - drone_world_y,
      target_world_x - drone_world_x)
```

Sau đó chỉ thêm một hiệu chỉnh pixel nhỏ. Nếu target world-frame sai do P0, `/ibvs/yaw_cmd` cũng sai dù pixel detector vẫn nhận diện đúng target.

---

## 3. Quy tắc chung khi test

- Mỗi test chỉ bật đúng các node cần thiết.
- Không tune gain khi chưa xác minh frame, timestamp và dấu pitch.
- Mỗi run dùng một thư mục log riêng.
- Ghi lại phase, tracking mode, target, odometry, pitch, yaw và command.
- Khi có thể, dùng target tĩnh trước rồi mới dùng H-Pad di động.

Chuẩn bị terminal:

```bash
cd /home/duy/VDT_project
source /opt/ros/humble/setup.bash
export GZ_PARTITION=vdt_harmonic
```

Topic nên record:

```bash
ros2 bag record -o /tmp/vdt_component_test \
  /odom /hpad/position_camera /hpad/bbox /hpad/detected \
  /ekf/target_state /ekf/tracking_mode \
  /apf/velocity_cmd /ibvs/yaw_cmd /ibvs/gimbal_pitch \
  /mission/phase /mission/velocity_setpoint /mission/yaw_setpoint \
  /tf /tf_static
```

---

## 4. Test 0 — Kiểm tra TF và timestamp trước khi test thuật toán

### 4.1. Kiểm tra frame và clock

Khởi động PX4/Gazebo và launcher cơ sở, không dùng `--autonomous`:

```bash
source /opt/ros/humble/setup.bash
export GZ_PARTITION=vdt_harmonic
python3 /home/duy/VDT_project/simulation_maps/launch_simulation.py
```

Kiểm tra:

```bash
ros2 run tf2_tools view_frames
ros2 run tf2_ros tf2_echo world base_link
ros2 run tf2_ros tf2_echo base_link camera_link
ros2 run tf2_ros tf2_echo camera_link camera_optical_frame
```

PASS khi cây TF chỉ có một chuỗi hợp lệ:

```text
world → base_link → camera_link → camera_optical_frame
```

và không có hai publisher cùng broadcast cùng một cặp frame.

### 4.2. Kiểm tra dấu pitch

Gửi lần lượt hai góc cố định:

```bash
ros2 topic pub --once \
  /model/x500_depth_0/command/gimbal_pitch \
  std_msgs/msg/Float64 "{data: 0.30}"

ros2 run tf2_ros tf2_echo base_link camera_link

ros2 topic pub --once \
  /model/x500_depth_0/command/gimbal_pitch \
  std_msgs/msg/Float64 "{data: -0.30}"

ros2 run tf2_ros tf2_echo base_link camera_link
```

PASS khi:

- command `+0.30` làm camera quay đúng chiều dương đã quy ước;
- command `-0.30` làm camera quay chiều ngược lại;
- góc đo từ Gazebo và góc trong TF có cùng dấu, sai số nhỏ hơn khoảng `0.05 rad`.

Nếu Gazebo quay một chiều nhưng TF quay chiều ngược lại, đây là nguyên nhân P0 cần sửa trước các test tracking.

### 4.3. Kiểm tra target world-frame với drone/gimbal đứng yên

Chạy riêng:

```bash
python3 simulation_maps/aruco_sim_node.py
python3 simulation_maps/ekf_ros_adapter.py
```

Để H-Pad đứng yên, không quay drone và không đổi pitch. Ghi:

```bash
ros2 topic echo /hpad/position_camera
ros2 topic echo /ekf/target_state
```

PASS khi target world-frame hội tụ ổn định. Sau đó chỉ đổi pitch gimbal, không đổi vị trí H-Pad.

PASS tiếp theo khi target world-frame vẫn gần như cố định. Nếu target world-frame di chuyển mạnh chỉ vì đổi pitch, lỗi nằm ở TF pitch hoặc timestamp.

---

## 5. Test nhóm A — APF độc lập, không ROS/PX4

Mục tiêu là chứng minh toán APF đúng trước khi đưa vào node ROS.

### A1. Test điểm bám và độ cao FOLLOW

Kiểm tra cự ly 3D từ drone tới marker thực tế đạt đúng 3.5 m (với độ cao drone $Z=3.0$ m, cự ly ngang chiếu đất $R_{xy} = \sqrt{3.5^2 - (3.0 - 0.02)^2} \approx 1.84$ m):

```bash
python3 - <<'PY'
import numpy as np
from simulation_maps.apf_planner import APFCore, APFParams, make_follow_goal

drone = np.array([0.0, 0.0, 3.0])
target = np.array([5.0, 2.0, 0.02])
goal = make_follow_goal(drone, target, 3.5)
result = APFCore(APFParams(v_max=1.5, k_att=10.0)).compute(drone, goal, [])

print('goal:', goal)
print('velocity:', result.velocity)
print('yaw:', result.yaw_cmd)
print('3D dist to target:', np.linalg.norm(goal - target))
print('Horizontal dist to target:', np.linalg.norm(goal[:2] - target[:2]))

assert np.isclose(goal[2], drone[2])
assert np.isclose(np.linalg.norm(goal - target), 3.5)
assert np.isclose(np.linalg.norm(goal[:2] - target[:2]), np.sqrt(3.5**2 - (3.0 - 0.02)**2))
assert np.isclose(result.velocity[2], 0.0)
assert result.velocity[0] > 0.0 and result.velocity[1] > 0.0
PY
```

Kết quả kỳ vọng gần:

```text
goal ≈ [3.30, 1.32, 3.00]
velocity ≈ [1.39, 0.56, 0.00] m/s
3D dist to target = 3.50 m
Horizontal dist to target ≈ 1.84 m
```

Nếu test này fail thì chưa cần chạy Gazebo; lỗi nằm ở APF core hoặc cách tạo follow goal.

### A2. Test bốn hướng cardinal

Giữ drone ở `[0, 0, 3]`, lần lượt đặt target ở 4 hướng chính. Khoảng cách 3D từ Goal tới Target luôn đạt đúng 3.5 m:

| Target XY | Hướng velocity kỳ vọng | Yaw kỳ vọng | 3D Dist | Horizontal Dist |
|---|---|---|---|---|
| `[+5, 0]` | `+X`, `z=0` | `0.0°` | `3.5 m` | `~1.84 m` |
| `[0, +5]` | `+Y`, `z=0` | `+90.0°` | `3.5 m` | `~1.84 m` |
| `[-5, 0]` | `-X`, `z=0` | `180.0°` | `3.5 m` | `~1.84 m` |
| `[0, -5]` | `-Y`, `z=0` | `-90.0°` | `3.5 m` | `~1.84 m` |

Chạy lệnh test tự động kiểm tra 4 hướng:

```bash
python3 - <<'PY'
import numpy as np
from simulation_maps.apf_planner import APFCore, APFParams, make_follow_goal

drone = np.array([0.0, 0.0, 3.0])
core = APFCore(APFParams(v_max=1.5, k_att=10.0))

test_cases = [
    ("East (+X)",  np.array([5.0, 0.0, 0.02]),  [1.0, 0.0, 0.0], 0.0),
    ("North (+Y)", np.array([0.0, 5.0, 0.02]),  [0.0, 1.0, 0.0], np.pi / 2),
    ("West (-X)",  np.array([-5.0, 0.0, 0.02]), [-1.0, 0.0, 0.0], np.pi),
    ("South (-Y)", np.array([0.0, -5.0, 0.02]), [0.0, -1.0, 0.0], -np.pi / 2),
]

for name, target, expected_dir, expected_yaw in test_cases:
    goal = make_follow_goal(drone, target, follow_distance=3.5)
    res = core.compute(drone, goal, obstacles=[])
    
    v_norm = np.linalg.norm(res.velocity)
    v_dir = res.velocity / v_norm if v_norm > 1e-6 else np.zeros(3)
    dist_3d = np.linalg.norm(goal - target)
    dist_xy = np.linalg.norm(goal[:2] - target[:2])
    
    print(f"[{name}] Target: {target[:2]} -> Goal: {goal[:2]} | 3D dist={dist_3d:.2f}m, XY dist={dist_xy:.2f}m | Vel: {res.velocity[:2]} | Yaw: {np.degrees(res.yaw_cmd):.1f}°")
    
    assert np.isclose(dist_3d, 3.5, atol=1e-5), f"{name}: 3D distance to marker must be 3.5m"
    assert np.isclose(res.velocity[2], 0.0, atol=1e-5), f"{name}: vz should be 0"
    assert np.allclose(v_dir, expected_dir, atol=1e-3), f"{name}: unexpected velocity direction {v_dir}"
    assert np.isclose(v_norm, 1.5, atol=1e-3), f"{name}: should cap at v_max=1.5"

print("\n>>> ALL 4 CARDINAL DIRECTIONS PASSED! <<<")
PY
```

Không có obstacle trong test này. Sai hướng ở đây là lỗi vector hoặc frame, không phải lỗi PX4.

### A3. Test ảnh hưởng obstacle

Dùng các obstacle cố định để kiểm tra:

1. `|velocity| <= v_max`;
2. Vector tổng bị lệch khỏi vector hút thuần (xuất hiện thành phần vận tốc vuông góc để né cản);
3. Khi obstacle ở phía trên drone, xác định rõ `velocity.z` có thay đổi hay không (kỳ vọng: $v_z < 0$ để hạ độ cao né);
4. Không dùng yaw để đánh giá APF ở bước này, chỉ đánh giá `velocity` và `f_total`.

Chạy lệnh test tự động kiểm tra ảnh hưởng của vật cản:

```bash
python3 - <<'PY'
import numpy as np
from simulation_maps.apf_planner import APFCore, APFParams, make_follow_goal

print("=== BẮT ĐẦU TEST A3: ẢNH HƯỞNG CỦA VẬT CẢN (OBSTACLE) ===")

drone = np.array([0.0, 0.0, 3.0])
target = np.array([5.0, 0.0, 0.02])
goal = make_follow_goal(drone, target, 3.5)
v_max = 1.5
core = APFCore(APFParams(v_max=v_max, k_att=10.0, k_rep=2500.0, d0=2.5))

# 1. Baseline: Đường bay trống không có vật cản
res_clean = core.compute(drone, goal, obstacles=[])
print(f"\n1. Baseline (Không vật cản):")
print(f"   Vel: {res_clean.velocity} | |v| = {np.linalg.norm(res_clean.velocity):.2f} m/s")
assert np.isclose(res_clean.velocity[2], 0.0)
assert res_clean.velocity[0] > 0.0 and np.isclose(res_clean.velocity[1], 0.0)

# 2. Vật cản nằm chắn ngang đường bay [1.5, 0.2, 3.0]
obs_front = [(1.5, 0.2, 3.0)]
res_front = core.compute(drone, goal, obstacles=obs_front)
v_front = res_front.velocity
norm_front = np.linalg.norm(v_front)
print(f"\n2. Vật cản chắn ngang [1.5, 0.2, 3.0]:")
print(f"   F_att:   {res_front.f_att}")
print(f"   F_rep:   {res_front.f_rep}")
print(f"   F_total: {res_front.f_total}")
print(f"   Vel:     {v_front} | |v| = {norm_front:.2f} m/s")

# Tiêu chí:
assert norm_front <= v_max + 1e-4, f"Vận tốc vượt quá v_max: {norm_front}"
assert abs(v_front[1]) > 0.1, "Vector vận tốc chưa bị lệch để né vật cản!"
assert np.isclose(v_front[2], 0.0, atol=1e-4), f"vz phải bằng 0 khi cản ngang: {v_front[2]}"

# 3. Vật cản nằm ngay phía trên đầu drone [0.0, 0.0, 4.0]
obs_above = [(0.0, 0.0, 4.0)]
res_above = core.compute(drone, goal, obstacles=obs_above)
v_above = res_above.velocity
norm_above = np.linalg.norm(v_above)
print(f"\n3. Vật cản ở phía trên đầu drone [0.0, 0.0, 4.0]:")
print(f"   F_rep:   {res_above.f_rep}")
print(f"   Vel:     {v_above} | vz = {v_above[2]:.2f} m/s | |v| = {norm_above:.2f} m/s")

# Tiêu chí:
assert norm_above <= v_max + 1e-4
assert v_above[2] < -0.1, f"vz phải âm (hạ độ cao né cản trên đầu): {v_above[2]}"

print("\n>>> TẤT CẢ TIÊU CHÍ CỦA TEST A3 ĐỀU ĐẠT CHUẨN (PASSED)! <<<")
PY
```

Kết quả kỳ vọng:
- Khi có vật cản phía trước: `F_rep` xuất hiện lực đẩy ngược và thành phần tiếp tuyến, làm vector vận tốc lệch sang bên ($v_y \ne 0$) để lách qua;
- Khi có vật cản trên đầu: $v_z < 0$ (lực đẩy ép drone bay thấp xuống né cản);
- Ở mọi trường hợp, tổng độ lớn vận tốc luôn được khống chế $\le v_{\text{max}} = 1.5\text{ m/s}$.

---

## 6. Test nhóm B — APF ROS node độc lập

### B1. APF ROS Topic Interface & Bám mục tiêu Cardinal (Không vướng cản)

Không chạy EKF, IBVS, FSM hoặc Offboard. Dùng các topic giả lập bằng `ros2 topic pub`.

Khởi động node:

```bash
python3 simulation_maps/apf_planner.py \
  --ros-args \
  --params-file simulation_maps/config/mission_params.yaml \
  -p world_sdf:=simulation_maps/gazebo_worlds/obstacle_avoidance.sdf
```

Terminal khác, publish phase và tracking mode:

```bash
ros2 topic pub -r 10 /mission/phase std_msgs/msg/String "{data: FOLLOW}"
ros2 topic pub -r 10 /ekf/tracking_mode std_msgs/msg/String "{data: TRACKING}"
```

Publish odometry drone cố định:

```bash
ros2 topic pub -r 10 /odom nav_msgs/msg/Odometry \
  "{pose: {pose: {position: {x: 0.0, y: 0.0, z: 3.0}}}}"
```

Publish target:

```bash
ros2 topic pub -r 10 /ekf/target_state nav_msgs/msg/Odometry \
  "{pose: {pose: {position: {x: 5.0, y: 2.0, z: 0.02}}}}"
```

Quan sát:

```bash
ros2 topic echo /apf/velocity_cmd
ros2 topic echo /apf/force_markers
```

PASS khi:

- FOLLOW không tạo lệnh hút theo `z` của target ($v_z = 0$);
- hướng velocity hướng tới điểm bám 3D `[3.30, 1.32, 3.0]` (với vận tốc $\approx [1.39, 0.56, 0.00]$ m/s);
- chuyển `tracking_mode` sang `PREDICTING_DEGRADED` hoặc `EXPIRED` làm velocity về 0;
- đổi target theo bốn hướng cardinal cho kết quả đúng như A2.

Nếu APF topic đúng nhưng drone bay sai hướng, lỗi nằm sau APF: FSM, ENU/NED hoặc PX4.

---

### B2. Test trực quan né vật cản Gazebo trên RViz2 (Visual Inspection - Cách 1)

**Mục đích:**
Kiểm tra trực quan 100% logic né vật cản 3D của thuật toán APF dựa trên map thực tế của Gazebo (`obstacle_avoidance.sdf`), hiển thị trực tiếp các cột vật cản, điểm Goal bám theo H-Pad, vị trí Drone và các vector lực trên giao diện đồ họa không gian 3D của RViz2 mà **không cần khởi động PX4 bay thật**, không sợ rơi hay va chạm hỏng hóc.

#### 1. Kịch bản bố trí không gian (Scenario Setup)
Trong file world Gazebo `simulation_maps/gazebo_worlds/obstacle_avoidance.sdf`:
- **Vật cản trụ `cyl_1`:** Tọa độ $(x = 7.51,\ y = 3.64)$, bán kính $R = 0.35\text{ m}$, chiều cao $H = 2.89\text{ m}$.
- **Target (H-Pad ArUco):** Đặt tại $(x = 10.00,\ y = 3.64,\ z = 0.02)$ (ngay sau lưng trụ `cyl_1`).
- **Điểm Goal bám 3D (3.5m stand-off):** Tự động tính ra tại $(x \approx 7.53,\ y = 3.64,\ z = 2.50)$, nằm ngay sát trụ `cyl_1`. Đường bay thẳng từ drone tới goal sẽ đâm trực diện vào tâm trụ!
- **Drone xuất phát:** Đặt tại $(x = 6.00,\ y = 3.64,\ z = 2.50)$ (cách tâm trụ $1.51\text{ m}$, cách mặt ngoài trụ $1.16\text{ m}$, nằm sâu trong bán kính ảnh hưởng đẩy lùi $d_{\text{inf}} = 2.5\text{ m}$).

#### 2. Các bước thực hiện chi tiết

**Bước 1 — Khởi động APF Planner với map Gazebo (Terminal 1):**
```bash
python3 simulation_maps/apf_planner.py \
  --ros-args \
  --params-file simulation_maps/config/mission_params.yaml \
  -p world_sdf:=simulation_maps/gazebo_worlds/obstacle_avoidance.sdf
```
*Kỳ vọng log terminal:*
```text
[INFO] [apf_planner]: Loaded 10 cylinder obstacles from simulation_maps/gazebo_worlds/obstacle_avoidance.sdf
[INFO] [apf_planner]: APF Planner node ready.
```

**Bước 2 — Mở RViz2 để quan sát không gian 3D (Terminal 2):**
```bash
rviz2
```
*Cấu hình trên giao diện RViz2 (chỉ mất 15 giây):*
1. Tại panel trái mục **Global Options** -> **Fixed Frame**: nhập `world`.
2. Bấm nút **Add** ở góc dưới bên trái.
3. Chọn tab **By topic** -> tìm đến topic `/apf/force_markers` -> chọn **MarkerArray** -> bấm **OK**.
4. *(Tùy chọn)*: Để hiển thị lưới Grid rõ hơn, trong Displays chọn **Grid** -> đặt **Cell Size** = `1.0`, **Plane Cell Count** = `30`.

**Bước 3 — Bật Phase FOLLOW & EKF Tracking (Terminal 3):**
```bash
ros2 topic pub -r 10 /mission/phase std_msgs/msg/String "{data: FOLLOW}" &
ros2 topic pub -r 10 /ekf/tracking_mode std_msgs/msg/String "{data: TRACKING}"
```

**Bước 4 — Đặt Target phía sau vật cản (Terminal 4):**
```bash
ros2 topic pub -r 10 /ekf/target_state nav_msgs/msg/Odometry \
  "{pose: {pose: {position: {x: 10.0, y: 3.64, z: 0.02}}}}"
```

**Bước 5 — Đặt Drone ở cự ly đối đầu với trụ `cyl_1` (Terminal 5):**
```bash
ros2 topic pub -r 10 /odom nav_msgs/msg/Odometry \
  "{pose: {pose: {position: {x: 6.0, y: 3.64, z: 2.5}}}}"
```

**Bước 6 — Đọc dữ liệu vận tốc phản hồi (Terminal 6):**
```bash
ros2 topic echo /apf/velocity_cmd
```

#### 3. Tiêu chí đánh giá & Kết quả quan sát (PASS/FAIL)

**A. Quan sát trực quan trên RViz2:**
1. **10 Cột trụ chướng ngại vật (`apf_obstacles`):** Hiển thị dạng hình trụ màu cam trong suốt (translucent orange) đúng vị trí và kích thước thực tế trong Gazebo world.
2. **Điểm Goal bám (`apf_goal`):** Quả cầu màu vàng (Gold Sphere) xuất hiện tại $(7.53, 3.64, 2.50)$ ngay cạnh trụ.
3. **Drone (`apf_drone`):** Quả cầu màu xanh ngọc (Cyan Sphere) tại $(6.00, 3.64, 2.50)$.
4. **Hệ 3 Vector Lực (`Marker.ARROW`):**
   - **Mũi tên Xanh lá (`apf_att`):** Hướng thẳng tới trước dọc theo trục $+X$ (muốn lao về phía Goal).
   - **Mũi tên Đỏ (`apf_rep`):** Phóng ngược lại theo $-X$ và đẩy lệch sang $-Y$ (lực đẩy lùi cực mạnh từ bề mặt `cyl_1`).
   - **Mũi tên Xanh dương (`apf_total`):** Vector lệnh vận tốc tổng hợp bị bẻ lái lệch hẳn sang sườn trụ (`vx < 0, vy < 0`) để dạt ngang né trụ thay vì lao vào vật cản!

**B. Đọc số liệu trên Terminal 6 (`/apf/velocity_cmd`):**
```yaml
linear:
  x: -1.56106845   # Bị lực đẩy ép lùi lại, ngăn chặn va chạm
  y: -1.25022609   # Lực đẩy tiếp tuyến bẻ lái dạt sang sườn trái
  z: 0.0           # Độ cao z được giữ nguyên tuyệt đối trong phase FOLLOW
```
- Độ lớn vận tốc: $\|v\| = \sqrt{(-1.56)^2 + (-1.25)^2} = 2.0\text{ m/s} = v_{\max}$ (được chuẩn hóa bảo toàn tốc độ tối đa).

#### 4. Thử nghiệm tương tác động (Interactive Verification)
Tại Terminal 5, thử thay đổi tọa độ `x` của drone để kiểm tra đáp ứng của APF theo khoảng cách:
- **Khi lùi Drone ra xa an toàn ($x = 4.0\text{ m}$):**
  Khoảng cách tới mặt trụ là $7.51 - 0.35 - 4.0 = 3.16\text{ m} > d_{\text{inf}}$ ($2.5\text{ m}$).
  $\rightarrow$ Mũi tên Đỏ biến mất hoàn toàn (`F_rep = 0`). Mũi tên Xanh dương chỉ thẳng về Goal ($v_x = 2.0\text{ m/s}, v_y = 0$).
- **Khi Drone áp sát nguy hiểm ($x = 6.8\text{ m}$):**
  Khoảng cách tới mặt trụ chỉ còn $0.36\text{ m}$.
  $\rightarrow$ Mũi tên Đỏ nở cực đại, ép mũi tên Xanh dương đảo chiều quay đầu tháo chạy khẩn cấp.

---

## 7. Test nhóm C — Target tracking và gimbal, không APF/PX4

### C1. ArUco detector độc lập

Chỉ chạy camera bridge và `aruco_sim_node.py`:

```bash
python3 simulation_maps/aruco_sim_node.py
```

Kiểm tra:

```bash
ros2 topic hz /camera
ros2 topic hz /hpad/bbox
ros2 topic echo /hpad/position_camera
ros2 topic echo /hpad/detected
```

PASS khi target tĩnh cho `camera_optical xyz` dao động nhỏ, không có dấu nhảy bất thường khi camera không di chuyển.

### C2. EKF và TF không có drone controller

Chạy thêm:

```bash
python3 simulation_maps/ekf_ros_adapter.py
```

Giữ drone tĩnh, target tĩnh. Sau đó chỉ quay yaw drone hoặc đổi pitch camera từng bước nhỏ.

Ghi lại đồng thời:

```bash
ros2 topic echo /odom
ros2 topic echo /hpad/position_camera
ros2 topic echo /ekf/target_state
ros2 topic echo /ekf/tracking_mode
```

PASS khi target world-frame không quay quanh drone chỉ vì target thật đang đứng yên.

Nếu fail, ưu tiên sửa theo thứ tự:

1. truyền timestamp ảnh qua `PointStamped.header.stamp`;
2. lookup TF tại timestamp đó;
3. sửa dấu pitch giữa SDF và TF;
4. kiểm tra offset `camera_link` và `camera_optical_frame`.

### C3. IBVS open-loop bằng target giả

Không chạy APF, FSM hoặc Offboard. Chạy IBVS:

```bash
python3 simulation_maps/ibvs_controller.py
```

Publish:

```bash
ros2 topic pub -r 10 /mission/phase std_msgs/msg/String "{data: FOLLOW}"
ros2 topic pub -r 10 /ekf/tracking_mode std_msgs/msg/String "{data: TRACKING}"
ros2 topic pub -r 10 /odom nav_msgs/msg/Odometry \
  "{pose: {pose: {position: {x: 0.0, y: 0.0, z: 3.0}}}}"
```

Lần lượt publish target world ở `[5,0,0]`, `[0,5,0]`, `[-5,0,0]`, `[0,-5,0]`. Quan sát:

```bash
ros2 topic echo /ibvs/yaw_cmd
ros2 topic echo /ibvs/gimbal_pitch
```

Kỳ vọng yaw world gần `0`, `+pi/2`, `pi`, `-pi/2`. Khi target chuyển từ `+179°` sang `-179°`, yaw command chỉ được đi qua sai số ngắn khoảng `2°`, không nhảy gần `358°`.

Đây là test logic yaw, chưa chứng minh PX4 quay đúng chiều.

---

## 8. Test nhóm D — Servo thực tế nhưng không APF

Sau khi C3 PASS, thêm một relay rất đơn giản:

```text
/ibvs/yaw_cmd → /mission/yaw_setpoint
```

và publish velocity bằng 0. Chỉ chạy `offboard_commander.py`, không chạy FSM. Mục tiêu là kiểm tra riêng:

```text
IBVS yaw world ENU → ENU/NED conversion → PX4 yaw → drone yaw
```

Test cardinal:

| Target bearing world | Kỳ vọng drone yaw |
|---:|---:|
| `0°` | quay về `+X` world |
| `+90°` | quay về `+Y` world |
| `180°` | quay về `-X` world |
| `-90°` | quay về `-Y` world |

Nếu `/ibvs/yaw_cmd` đúng nhưng drone quay ngược hoặc dừng ở hướng lệch 90°, lỗi nằm ở `_enu_yaw_to_ned()` hoặc giả định frame PX4.

Trong test này phải kiểm tra cả:

```bash
ros2 topic echo /mission/yaw_setpoint
ros2 topic echo /odom
```

Không cho phép APF velocity tham gia, để loại bỏ chuyển động ngang khỏi kết luận yaw.

---

## 9. Test nhóm E — Bay thực tế kiểm thử độc lập thuật toán APF (Không dùng Vision/IBVS)

**Mục tiêu bài test:**
Kiểm thử vòng lặp kín (Closed-Loop Flight) giữa **Thuật toán APF 3D** và **Bộ điều khiển bay PX4 trong Gazebo** mà **HOÀN TOÀN KHÔNG CẦN Vision (ArUco detector)**, **KHÔNG CẦN Gimbal Servo (IBVS)**, và không phụ thuộc vào điều kiện ánh sáng camera. 

Vị trí Target được cấp cố định từ trước vào map. Drone sẽ cất cánh từ vị trí ban đầu cho trước, đọc Odometry thật từ Gazebo, dùng APF tính vector vận tốc bay tới mục tiêu (đồng thời né chướng ngại vật từ file SDF) và điều khiển drone thực sự bay trong Gazebo + RViz2 để đánh giá độ ổn định.

---

### 1. Thông số không gian định trước (Known Coordinates Setup)

- **Vị trí Drone ban đầu:**
  - Trên mặt đất Gazebo: $(x = 0.0,\ y = 0.0,\ z = 0.0)$.
  - Sau khi cất cánh Takeoff: $(x = 0.0,\ y = 0.0,\ z = 3.0\text{ m})$ (hover ổn định).
- **Vị trí Target cố định (Pre-fed Target):**
  - Tọa độ: $(x = 5.00,\ y = 2.00,\ z = 0.02\text{ m})$ (hoặc tọa độ tùy chọn bạn muốn cấp).
- **Vật cản tĩnh (Known Obstacles từ SDF):** 10 trụ cố định từ file `obstacle_avoidance.sdf`.
- **Khoảng cách bám mong muốn ($d_{\text{follow}}$):** $3.5\text{ m}$ (khoảng cách 3D từ drone tới Target).
- **Điểm Goal bám 3D (Stand-off Goal):**
  $$R_{xy} = \sqrt{3.5^2 - (3.0 - 0.02)^2} \approx 1.836\text{ m} \implies (x_{\text{goal}} \approx 3.30,\ y_{\text{goal}} \approx 1.32,\ z_{\text{goal}} = 3.00\text{ m})$$

---

### 2. Trình tự khởi động các Terminal

#### Terminal 1: Khởi động PX4 SITL & Gazebo Harmonic
```bash
cd /home/duy/VDT_project/PX4-Autopilot
export GZ_PARTITION=vdt_harmonic
export GZ_SIM_RESOURCE_PATH="$PWD/Tools/simulation/gz/models:${GZ_SIM_RESOURCE_PATH:-}"
PX4_GZ_WORLD=obstacle_avoidance make px4_sitl gz_x500_depth
```
*(Chờ Gazebo mở: Drone xuất hiện tại `(0, 0, 0)`, bãi đáp tại `(5.0, 2.0)` và 10 cột trụ cản xung quanh)*.

#### Terminal 2: Cầu nối Odometry & Mở RViz2 (Không bật autonomous pipeline)
```bash
cd /home/duy/VDT_project
source /opt/ros/humble/setup.bash
export GZ_PARTITION=vdt_harmonic
python3 simulation_maps/launch_simulation.py
```
*(Lệnh này chỉ chạy Bridge `/odom`, broadcast TF và mở RViz2 hiển thị 3D; KHÔNG khởi chạy Vision hay IBVS)*.

* **Giám sát trên QGroundControl (Terminal 2)**:
```bash
~/QGroundControl.AppImage
```

#### Terminal 3: Khởi động APF Planner với map Gazebo
```bash
cd /home/duy/VDT_project
source /opt/ros/humble/setup.bash
python3 simulation_maps/apf_planner.py \
  --ros-args \
  --params-file simulation_maps/config/mission_params.yaml \
  -p world_sdf:=simulation_maps/gazebo_worlds/obstacle_avoidance.sdf
```
*(Node nạp 10 trụ cản, tính toán vận tốc và góc yaw điều khiển bay)*.

#### Terminal 4: Khởi động Offboard Commander (Nối trực tiếp với APF)
```bash
cd /home/duy/VDT_project
source /opt/ros/humble/setup.bash
python3 simulation_maps/offboard_commander.py \
  --ros-args \
  -r /mission/velocity_setpoint:=/apf/velocity_cmd \
  -r /mission/yaw_setpoint:=/apf/yaw_cmd
```
*(Offboard Commander nhận trực tiếp lệnh vận tốc và góc yaw từ APF, kết nối MAVLink UDP tới PX4)*.

#### Terminal 5: Cấp vị trí Target cố định vào map
```bash
cd /home/duy/VDT_project
source /opt/ros/humble/setup.bash
python3 simulation_maps/mock_target_publisher.py --x 5.0 --y 2.0 --z 0.02
```
*(Node cấp vị trí Target `(5.0, 2.0, 0.02)`, đồng thời kích hoạt phase `FOLLOW` & mode `TRACKING`)*.

#### Terminal 6: Ra lệnh Cất cánh Drone lên độ cao 3.0 m
```bash
python3 /home/duy/VDT_project/simulation_maps/takeoff.py --alt 3.0
```

---

### 3. Diễn biến quan sát & Tiêu chuẩn đánh giá ổn định (PASS Criteria)

1. **Cất cánh mượt mà:** Drone arm động cơ, bay thẳng đứng từ mặt đất lên $z = 3.0\text{ m} \pm 0.15\text{ m}$ và hover thăng bằng.
2. **Kích hoạt bay theo APF:**
   - Ngay khi ở trên không, `offboard_commander` tự động nhận diện phase `FOLLOW` và chiếm quyền `OFFBOARD` trên PX4.
   - Vector APF (mũi tên xanh dương trên RViz2) chỉ thẳng về phía điểm bám $(3.30, 1.32, 3.0)$.
   - **Drone trong Gazebo thực sự nghiêng thân và bay lướt đi trong không gian**, tịnh tiến thẳng về điểm bám với tốc độ $\le 2.0\text{ m/s}$.
3. **Dừng chính xác & Hover ổn định:**
   - Khi áp sát cự ly $3.5\text{ m}$ so với Target (tọa độ $(3.30, 1.32)$), vector vận tốc APF tự động triệt tiêu dần về 0.
   - Drone giảm tốc êm ái, dừng lại và **hover thăng bằng cực kỳ ổn định tại $(3.30, 1.32, 3.0\text{ m})$**, mũi drone xoay yaw hướng thẳng về phía Target!
   - Không bị hiện tượng dao động lố (overshoot) hay rung giật qua lại quanh đích.

---

### 4. Thử nghiệm đổi vị trí Target hoặc kiểm tra né vật cản
Trong khi drone đang hover ổn định, tại **Terminal 5**, bạn có thể bấm `Ctrl+C` và đổi tọa độ Target sang một vị trí khác (ví dụ ra sau lưng một cột trụ cụ thể để xem drone lượn vòng né trụ):
```bash
# Đổi Target ra tọa độ mới (10.0, 3.64) nằm ngay sau trụ cyl_1:
python3 simulation_maps/mock_target_publisher.py --x 10.0 --y 3.64 --z 0.02
```
👉 Drone trong Gazebo sẽ lập tức nhận lệnh mới từ APF, cất cánh bay tiếp, gặp trụ `cyl_1` chắn đường sẽ tự động lượn vòng né sang sườn và bay tới vị trí mới một cách trơn tru!

---

## 10. Bảng chẩn đoán nhanh

| Quan sát | Nguyên nhân ưu tiên |
|---|---|
| `/hpad/position_camera` đúng nhưng `/ekf/target_state` quay khi gimbal đổi pitch | TF pitch sign hoặc timestamp |
| Target world-frame nhảy mỗi khi drone quay | lookup TF latest thay vì image timestamp |
| `/ekf/target_state` ổn định, `/ibvs/yaw_cmd` nhảy ±π | angle wrap, bearing hoặc pixel sign |
| `/ibvs/yaw_cmd` đúng nhưng drone quay sai chiều | ENU/NED yaw conversion hoặc PX4 frame |
| `/apf/velocity_cmd` đúng nhưng drone bay sai hướng | ENU/NED velocity conversion |
| APF vector hướng tới `[1.75, 0.70]` thay vì `[5, 2]` | hành vi stand-off 3.5 m đang hoạt động đúng thiết kế |
| `linear.z` khác 0 khi không có obstacle và target z thấp | lỗi follow goal/altitude hold |
| target mất sau khi bắt đầu quay | camera TF, timestamp, gimbal pitch hoặc yaw quá nhanh |

---

## 11. Thứ tự thực hiện đề xuất

1. **Test 0.2** — xác nhận dấu pitch Gazebo ↔ TF.
2. **Test 0.3/C2** — target world-frame phải đứng yên khi target thật đứng yên.
3. **Test A** — APF core offline.
4. **Test B** — APF ROS node với topic giả và quan sát vector trực quan trên RViz2.
5. **Test C3** — IBVS yaw open-loop.
6. **Test D** — yaw servo/PX4 với velocity bằng 0.
7. **Test E** — Chạy `--autonomous` với drone thật cất cánh, bay bám H-Pad và né vật cản trong Gazebo + RViz2.
