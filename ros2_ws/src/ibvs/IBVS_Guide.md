# IBVS Controller — Guide

## 1. Package cần cài đặt ngoài

Không cần thư viện Python hay binary ngoài. Node chỉ dùng các package ROS2 sau, thường đã có sẵn trong bản cài ROS2 desktop:

```bash
sudo apt install ros-$ROS_DISTRO-vision-msgs
```

Package ROS2 cần có sẵn: `rclpy`, `std_msgs`, `nav_msgs`, `sensor_msgs`, `vision_msgs`.

Điều kiện đầu vào:
- `aruco_node` đang chạy, publish `/hpad/detected` (mọi frame) và `/hpad/bbox` (chỉ khi thấy marker).
- `px4_state_bridge` publish `/odom` (frame `world`, ENU).
- `vision_interface_bridge` publish `/mission/phase`.
- `ekf_node` publish `/ekf/target_state` và `/ekf/tracking_mode`. Thiếu hai topic này node vẫn chạy ở chế độ chỉ dùng pixel.
- `gimbal_control` publish `/gimbal/target_angle_deg` (chưa nhận thì giả định -30°).
- `/camera_info` phải cùng camera với ảnh phát hiện marker (IR1).

## 2. Nguyên lý hoạt động

```
/hpad/bbox, /hpad/detected   (aruco_node)
/camera_info                 (camera driver)
/odom                        (px4_state_bridge)
/ekf/target_state, /ekf/tracking_mode   (ekf_node)
/mission/phase               (vision_interface_bridge)
/gimbal/target_angle_deg     (gimbal_control)
        |
        v
   ibvs_controller
     - bbox_cb: tính theo sự kiện khi có bbox mới
     - timer 30 Hz: Trong pha SEARCH, IBVS không tự quét yaw (yaw_cmd = drone_yaw, pitch_trim = 0.0). Nhiệm vụ quay quét tìm kiếm 360° do offboard_manager toàn quyền điều khiển, IDLE reset, FOLLOW/APPROACH tính lại
        |
        +--> /ibvs/pitch_trim_deg (Float32, độ) --> gimbal_control cộng vào góc mục tiêu
        +--> /ibvs/yaw_cmd        (Float64, rad, heading ENU tuyệt đối)
```

Phân vai với `gimbal_control`: `gimbal_control` quyết định góc pitch nền theo phase và hình học 3D, rồi làm mượt và giới hạn tốc độ. IBVS không ra góc pitch tuyệt đối, chỉ gửi lượng hiệu chỉnh nhỏ theo pixel để bù sai số EKF và sai lệch góc servo.

Hành vi theo phase (`/mission/phase`):

| Phase | Hiệu chỉnh pitch | Yaw |
|---|---|---|
| IDLE (mặc định khi chưa nhận phase) | 0 | `yaw_cmd` bằng yaw thực, không điều khiển |
| SEARCH | 0 | Quét với `search_yaw_rate`. Khi thấy marker thì giữ yaw cố định, quá 0.75 s không thấy thì quét tiếp từ yaw thực |
| FOLLOW, APPROACH | Theo pixel, kẹp ±`pitch_trim_limit_deg` | Bám pixel kèm feedforward |
| LAND | 0 | Chỉnh nhẹ theo pixel, chỉ khi có bbox mới |

Mỗi lần tính (FOLLOW/APPROACH):
1. `dt` lấy từ lần tính trước, kẹp trong [0.001, 0.5] s.
2. Mục tiêu EKF coi là dùng được khi đã nhận `/ekf/target_state` và `tracking_mode` thuộc `TRACKING`, `PREDICTING`, `PREDICTING_DEGRADED`.
3. Hiệu chỉnh pixel: `correction = -K_pitch * (ev_eff / f_y) * pitch_pixel_trim_gain`, với `ev_eff` là sai số dọc đã trừ deadband 6 px.
4. Hiệu chỉnh pitch gửi đi (`pitch_trim`):
   - EKF dùng được: mục tiêu là `correction` (về 0 khi mất marker).
   - Chỉ có pixel: cộng dồn `correction` vào `pitch_trim` hiện tại.
   - Không có gì: giữ `pitch_trim`.
   - Sau đó kẹp ±`pitch_trim_limit_deg`, giới hạn tốc độ `pitch_rate_limit * dt`, rồi lọc EMA (`pitch_ema_alpha`).
5. Yaw, khi thấy marker: `desired = yaw_uav - K_yaw * (eu_eff / f_x) * max(0.4, cos(pitch))`, với `pitch` là góc gimbal thực nhận từ `/gimbal/target_angle_deg`. `yaw_cmd` tiến về `desired` theo đường ngắn nhất, tối đa `yaw_rate_limit * dt`.
6. Feedforward yaw: nếu EKF dùng được và khoảng cách ngang lớn hơn 1 m, cộng thêm tốc độ góc tiếp tuyến của mục tiêu (kẹp ±0.15 rad/s, bỏ qua nếu nhỏ hơn 0.01).
7. Yaw, khi mất marker nhưng EKF dùng được và khoảng cách ngang lớn hơn 0.3 m: `yaw_cmd` tiến về hướng tới vị trí dự đoán (vị trí EKF cộng vận tốc nhân 0.3 s). Còn lại giữ nguyên `yaw_cmd`.

Dấu: `eu = u - u0` dương (marker lệch phải) làm yaw giảm, tức quay phải. `ev = v - v0` dương (marker lệch dưới) làm trim âm, tức chúi thêm xuống.

Giao ước với `gimbal_control` (phía gimbal cần làm):
- Đăng ký `/ibvs/pitch_trim_deg` (`Float32`, độ, âm là chúi thêm xuống).
- Chỉ cộng vào góc mục tiêu ở FOLLOW và APPROACH.
- Bỏ qua (coi bằng 0) nếu giá trị không hữu hạn hoặc cũ hơn khoảng 0.3 s, để mất IBVS không làm kẹt góc.
- Kẹp tổng góc trong `[out_min_deg, out_max_deg]` trước khi đưa vào PID và slew limiter.
- Góc cuối cùng vẫn publish ở `gimbal/target_angle_deg`, nên TF camera của `px4_state_bridge` đã gồm cả phần hiệu chỉnh.

Lưu ý:
- `yaw_cmd` là heading ENU tuyệt đối (rad, 0 là hướng Đông, ngược chiều kim đồng hồ là dương), cùng quy ước với `PlannerOutput.yaw`.
- `/ibvs/yaw_cmd` chưa có node tiêu thụ vì node ghép planner chưa có.
- Giới hạn góc theo phase (ví dụ APPROACH không chúi dưới -20° hay LAND cố định -90°) không còn nằm ở IBVS. Phần này do `gimbal_control` quyết định.
- Intrinsics (`fx, fy, u0, v0`) được ghi đè bằng `/camera_info` khi nhận message có `K[0] > 0`. Trước đó dùng giá trị tham số.
- Ở LAND, bước chỉnh yaw nhân thêm `dt` nên độ lợi rất nhỏ so với FOLLOW (ví dụ lệch 100 px chỉ ra khoảng 0.003 rad mỗi lần).
- Ở LAND, timer 30 Hz không tính lại. Mất marker thì không còn lệnh mới, `yaw_cmd` giữ giá trị cuối.

## 3. Cách chạy

```bash
colcon build --packages-select ibvs
source install/setup.bash
ros2 launch ibvs ibvs.launch.py
```

Chạy riêng, kèm tham số tùy chỉnh:

```bash
ros2 run ibvs ibvs_controller --ros-args -p K_yaw:=0.4 -p yaw_rate_limit:=0.3
```

Tham số launch:

| Tên | Mặc định | Ý nghĩa |
|---|---|---|
| `params_file` | `config/ibvs_params.yaml` | File tham số nạp cho node |
| `use_sim_time` | false | Dùng thời gian mô phỏng hoặc rosbag |

Tham số của node (`config/ibvs_params.yaml`):

| Tên | Mặc định | Ý nghĩa |
|---|---|---|
| `K_pitch` | 0.8 | Độ lợi hiệu chỉnh pitch theo pixel |
| `K_yaw` | 0.5 | Độ lợi yaw theo pixel |
| `focal_x`, `focal_y` | 466.0 | Tiêu cự (pixel), bị ghi đè bởi `/camera_info` |
| `u0`, `v0` | 320.0, 240.0 | Tâm ảnh (pixel), bị ghi đè bởi `/camera_info` |
| `search_yaw_rate` | 0.2 | Tốc độ quét yaw ở SEARCH (rad/s) |
| `pitch_rate_limit` | 1.5 | Giới hạn tốc độ đổi hiệu chỉnh pitch (rad/s) |
| `pitch_ema_alpha` | 0.25 | Hệ số lọc EMA của hiệu chỉnh pitch (1 là không lọc) |
| `pitch_pixel_trim_gain` | 0.5 | Hệ số nhân của hiệu chỉnh pixel |
| `pitch_trim_limit_deg` | 20.0 | Biên độ tối đa của hiệu chỉnh pitch (độ) |
| `yaw_rate_limit` | 0.5 | Giới hạn tốc độ đổi `yaw_cmd` (rad/s) |

Các hằng số cố định trong code (không phải tham số):

| Hằng số | Giá trị |
|---|---|
| Deadband pitch / yaw | 6 px / 12 px |
| Kẹp feedforward yaw | ±0.15 rad/s |
| Góc gimbal giả định khi chưa nhận lệnh | -30° |
| Giữ yaw sau khi thấy marker ở SEARCH | 0.75 s |
| Giữ yaw lúc vào SEARCH | 0.30 s |
| Tần số timer | 30 Hz |

Topic vào và ra:

| Topic | Kiểu | Hướng |
|---|---|---|
| `/camera_info` | `CameraInfo` | vào (QoS `BEST_EFFORT`) |
| `/hpad/bbox` | `vision_msgs/BoundingBox2D` | vào |
| `/hpad/detected` | `Bool` | vào |
| `/ekf/tracking_mode` | `String` | vào |
| `/mission/phase` | `String` | vào (QoS `BEST_EFFORT`) |
| `/odom` | `Odometry` | vào (QoS `BEST_EFFORT`) |
| `/ekf/target_state` | `Odometry` | vào |
| `/gimbal/target_angle_deg` | `Float32` | vào (QoS `BEST_EFFORT`) |
| `/ibvs/pitch_trim_deg` | `Float32` | ra |
| `/ibvs/yaw_cmd` | `Float64` | ra |

Tên topic nằm cố định trong code. Đổi tên bằng remap:

```bash
ros2 run ibvs ibvs_controller --ros-args -r /camera_info:=/camera/infra1/camera_info
```

## 4. Cách debug

Node chỉ log lúc khởi động (`IBVS controller ready`) và khi chuyển SEARCH sang FOLLOW (`SEARCH->FOLLOW: reset yaw_cmd to ...`). Quan sát bằng topic:

```bash
ros2 topic echo /mission/phase
ros2 topic echo /ibvs/yaw_cmd
ros2 topic echo /ibvs/pitch_trim_deg
ros2 topic hz /ibvs/yaw_cmd
ros2 topic echo /hpad/bbox --field center.position
ros2 topic echo /ekf/tracking_mode
ros2 topic echo /gimbal/target_angle_deg
```

Nếu `pitch_trim_deg` luôn bằng 0 và `yaw_cmd` bám yaw thực: phase đang là IDLE. Kiểm tra `/mission/phase` có message không. `vision_interface_bridge` phát `IDLE` khi `fsm/state` cũ hơn timeout hoặc FSM ở COMPLETE.

Nếu node không nhận được topic nào đó dù topic đang có dữ liệu: nghi QoS không khớp. Subscriber `RELIABLE` không nhận được từ publisher `BEST_EFFORT`, ngược lại thì nhận được. Kiểm tra bằng `ros2 topic info <topic> -v`.

QoS hiện tại của node:
- `/odom`, `/camera_info`, `/mission/phase`, `/gimbal/target_angle_deg`: subscriber `BEST_EFFORT` (depth 5), nhận được từ cả hai loại publisher.
- `/hpad/bbox`, `/hpad/detected` (`aruco_node`) và `/ekf/target_state`, `/ekf/tracking_mode` (`ekf_node`): subscriber `RELIABLE` (depth 10). Hai node kia publish bằng QoS mặc định `RELIABLE` nên khớp. Nếu sau này đổi publisher sang `BEST_EFFORT` thì phải đổi subscriber tương ứng.
- `/ibvs/yaw_cmd`, `/ibvs/pitch_trim_deg`: publisher `RELIABLE`, subscriber nào cũng nhận được.

Nếu `yaw_cmd` không đổi hoặc nhảy lúc mới chạy: chưa nhận `/odom`. Khi đó yaw thực coi như 0 và `yaw_cmd` khởi tạo theo giá trị đầu tiên của odom.

Nếu marker không về giữa ảnh hoặc yaw quay ngược chiều: kiểm tra `/camera_info` có đúng camera của ảnh phát hiện không (K sai làm sai độ lợi), và `/odom` có đúng ENU không. Yaw của node lấy từ quaternion của `/odom`.

Nếu yaw dao động quanh marker: giảm `K_yaw` hoặc `yaw_rate_limit`. Nếu pitch dao động: giảm `K_pitch`, `pitch_pixel_trim_gain` hoặc `pitch_ema_alpha`. Deadband là hằng số cố định trong `ibvs_logic.py`.

Nếu `pitch_trim_deg` khác 0 nhưng gimbal không đổi: `gimbal_control` chưa đăng ký topic này (giao ước ở phần 2). So sánh `/gimbal/target_angle_deg` khi có và không có hiệu chỉnh.

Nếu hiệu chỉnh pitch chạm biên ±`pitch_trim_limit_deg` thường xuyên: góc nền của `gimbal_control` đang lệch lớn so với marker thật. Kiểm tra EKF, vị trí lắp camera (`gimbal_pivot_xyz`, `camera_in_gimbal_xyz`) và góc thực của servo so với góc lệnh, thay vì tăng biên.

Nếu pitch chỉ cộng dồn từ pixel mà không theo hình học 3D: `/ekf/tracking_mode` đang là `EXPIRED` hoặc chưa nhận `/ekf/target_state`, nên node rơi về chế độ chỉ dùng pixel.

Nếu SEARCH không dừng quét khi thấy marker: kiểm tra `/hpad/detected` có `true` không. Giữ yaw chỉ kích hoạt từ cờ này, không từ bbox.

Nếu ở LAND mất marker mà lệnh yaw đứng yên: đúng với thiết kế hiện tại (xem phần Lưu ý), node chỉ tính lại khi có bbox mới.

## Unit test

Test chạy trên các hàm tính toán thuần trong `ibvs_logic.py`, không cần ROS2 hay phần cứng.

```bash
python3 -m pip install pytest
cd ros2_ws/src/ibvs
python3 -m pytest test/test_ibvs_logic.py
```

Hoặc dùng `colcon test --packages-select ibvs`.

Nội dung được kiểm tra: đổi quaternion sang yaw (round-trip), chuẩn hóa góc, `slew_angle` (giới hạn bước, đường ngắn nhất qua ±π), `clamp`, deadband (đối xứng, biên), hiệu chỉnh pitch theo pixel (dấu, deadband, tỉ lệ với hệ số), feedforward yaw tiếp tuyến (dấu, kẹp, khoảng cách 0, bất biến theo phép xoay).

Node `ibvs_controller.py` (máy trạng thái theo phase, publish) không nằm trong unit test. Kiểm tra bằng rosbag hoặc mô phỏng và quan sát `/ibvs/yaw_cmd`, `/ibvs/pitch_trim_deg`.