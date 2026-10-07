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
     - timer 30 Hz: tính lại theo phase
         IDLE      : reset, yaw_cmd = yaw thực, pitch_trim = 0
         SEARCH    : yaw_cmd = yaw thực, pitch_trim = 0 (IBVS không quét,
                     việc quay quét 360° do offboard_manager điều khiển)
         FOLLOW,
         APPROACH  : tính lại theo pixel và EKF
        |
        +--> /ibvs/pitch_trim_deg (Float32, độ) --> gimbal_control cộng vào góc mục tiêu
        +--> /ibvs/yaw_cmd        (Float64, rad, heading ENU tuyệt đối) --> planner_merge_node
```

Phân vai với `gimbal_control`: `gimbal_control` quyết định góc pitch nền theo phase và hình học 3D, rồi làm mượt và giới hạn tốc độ. IBVS không ra góc pitch tuyệt đối, chỉ gửi lượng hiệu chỉnh nhỏ theo pixel để bù sai số EKF và sai lệch góc servo.

Hành vi theo phase (`/mission/phase`):

| Phase | Hiệu chỉnh pitch | Yaw |
|---|---|---|
| IDLE (mặc định khi chưa nhận phase) | 0 | `yaw_cmd` bằng yaw thực, không điều khiển |
| SEARCH | 0 | `yaw_cmd` bằng yaw thực, IBVS không quét (việc quét do `offboard_manager` điều khiển) |
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
- `/ibvs/yaw_cmd` được `planner_merge_node` (package `apf_planner`) tiêu thụ và ghép vào `planner/velocity_setpoint` theo `yaw_source` (mặc định `ibvs_apf`: ưu tiên IBVS, dự phòng yaw của APF).
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
| Tần số timer | 30 Hz |

Topic vào và ra:

| Topic | Kiểu | Hướng | Nguồn / Đích |
|---|---|---|---|
| `/camera_info` | `CameraInfo` | vào (QoS `BEST_EFFORT`) | camera driver |
| `/hpad/bbox` | `vision_msgs/BoundingBox2D` | vào | `aruco_node` |
| `/hpad/detected` | `Bool` | vào | `aruco_node` |
| `/ekf/tracking_mode` | `String` | vào | `ekf_node` |
| `/mission/phase` | `String` | vào (QoS `BEST_EFFORT`) | `vision_interface_bridge` |
| `/odom` | `Odometry` | vào (QoS `BEST_EFFORT`) | `px4_state_bridge` |
| `/ekf/target_state` | `Odometry` | vào | `ekf_node` |
| `/gimbal/target_angle_deg` | `Float32` | vào (QoS `BEST_EFFORT`) | `gimbal_control` |
| `/ibvs/pitch_trim_deg` | `Float32` | ra | `gimbal_control` |
| `/ibvs/yaw_cmd` | `Float64` | ra | `planner_merge_node` |

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

Nếu `pitch_trim_deg` luôn bằng 0 và `yaw_cmd` bám yaw thực: phase đang là IDLE (hoặc SEARCH, IBVS không điều khiển ở hai phase này). Kiểm tra `/mission/phase` có message không. `vision_interface_bridge` phát `IDLE` khi `fsm/state` cũ hơn timeout hoặc FSM ở COMPLETE.

Nếu node không nhận được topic nào đó dù topic đang có dữ liệu: nghi QoS không khớp. Subscriber `RELIABLE` không nhận được từ publisher `BEST_EFFORT`, ngược lại thì nhận được. Kiểm tra bằng `ros2 topic info <topic> -v`.

QoS hiện tại của node:
- `/odom`, `/camera_info`, `/mission/phase`, `/gimbal/target_angle_deg`: subscriber `BEST_EFFORT` (depth 5), nhận được từ cả hai loại publisher.
- `/hpad/bbox`, `/hpad/detected` (`aruco_node`) và `/ekf/target_state`, `/ekf/tracking_mode` (`ekf_node`): subscriber `RELIABLE` (depth 10). Hai node kia publish bằng QoS mặc định `RELIABLE` nên khớp. Nếu sau này đổi publisher sang `BEST_EFFORT` thì phải đổi subscriber tương ứng.
- `/ibvs/yaw_cmd`, `/ibvs/pitch_trim_deg`: publisher `RELIABLE`, subscriber nào cũng nhận được.

Nếu `yaw_cmd` không đổi hoặc nhảy lúc mới chạy: chưa nhận `/odom`. Khi đó yaw thực coi như 0 và `yaw_cmd` khởi tạo theo giá trị đầu tiên của odom.

Nếu marker không về giữa ảnh hoặc yaw quay ngược chiều: kiểm tra `/camera_info` có đúng camera của ảnh phát hiện không (K sai làm sai độ lợi), và `/odom` có đúng ENU không. Yaw của node lấy từ quaternion của `/odom`.

Nếu yaw dao động quanh marker: giảm `K_yaw` hoặc `yaw_rate_limit`. Nếu pitch dao động: giảm `K_pitch`, `pitch_pixel_trim_gain` hoặc `pitch_ema_alpha`. Deadband là hằng số cố định trong `ibvs_logic.py`.

Nếu `pitch_trim_deg` khác 0 nhưng gimbal không đổi: `gimbal_control` chưa đăng ký topic này (giao ước ở phần 2). So sánh `/gimbal/target_angle_deg` khi có và không có hiệu chỉnh.

Nếu `/ibvs/yaw_cmd` có giá trị nhưng UAV không xoay theo: kiểm tra `planner_merge_node` có nhận `/ibvs/yaw_cmd` trong `yaw_timeout_sec` và `yaw_source` có phải `hold` không.

Nếu hiệu chỉnh pitch chạm biên ±`pitch_trim_limit_deg` thường xuyên: góc nền của `gimbal_control` đang lệch lớn so với marker thật. Kiểm tra EKF, vị trí lắp camera (`gimbal_pivot_xyz`, `camera_in_gimbal_xyz`) và góc thực của servo so với góc lệnh, thay vì tăng biên.

Nếu pitch chỉ cộng dồn từ pixel mà không theo hình học 3D: `/ekf/tracking_mode` đang là `EXPIRED` hoặc chưa nhận `/ekf/target_state`, nên node rơi về chế độ chỉ dùng pixel.

Nếu pixel không có tác dụng dù có bbox: kiểm tra `/hpad/detected` có `true` không. Pixel chỉ được dùng khi cờ này bằng `true`, bbox đến mà `detected` bằng `false` thì bị bỏ qua.

Nếu ở LAND mất marker mà lệnh yaw đứng yên: đúng với thiết kế hiện tại (xem phần Lưu ý), node chỉ tính lại khi có bbox mới.

## 5. Test

Test chia 3 tầng, nằm trong `test/` của package. Tầng 1 không cần ROS 2 chạy; tầng 2 và 3 cần source ROS 2 và workspace đã build.

Cài đặt:

```bash
python3 -m pip install pytest
sudo apt install ros-$ROS_DISTRO-ros2bag ros-$ROS_DISTRO-rosbag2-storage-default-plugins
```

`test/` không có `__init__.py`, `test/conftest.py` thêm thư mục gốc của package vào `sys.path` để `from ibvs_logic import ...` và `from ibvs_controller import ...` hoạt động (code import kiểu module phẳng). `setup.py` cần khai báo `py_modules=['ibvs_controller', 'ibvs_logic']` và entry point `ibvs_controller = ibvs_controller:main`, vì test rosbag chạy `ros2 run ibvs ibvs_controller`.

### Tầng 1: logic thuần

Không tạo node, chạy trong vài giây.

| File | Kiểm tra |
|---|---|
| `test_ibvs_logic.py` | `quaternion_to_yaw` (round-trip, bỏ qua roll/pitch, quaternion đơn vị), `wrap_angle` (giá trị, nằm trong [-π, π], tương đương theo chu kỳ), `slew_angle` (giới hạn bước, tới đích khi trong bước, bước 0, đường ngắn nhất qua ±π, kết quả đã chuẩn hóa), `clamp`, `deadband` (trong vùng chết, tại biên, đối xứng, độ rộng 0), `pixel_pitch_correction` (dấu, deadband, giá trị, hàm lẻ, tỉ lệ với `K_pitch`/gain/`focal_y`), `tangential_yaw_feedforward` (dấu, vận tốc hướng tâm bằng 0, kẹp ±0.15, khoảng cách 0, giảm theo khoảng cách, bất biến theo phép xoay) |

```bash
cd ros2_ws/src/ibvs
python3 -m pytest test/test_ibvs_logic.py -v
```

### Tầng 2: node ROS 2 với publisher giả

Tạo `IBVSController` thật. Phần lớn test gọi trực tiếp callback (`odom_cb`, `phase_cb`, `bbox_cb`, `fallback_timer_cb`...) và đặt `last_update_time` để cố định `dt`, nên kết quả xác định. Một nhóm test chạy executor cùng node helper trong cùng process, publish vào topic đầu vào rồi kiểm tra topic đầu ra. `rclpy` khởi tạo một lần cho cả file, mỗi test tạo node mới.

| File | Kiểm tra |
|---|---|
| `test_ibvs_node.py` | Trạng thái và tham số mặc định, `/camera_info` ghi đè intrinsics (bỏ qua khi `K[0] = 0`), góc gimbal (bỏ qua NaN), odom đầu tiên khởi tạo `yaw_cmd` một lần, đổi phase reset `pitch_trim`, SEARCH sang FOLLOW reset `yaw_cmd`, bbox ở IDLE không ra lệnh, yaw FOLLOW (quay phải/trái, deadband, tỉ lệ dưới giới hạn tốc độ, `dt` bị kẹp 0.5 s, hệ số cos theo góc gimbal), bỏ qua pixel khi `detected` là false, pitch trim (bước đầu, cộng dồn, kẹp biên, dấu), EKF dùng được (kéo về `correction`) và EKF hết hạn (rơi về cộng dồn pixel), feedforward yaw (bật, tắt khi gần dưới 1 m, tắt khi EKF không dùng được), mất marker (quay về hướng mục tiêu EKF, dùng vị trí dự đoán, giữ yaw khi EKF không dùng được hoặc mục tiêu gần dưới 0.3 m, giữ pitch trim), APPROACH giống FOLLOW, LAND (trim bằng 0, bước yaw nhỏ, không có pixel thì yaw bám yaw thực), timer ở SEARCH và IDLE bám yaw thực và trim bằng 0; qua topic: publish đều 30 Hz ở IDLE, `yaw_cmd` bám odom, `/camera_info` và gimbal cập nhật node, topic EKF cập nhật node, chuỗi FOLLOW ra lệnh yaw và trim đúng chiều |

```bash
python3 -m pytest test/test_ibvs_node.py -v
```

### Tầng 3: rosbag

`test_ibvs_rosbag.py` tự sinh một bag tổng hợp bằng `rosbag2_py`, chạy `ros2 run ibvs ibvs_controller`, phát lại bag bằng `ros2 bag play` rồi kiểm tra `/ibvs/yaw_cmd` và `/ibvs/pitch_trim_deg`. Node dùng `time.monotonic()` nên không cần `--clock`. UAV đứng yên với yaw 0 (odom 50 Hz), phase `FOLLOW`, gimbal -30°, camera_info fx = fy = 500 và tâm (320, 240) ở 10 Hz, không có EKF (chế độ chỉ dùng pixel). `/hpad/detected` và `/hpad/bbox` 15 Hz.

| Đoạn | Thời gian | Đầu vào | Kỳ vọng |
|---|---|---|---|
| 1 | 0 - 3 s | Marker lệch phải 150 px, lệch dưới 80 px, `detected` true | `yaw_cmd` âm và hội tụ về `-K_yaw * (138 / 500) * cos(-30°)` khoảng -0.1195 rad (sai số 0.01), `pitch_trim` âm và chạm gần biên -20° |
| 2 | 3 - 6 s | Mất marker, `detected` false, không có bbox | `yaw_cmd` và `pitch_trim` giữ nguyên trong 2 s cuối (dao động dưới 1e-3) |

Tổng số message mỗi topic phải trên 50, mọi giá trị hữu hạn, `yaw_cmd` nằm trong [-π, π], `pitch_trim` không vượt -20° và giá trị cuối không lớn hơn -15°. Test chạy khoảng 10 s theo thời gian thật, cộng 3 s chờ node khởi động. Test tự bỏ qua nếu thiếu `rosbag2_py` hoặc lệnh `ros2`.

```bash
python3 -m pytest test/test_ibvs_rosbag.py -v
```

Để thay bằng bag thật, ghi bag rồi đổi đường dẫn bag trong fixture `replay` và chỉnh lại các giá trị kỳ vọng theo kịch bản đã ghi:

```bash
ros2 bag record -o sample_ibvs /odom /camera_info /mission/phase /gimbal/target_angle_deg /hpad/detected /hpad/bbox /ekf/target_state /ekf/tracking_mode
```

### Chạy toàn bộ

```bash
colcon build --packages-select ibvs
source install/setup.bash
colcon test --packages-select ibvs --event-handlers console_direct+
colcon test-result --verbose
```

### Lưu ý khi chạy test

- Test tầng 2 (nhóm qua topic) và tầng 3 dùng thời gian thật nên có thể chập chờn trên máy chậm; chạy lại một lần trước khi kết luận lỗi.
- Không chạy song song nhiều test ROS trong cùng `ROS_DOMAIN_ID`, vì các node dùng chung tên topic. Nếu cần chạy cạnh hệ thống đang chạy, đặt domain riêng: `ROS_DOMAIN_ID=77 python3 -m pytest ...`.
- Test rosbag chỉ chạy `ibvs_controller`. `aruco_node`, `ekf_node` và `gimbal_control` không chạy, dữ liệu của chúng đến từ bag. Nhánh dùng EKF chỉ được kiểm tra ở tầng 2.
- `BoundingBox2D` được dùng theo `center.position.x/y`, khớp với `vision_msgs` bản Humble trở lên.
- Cú pháp `TopicMetadata` của `rosbag2_py` có thể khác chút giữa các bản ROS 2 (Humble, Jazzy). Nếu lỗi, chỉnh `write_bag()`.
- `package.xml` cần `test_depend` cho `python3-pytest`, `ros2bag`, `rosbag2_py`, `rosbag2_storage_default_plugins`.

Nếu test tầng 2 báo không nhận được message: kiểm tra `ros2 topic list` trong cùng domain có node khác đang publish cùng topic không, và nhớ rằng `/odom`, `/mission/phase`, `/gimbal/target_angle_deg`, `/camera_info` là subscriber BEST_EFFORT nên publisher giả có thể là BEST_EFFORT hoặc RELIABLE.

Nếu test rosbag báo thiếu message: tăng thời gian chờ khởi động (`spin_for(collector, 3.0)` trong fixture `replay`), hoặc kiểm tra `colcon build` đã cài entry point `ibvs_controller` để `ros2 run ibvs ibvs_controller` chạy được.