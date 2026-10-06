# ArUco Detector — Guide

## 1. Package cần cài đặt ngoài

Có. Node dùng module `cv2.aruco`, nên cần OpenCV bản có aruco (từ 4.7 trở lên dùng `ArucoDetector`, bản cũ hơn vẫn chạy nhờ nhánh fallback trong `aruco_logic.py`).

```bash
python3 -m pip install numpy opencv-contrib-python
```

Không cài song song `opencv-python` và `opencv-contrib-python` trong cùng môi trường. Nếu hệ thống đã có `python3-opencv` qua apt và có `cv2.aruco` thì không cần cài thêm.

Package ROS2 cần có sẵn: `rclpy`, `sensor_msgs`, `geometry_msgs`, `std_msgs`, `vision_msgs`.

```bash
sudo apt install ros-$ROS_DISTRO-vision-msgs
```

Không có binary ngoài nào phải build riêng.

## 2. Nguyên lý hoạt động

```
Camera driver (IR1, mono8)          Camera driver (depth)
   /camera + /camera_info              /depth_camera
        |                                   |
        v                                   v
   aruco_node                     depth_to_image_node
        |                                   ^
        |  /hpad/mask_polygon (*)           |
        +-----------------------------------+
        |
        +--> /hpad/pose, /hpad/position_camera, /hpad/bbox,
             /hpad/detected, /hpad/annotated
```

(*) `depth_to_image_node` đã subscribe `/hpad/mask_polygon`, nhưng `aruco_node` hiện tại chưa publish topic này (tham số `mask_margin_percent` được khai báo nhưng chưa được dùng). Cho tới khi bổ sung publisher polygon, depth được chuẩn hóa mà không có mask.

Lúc khởi động `aruco_node`:
1. Khai báo tham số, tạo `ArUcoDetector` với camera matrix dự phòng tính từ `fallback_horizontal_fov_rad` và kích thước ảnh mặc định 640x480. Tên `dictionary` không hợp lệ (không bắt đầu bằng `DICT_` hoặc không có trong `cv2.aruco`) làm node dừng với `ValueError`.
2. Tạo publisher (QoS mặc định, depth 10) và subscriber ảnh, `CameraInfo` (QoS `BEST_EFFORT`, `KEEP_LAST`, depth 3).

Mỗi frame ảnh (`image_callback`):
1. `ros_image_to_array` đổi message thành mảng numpy (mono8 giữ nguyên 2D, `mono16`/`16uc1` lấy byte cao đổi về 8 bit, RGB/RGBA/BGRA đổi về BGR). Encoding không hỗ trợ hoặc buffer ngắn hơn `step × height` thì cảnh báo và bỏ frame (không publish gì).
2. Nếu chưa nhận `CameraInfo` và kích thước ảnh khác kích thước đang dùng, cập nhật camera matrix dự phòng theo kích thước ảnh thực.
3. `process_frame`: chuyển xám, `detectMarkers`, chọn marker có `marker_id` đúng, giải PnP bằng `SOLVEPNP_IPPE_SQUARE`.
4. `filter_detections` loại detection có khoảng cách nhỏ hơn `min_detection_distance_m` hoặc `z` nhỏ hơn `min_z_m` (so sánh bao gồm dấu bằng, nên `z` âm bị loại ngay cả khi để mặc định 0.0).
5. Nếu `require_camera_info` bật mà chưa có `CameraInfo` hợp lệ thì bỏ detection và cảnh báo.
6. Publish cờ `/hpad/detected` mỗi frame. Nếu có detection thì publish pose, point và bbox của detection đầu tiên.
7. Publish ảnh annotate mỗi frame (kể cả khi không thấy marker), log thống kê mỗi 2 giây.

`CameraInfo` bị bỏ qua nếu `K[0]` hoặc `K[4]` không dương; nếu `D` rỗng thì dùng méo bằng 0. Stamp của pose và point lấy từ header ảnh; header stamp bằng 0 thì dùng đồng hồ của node.

`depth_to_image_node` giữ polygon mới nhất từ `/hpad/mask_polygon`. Khi nhận depth (`32FC1`, `R_FLOAT32` hoặc `16UC1` tính theo mm; encoding khác bị bỏ im lặng), nếu polygon có stamp lệch không quá 0.2 s so với depth thì vùng polygon (cần từ 3 đỉnh trở lên, tọa độ làm tròn về pixel) được gán NaN trước khi chuẩn hóa về mono8. Depth được kẹp trong 0.2–8.0 m rồi ánh xạ tuyến tính về 0–255; NaN và +inf thành 255 (xa nhất, tức nền), -inf thành 0. Mất marker thì polygon cũ quá hạn và mask tự biến mất.

Lưu ý:
- Pose nằm trong hệ tọa độ optical của camera (`camera_frame_id`), quaternion thứ tự w, x, y, z từ `geometry_utils` được gán đúng vào các trường `w`, `x`, `y`, `z` của ROS ở `message_builders`.
- Chỉ detection đầu tiên (marker có `marker_id` đúng) được publish; marker ID khác bị bỏ qua.
- `/hpad/bbox` là hộp bao song song trục của 4 góc, tính theo pixel (`center` là tâm, `theta = 0`).
- Polygon tính theo pixel IR1, chỉ khớp với depth khi depth được align với IR1 và cùng độ phân giải.
- Node không tự chuyển pose sang hệ body hay local NED, phần này do node phía sau xử lý.

## 3. Cách chạy

```bash
ros2 run aruco_detector aruco_node
ros2 run aruco_detector depth_to_image_node
```

Chạy kèm tham số tùy chỉnh:

```bash
ros2 run aruco_detector aruco_node --ros-args \
  -p marker_id:=42 \
  -p marker_size_m:=0.15 \
  -p image_topic:=/camera/infra1/image_rect_raw \
  -p camera_info_topic:=/camera/infra1/camera_info
```

Danh sách tham số của `aruco_node`:

| Tên | Mặc định | Ý nghĩa |
|---|---|---|
| `marker_id` | 42 | ID marker cần bám |
| `marker_size_m` | 0.15 | Cạnh ngoài của marker, tính bằng mét |
| `dictionary` | DICT_6X6_50 | Tên dictionary ArUco |
| `min_detection_distance_m` | 0.0 | Bỏ detection gần hơn giá trị này |
| `min_z_m` | 0.0 | Bỏ detection có z nhỏ hơn giá trị này |
| `image_topic` | /camera | Topic ảnh xám từ IR1 |
| `camera_info_topic` | /camera_info | Topic intrinsics, phải cùng camera với ảnh |
| `camera_frame_id` | camera_optical_frame | Frame id gắn vào pose và point |
| `fallback_horizontal_fov_rad` | 1.52 | HFOV dùng khi chưa có `CameraInfo` (IR D430 khoảng 87 độ) |
| `require_camera_info` | true | Chặn publish pose khi chưa nhận `CameraInfo` |
| `mask_margin_percent` | 0.15 | Hệ số nới polygon mask quanh marker (chưa được dùng trong code hiện tại) |

`depth_to_image_node` không có tham số; các hằng số (dải depth 0.2–8.0 m, tuổi tối đa của polygon 0.2 s) nằm trong code.

Topic đầu vào:

| Topic | Kiểu | Node | Nội dung |
|---|---|---|---|
| `image_topic` (`/camera`) | `Image` | `aruco_node` | Ảnh IR1, BEST_EFFORT |
| `camera_info_topic` (`/camera_info`) | `CameraInfo` | `aruco_node` | Intrinsics, BEST_EFFORT |
| `/depth_camera` | `Image` (`32FC1`/`16UC1`) | `depth_to_image_node` | Depth, BEST_EFFORT |
| `/hpad/mask_polygon` | `PolygonStamped` | `depth_to_image_node` | Polygon vùng mask, pixel IR1 |

Topic đầu ra:

| Topic | Kiểu | Nội dung |
|---|---|---|
| `/hpad/detected` | `std_msgs/Bool` | Có thấy marker hay không, publish mỗi frame |
| `/hpad/pose` | `PoseStamped` | Pose marker trong frame camera |
| `/hpad/position_camera` | `PointStamped` | Vị trí marker trong frame camera |
| `/hpad/bbox` | `vision_msgs/BoundingBox2D` | Hộp bao theo pixel |
| `/hpad/mask_polygon` | `PolygonStamped` | 4 đỉnh đã nới margin, pixel IR1 (chưa được `aruco_node` publish) |
| `/hpad/annotated` | `Image` (bgr8) | Ảnh đã vẽ viền, ID và trục, publish mỗi frame |
| `/depth_camera/image_mono` | `Image` (mono8) | Depth đã mask và chuẩn hóa |

## 4. Cách debug

Log thống kê tự in mỗi 2 giây:

```
[INFO] [aruco_node]: frames=120 detected=118 (98.3%) camera_info=True
```

Kiểm tra topic ảnh đang đến và đúng encoding:

```bash
ros2 topic info /camera -v
ros2 topic echo /camera --field encoding --once
ros2 topic hz /camera
```

Kiểm tra `CameraInfo` hợp lệ (K khác 0, đúng độ phân giải):

```bash
ros2 topic echo /camera_info --once
```

Kiểm tra pose và cờ phát hiện:

```bash
ros2 topic echo /hpad/detected
ros2 topic echo /hpad/pose
```

Xem ảnh annotate:

```bash
ros2 run rqt_image_view rqt_image_view /hpad/annotated
```

Kiểm tra stamp của ảnh và depth có lệch nhau không (mask chỉ áp khi lệch không quá 0.2 s):

```bash
ros2 topic echo /camera --field header.stamp --once
ros2 topic echo /depth_camera --field header.stamp --once
```

Nếu log báo `unsupported image encoding`: encoding không nằm trong danh sách hỗ trợ (`mono8`, `8uc1`, `mono16`, `16uc1`, `rgb8`, `bgr8`, `rgba8`, `bgra8`) hoặc buffer ngắn hơn `step × height`. Đổi topic hoặc cấu hình driver. Khi đó `/hpad/detected` và `/hpad/annotated` đều không có message.

Nếu log báo `camera_info not received, pose suppressed`: sai `camera_info_topic`, `K` trong `CameraInfo` bằng 0, hoặc QoS không khớp (publisher `RELIABLE` thì subscriber `BEST_EFFORT` vẫn nhận được, ngược lại thì không). Kiểm tra bằng `ros2 topic info <topic> -v`. Trong lúc đó `/hpad/detected` luôn là `false`. Khi bench chưa có `CameraInfo`, đặt `require_camera_info:=false` để dùng camera matrix dự phòng (z chỉ gần đúng).

Nếu `detected` thấp dù marker rõ trong ảnh: kiểm tra `dictionary` và `marker_id`, độ phơi sáng IR (marker có thể bị cháy sáng hoặc quá tối), và chế độ projector của D430 (chấm IR có thể phủ lên marker, nên tắt emitter khi bám marker). Cũng kiểm tra `min_z_m` và `min_detection_distance_m` có đang loại detection không.

Nếu khoảng cách z sai hệ số cố định: sai `marker_size_m` (phải đo cạnh ngoài của phần đen), hoặc `CameraInfo` thuộc stream khác (ví dụ lấy của color thay vì infra1). Z tỉ lệ thuận với `marker_size_m` và với tiêu cự trong `CameraInfo`.

Nếu pose nhảy lật giữa hai hướng: hiện tượng mơ hồ của IPPE khi marker nhìn gần chính diện và ở xa. Có thể thêm kiểm tra reprojection error hoặc làm mượt phía node sau.

Nếu vùng mask không xuất hiện trên depth: `aruco_node` hiện chưa publish `/hpad/mask_polygon` (xem chú thích ở mục 2), nên mask chỉ xuất hiện khi có node khác publish polygon. Với polygon từ nguồn khác, kiểm tra topic có message không, depth có cùng độ phân giải với IR1 không, polygon có từ 3 đỉnh trở lên không, và chênh lệch stamp có vượt 0.2 s không.

Nếu `/depth_camera/image_mono` không có message: encoding depth không phải `32FC1`, `R_FLOAT32` hoặc `16UC1` (node bỏ im lặng, không có log). Kiểm tra bằng `ros2 topic echo /depth_camera --field encoding --once`.

## 5. Test

Test chia 3 tầng, nằm trong `test/` của package. Tầng 1 không cần ROS 2 chạy (riêng các test message cần source ROS 2 để import message); tầng 2 và 3 cần source ROS 2 và workspace đã build.

Cài đặt:

```bash
python3 -m pip install pytest
sudo apt install ros-$ROS_DISTRO-launch-pytest ros-$ROS_DISTRO-ros2bag ros-$ROS_DISTRO-rosbag2-storage-default-plugins
```

`test/` không có `__init__.py` để `from marker_utils import ...` và `from ros_utils import ...` hoạt động.

Ảnh test được sinh tổng hợp trong `marker_utils.py`: marker dán thẳng (không xoay trừ test xoay trong mặt phẳng ảnh) lên nền trắng 640x480, kích thước theo `f × cạnh / z` với `f` tính từ HFOV 1.52 rad. Giá trị z kỳ vọng được tính lại từ số pixel làm tròn và so với dung sai khoảng 5%. Các test dùng `marker_size_m = 0.3` để marker đủ lớn ở khoảng cách 1 đến 1.5 m.

### Tầng 1: logic thuần

Không tạo node, chạy trong vài giây.

| File | Kiểm tra |
|---|---|
| `test_geometry_utils.py` | Quaternion từ ma trận quay (cả 4 nhánh của hàm, 300 phép quay ngẫu nhiên khôi phục lại đúng ma trận), `rvec_to_quaternion`, hộp bao của 4 góc |
| `test_camera_model.py` | Camera matrix dự phòng theo HFOV, `zero_distortion`, `camera_info_to_intrinsics` (có/không có méo, `K` không hợp lệ, không dùng chung bộ nhớ với message) |
| `test_image_conversion.py` | mono8, padding cuối hàng (`step`), RGB/RGBA/BGRA sang BGR, mono16 little/big-endian, alias `8uc1`/`16uc1`, chữ hoa/thường, encoding không hỗ trợ, buffer ngắn, `step` không dương |
| `test_detection_filter.py` | Ngưỡng khoảng cách và `z` (bao gồm dấu bằng), thứ tự giữ nguyên, `z` âm bị loại ở ngưỡng mặc định |
| `test_aruco_logic.py` | `create_dictionary` (hợp lệ, tên sai), tham số detector, điểm 3D của marker, `build_pose_dict`, phát hiện marker thật trên ảnh tổng hợp (độ chính xác z, offset x/y, xoay trong mặt phẳng ảnh, chọn đúng ID giữa nhiều marker, ảnh trống, ảnh BGR, sai dictionary), `set_camera_parameters` (copy dữ liệu, z đổi theo tiêu cự), z tỉ lệ với `marker_size_m` |
| `test_annotation.py` | `ensure_bgr`, vẽ viền/nhãn/trục (màu và vị trí), không sửa ảnh gốc, hình chiếu trục |
| `test_message_builders.py` | `resolve_stamp`, pose/point/bbox/ảnh bgr8 (cần source ROS 2) |
| `test_depth_logic.py` | `decode_depth_meters`, chuẩn hóa mono8 (đầu mút, kẹp, NaN/inf), `build_mono8_message`, `stamp_to_seconds`, `apply_polygon_mask` (không sửa input, cần từ 3 đỉnh) |

```bash
cd ros2_ws/src/aruco_detector
python3 -m pytest test/test_geometry_utils.py test/test_camera_model.py test/test_image_conversion.py test/test_detection_filter.py test/test_aruco_logic.py test/test_annotation.py test/test_message_builders.py test/test_depth_logic.py -v
```

Nếu chạy từ thư mục khác và import lỗi, thêm đường dẫn package:

```bash
PYTHONPATH=ros2_ws/src python3 -m pytest ros2_ws/src/aruco_detector/test -v
```

### Tầng 2: node ROS 2 với publisher giả

Tạo node thật cùng một node probe trong cùng process, publish ảnh/`CameraInfo`/depth giả rồi kiểm tra topic đầu ra. Tham số được truyền qua `--ros-args -p` nên mỗi test khởi tạo `rclpy` riêng (fixture `make_rig` trong `conftest.py`).

| File | Kiểm tra |
|---|---|
| `test_aruco_node_ros.py` | Khung trống chỉ publish `detected = false`, marker publish đủ pose/point/bbox/annotated (vị trí, tâm bbox, quaternion chuẩn), định dạng ảnh annotate, `camera_frame_id`, stamp (0 thì dùng đồng hồ node, khác 0 giữ nguyên), chặn pose khi thiếu hoặc sai `CameraInfo`, dùng matrix dự phòng khi `require_camera_info:=false`, tiêu cự trong `CameraInfo` làm đổi z, `min_z_m` và `min_detection_distance_m`, `marker_id`, `marker_size_m`, encoding `mono16`/`rgb8`, encoding không hỗ trợ không publish gì, độ phân giải khác 640x480 |
| `test_depth_node_ros.py` | Chuẩn hóa `32FC1` và `16UC1`, giữ nguyên header, encoding không hỗ trợ, mask khi polygon còn mới, polygon lệch trong dung sai 0.2 s vẫn áp, polygon quá cũ hoặc không có hoặc dưới 3 đỉnh thì bỏ qua, dùng polygon mới nhất |

Dùng chung `ros_utils.py` (tạo `Rig`) và `marker_utils.py` (ảnh, `Image` và `CameraInfo` giả).

```bash
python3 -m pytest test/test_aruco_node_ros.py test/test_depth_node_ros.py -v
```

Test depth tự publish polygon để kiểm tra mask, vì `aruco_node` hiện chưa publish `/hpad/mask_polygon`.

### Tầng 3: rosbag

`test_aruco_rosbag.py` tự sinh bag tổng hợp (`/camera` và `/camera_info` ở 10 Hz, mono8 640x480), launch `aruco_node` với `marker_size_m:=0.3` và `marker_id:=42`, sau 5 s thì `ros2 bag play`, rồi kiểm tra các topic đầu ra.

| Đoạn | Nội dung ảnh | Thời lượng |
|---|---|---|
| 1 | Không có marker | 1.5 s |
| 2 | Marker ở z = 1.0 m | 2.0 s |
| 3 | Marker ở z = 1.5 m | 2.0 s |
| 4 | Không có marker | 1.0 s |

Kỳ vọng: `/hpad/detected` bắt đầu và kết thúc bằng `false`, ít nhất 32 và nhiều nhất 42 frame là `true` (tối đa 40 frame có marker); ít nhất 40 ảnh annotate; z trung vị gần 1.0 m (±0.1) ở đoạn 2 và gần 1.5 m (±0.15) ở đoạn 3, đoạn gần xuất hiện trước đoạn xa. Bag chỉ có ảnh hồng ngoại nên không kiểm tra `depth_to_image_node`.

```bash
python3 -m pytest test/test_aruco_rosbag.py -v
```

Để dùng bag thật, ghi bag rồi đổi `bag_dir` trong `generate_test_description()` và chỉnh lại các giá trị kỳ vọng theo kịch bản đã ghi (đặc biệt `marker_size_m` phải đúng với marker thật):

```bash
ros2 bag record -o sample_aruco /camera /camera_info
```

### Chạy toàn bộ

```bash
colcon build --packages-select aruco_detector
source install/setup.bash
colcon test --packages-select aruco_detector --event-handlers console_direct+
colcon test-result --verbose
```

### Lưu ý khi chạy test

- Test tầng 2 và 3 dùng thời gian thật nên có thể chập chờn trên máy chậm; chạy lại một lần trước khi kết luận lỗi.
- Không chạy song song nhiều test ROS trong cùng `ROS_DOMAIN_ID`, vì các node dùng chung tên topic. Nếu cần chạy cạnh hệ thống đang chạy, đặt domain riêng: `ROS_DOMAIN_ID=77 python3 -m pytest ...`.
- Test phát hiện marker phụ thuộc phiên bản OpenCV (cả `ArucoDetector` mới lẫn nhánh `detectMarkers` cũ đều được dùng tự động); độ chính xác z kỳ vọng trong khoảng 5% đến 6%.
- Test rosbag cần `ros2 bag play` trong `PATH` (package `ros2bag`) và plugin lưu trữ `sqlite3`. Bag gồm khoảng 65 ảnh 640x480 nên nặng khoảng 20 MB trong thư mục tạm.
- Test rosbag launch `aruco_node` qua `launch_ros` với tên executable `aruco_node`; nếu entry point trong `setup.py` đặt tên khác thì phải sửa trong file test.

Nếu test tầng 2 báo không nhận được message: kiểm tra `ros2 topic list` trong cùng domain có node khác đang publish cùng topic không, và nhớ rằng `aruco_node` subscribe ảnh với QoS BEST_EFFORT nên publisher giả cũng phải là BEST_EFFORT hoặc RELIABLE.

Nếu test rosbag báo thiếu message hoặc `detected` luôn `false`: tăng `BAG_DELAY` trong `test_aruco_rosbag.py` (node cần thời gian khởi động trước khi bag phát), hoặc kiểm tra `marker_size_m`, `dictionary` có khớp với marker trong ảnh không.