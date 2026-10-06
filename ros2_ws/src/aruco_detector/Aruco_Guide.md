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
        |  /hpad/mask_polygon               |
        +-----------------------------------+
        |
        +--> /hpad/pose, /hpad/position_camera, /hpad/bbox,
             /hpad/detected, /hpad/annotated
```

Lúc khởi động `aruco_node`:
1. Khai báo tham số, tạo `ArUcoDetector` với camera matrix dự phòng tính từ `fallback_horizontal_fov_rad` và kích thước ảnh mặc định 640x480.
2. Tạo publisher và subscriber (QoS `BEST_EFFORT`, `KEEP_LAST`, depth 3).

Mỗi frame ảnh (`image_callback`):
1. `ros_image_to_array` đổi message thành mảng numpy (mono8 giữ nguyên 2D, `mono16` đổi về 8 bit, RGB/RGBA đổi về BGR).
2. Nếu chưa nhận `CameraInfo`, cập nhật camera matrix dự phòng theo kích thước ảnh thực.
3. `process_frame`: chuyển xám, `detectMarkers`, chọn marker có `marker_id` đúng, giải PnP bằng `SOLVEPNP_IPPE_SQUARE`.
4. `filter_detections` loại detection có khoảng cách nhỏ hơn `min_detection_distance_m` hoặc `z` nhỏ hơn `min_z_m`.
5. Nếu `require_camera_info` bật mà chưa có `CameraInfo` thì bỏ detection và cảnh báo.
6. Publish cờ `/hpad/detected` mỗi frame. Nếu có detection thì publish pose, point, bbox và polygon của detection đầu tiên.
7. Publish ảnh annotate, log thống kê mỗi 2 giây.

`depth_to_image_node` giữ polygon mới nhất từ `/hpad/mask_polygon`. Khi nhận depth, nếu polygon có stamp lệch không quá 0.2 s so với depth thì vùng polygon được gán NaN trước khi chuẩn hóa về mono8 (NaN ra giá trị xa nhất, tức nền). Mất marker thì polygon cũ quá hạn và mask tự biến mất.

Lưu ý:
- Pose nằm trong hệ tọa độ optical của camera (`camera_frame_id`), quaternion thứ tự w, x, y, z được đổi sang x, y, z, w của ROS ở `message_builders`.
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
| `mask_margin_percent` | 0.15 | Hệ số nới polygon mask quanh marker |

Topic đầu ra:

| Topic | Kiểu | Nội dung |
|---|---|---|
| `/hpad/detected` | `std_msgs/Bool` | Có thấy marker hay không, publish mỗi frame |
| `/hpad/pose` | `PoseStamped` | Pose marker trong frame camera |
| `/hpad/position_camera` | `PointStamped` | Vị trí marker trong frame camera |
| `/hpad/bbox` | `vision_msgs/BoundingBox2D` | Hộp bao theo pixel |
| `/hpad/mask_polygon` | `PolygonStamped` | 4 đỉnh đã nới margin, pixel IR1 |
| `/hpad/annotated` | `Image` (bgr8) | Ảnh đã vẽ viền, ID và trục |
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

Nếu log báo `unsupported image encoding`: encoding không nằm trong danh sách hỗ trợ (`mono8`, `8uc1`, `mono16`, `16uc1`, `rgb8`, `bgr8`, `rgba8`, `bgra8`). Đổi topic hoặc cấu hình driver.

Nếu log báo `camera_info not received, pose suppressed`: sai `camera_info_topic`, hoặc QoS không khớp (publisher `RELIABLE` thì subscriber `BEST_EFFORT` vẫn nhận được, ngược lại thì không). Kiểm tra bằng `ros2 topic info <topic> -v`.

Nếu `detected` thấp dù marker rõ trong ảnh: kiểm tra `dictionary` và `marker_id`, độ phơi sáng IR (marker có thể bị cháy sáng hoặc quá tối), và chế độ projector của D430 (chấm IR có thể phủ lên marker, nên tắt emitter khi bám marker).

Nếu khoảng cách z sai hệ số cố định: sai `marker_size_m` (phải đo cạnh ngoài của phần đen), hoặc `CameraInfo` thuộc stream khác (ví dụ lấy của color thay vì infra1).

Nếu pose nhảy lật giữa hai hướng: hiện tượng mơ hồ của IPPE khi marker nhìn gần chính diện và ở xa. Có thể thêm kiểm tra reprojection error hoặc làm mượt phía node sau.

Nếu vùng mask không xuất hiện trên depth: kiểm tra `/hpad/mask_polygon` có message không, depth có cùng độ phân giải với IR1 không, và chênh lệch stamp có vượt 0.2 s không.

## Unit test

Test chạy trên ảnh marker tổng hợp (tạo bằng `generateImageMarker`), không cần camera và không cần ROS2. Các test phụ thuộc `geometry_msgs`, `vision_msgs` (nhóm `TestMessageBuilders`) tự bỏ qua nếu môi trường chưa source ROS2. Test cho `mono16` và `expand_polygon` tự bỏ qua nếu patch tương ứng chưa được áp dụng.

Nội dung được kiểm tra: chuyển đổi quaternion, bbox và nới polygon, công thức camera matrix dự phòng, đọc `CameraInfo`, giải mã ảnh (stride, mono16, RGB sang BGR), bộ lọc detection, phát hiện marker và độ chính xác z, vẽ annotate, và các message builder.

```bash
python3 -m pip install pytest
cd ros2_ws/src/aruco_detector
python3 -m pytest test/test_aruco_logic.py
```

Nếu chạy từ thư mục khác và import lỗi, thêm đường dẫn package:

```bash
PYTHONPATH=ros2_ws/src python3 -m pytest ros2_ws/src/aruco_detector/test/test_aruco_logic.py
```

Để chạy cả nhóm message builder, source ROS2 trước khi chạy pytest:

```bash
source /opt/ros/$ROS_DISTRO/setup.bash
```