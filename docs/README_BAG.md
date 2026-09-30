# Hướng Dẫn Xuất & Sử Dụng File `.bag` Cho Camera RealSense D430

Tài liệu này cung cấp hướng dẫn thu thập (record) và phát lại (playback) dữ liệu từ **cụm camera đo độ sâu Intel RealSense D430** dưới định dạng file `.bag` để bàn giao cho thành viên phụ trách xử lý dữ liệu camera trên Raspberry Pi (ROS 2 Humble / Python).

---

## 1. Đặc thù của Camera RealSense D430

> [!IMPORTANT]
> **Hệ thống mặc định sử dụng module Intel RealSense D430 (Active Stereo IR):**
> - **KHÔNG có cảm biến màu RGB:** Luồng hình ảnh thị giác chính là **Infrared 1 (IR1 / Left IR)** ở định dạng xám (`mono8` / `Y8`).
> - **KHÔNG có IMU tích hợp:** Khác với dòng D435i, module D430 không có cảm biến con quay hồi chuyển/gia tốc kế tích hợp sẵn.
> - **Luồng dữ liệu trong file `.bag`:** Chứa đồng bộ **Infrared 1 (IR1) + Depth (Z16) + Thông số nội kim (Intrinsics $K, D$) + Depth Scale + Hardware Timestamps**.
> - **Chế độ ghi:** Ghi thô không nén (Uncompressed Raw) để tối thiểu tải CPU trên thiết bị quay trong các đợt bay thử ngắn (~1 phút).

---

## 2. Cách thu dữ liệu ra file `.bag` (Trên máy kết nối Camera)

Thư mục lưu trữ mặc định: `recordings/` (tự động tạo nếu chưa có).

### Cách 2.1: Sử dụng công cụ thu hình `vision/record_bag.py` (Khuyến nghị)

Script chuyên dụng được tối ưu riêng cho RealSense D430:

```bash
# 1. Thu mặc định trong 60 giây (tự động đặt tên recordings/d430_capture_YYYYMMDD_HHMMSS.bag):
python3 vision/record_bag.py

# 2. Thu trong 60 giây với tên file tùy chọn:
python3 vision/record_bag.py --output recordings/d430_hpad_flight1.bag --duration 60

# 3. Thu ở chế độ không mở cửa sổ GUI (headless / SSH):
python3 vision/record_bag.py --output recordings/d430_hpad_flight1.bag --duration 60 --no-display

# 4. Thu không giới hạn thời gian (nhấn 'q' trên cửa sổ preview hoặc Ctrl+C trong terminal để dừng an toàn):
python3 vision/record_bag.py --output recordings/d430_manual_run.bag --duration 0
```

> [!TIP]
> Nếu môi trường Python chưa cài `pyrealsense2`, script sẽ **tự động fallback** sang gọi binary `/usr/local/bin/rs-record` đã được biên dịch sẵn trên hệ thống.

---

### Cách 2.2: Vừa chạy ArUco Detector vừa ghi file `.bag`

Nếu bạn muốn vừa kiểm tra nhận diện ArUco/PnP trực tiếp vừa lưu dữ liệu raw ra file:

```bash
python3 main_aruco_detector.py --record-bag recordings/d430_hpad_run1.bag
```

---

### Cách 2.3: Sử dụng trực tiếp công cụ `rs-record`

Nếu cần thu nhanh qua command line Linux thuần túy:

```bash
/usr/local/bin/rs-record -f recordings/d430_test_60s.bag -t 60
```

---

## 3. Cách sử dụng file `.bag` trên Raspberry Pi (Dành cho thành viên Nhúng)

Sau khi nhận file `.bag` từ máy test (ví dụ `d430_hpad_flight1.bag`), thành viên trên Raspberry Pi 5 có thể kiểm thử hệ thống vision theo các cách sau mà **không cần cắm camera vật lý**:

---

### Cách 3.1: Chạy trực tiếp qua Python script (`pyrealsense2`)

File `.bag` chứa đầy đủ ma trận nội kim $K$, hệ số méo $D$ và tỉ lệ độ sâu thực tế từ phần cứng D430:

```bash
# Phát lại file .bag (mặc định tự lặp lại khi hết file):
python3 main_aruco_detector.py --bag recordings/d430_hpad_flight1.bag

# Phát lại 1 lần duy nhất rồi tự thoát khi hết file (thích hợp cho script test tự động):
python3 main_aruco_detector.py --bag recordings/d430_hpad_flight1.bag --no-repeat-bag

# Chạy headless trên Pi qua SSH:
python3 main_aruco_detector.py --bag recordings/d430_hpad_flight1.bag --no-display
```

**Cơ chế hoạt động bên trong:**
- Pipeline tự động nhận diện file `.bag` có luồng Infrared 1 và Depth.
- Ảnh hồng ngoại đơn sắc `IR1` được chuyển đổi thành 3 kênh BGR để các thuật toán `ArUcoDetector` và `DepthMasker` xử lý mà không cần sửa đổi mã nguồn.

---

### Cách 3.2: Phát lại trên ROS 2 Humble (`realsense2_camera`)

Trên Raspberry Pi 5 đã cài ROS 2 Humble và driver `realsense2_camera`, chạy lệnh:

```bash
ros2 launch realsense2_camera rs_launch.py rosbag_filename:=/home/pi/recordings/d430_hpad_flight1.bag
```

Hoặc chạy node trực tiếp:

```bash
ros2 run realsense2_camera realsense2_camera_node --ros-args -p rosbag_filename:=/home/pi/recordings/d430_hpad_flight1.bag
```

#### Bảng Topic ROS 2 do RealSense D430 xuất ra:

| Topic ROS 2 | Kiểu bản tin (`msg type`) | Mô tả & Cách sử dụng trên Pi |
| :--- | :--- | :--- |
| `/camera/infra1/image_rect_raw` | `sensor_msgs/msg/Image` (mono8) | **Luồng hình ảnh chính.** Node ArUco detection trên Pi subscribe topic này để phát hiện H-Pad/ArUco. |
| `/camera/infra1/camera_info` | `sensor_msgs/msg/CameraInfo` | Ma trận nội tại $K, D$ của mắt IR1 để giải bài toán PnP. |
| `/camera/depth/image_rect_raw` | `sensor_msgs/msg/Image` (16UC1) | Mảng 2D độ sâu (khoảng cách tính bằng mm). Dùng cho Depth Masker và APF tránh vật cản. |
| `/camera/depth/camera_info` | `sensor_msgs/msg/CameraInfo` | Thông số nội tại của cảm biến độ sâu. |
| `/tf`, `/tf_static` | `tf2_msgs/msg/TFMessage` | Biến đổi hình học giữa `camera_link` và `camera_infra1_optical_frame`. |

> [!WARNING]
> **Lưu ý quan trọng khi lập trình node trên Pi:**
> 1. **KHÔNG** subscribe vào `/camera/color/...` vì module D430 không có camera màu RGB.
> 2. **KHÔNG** subscribe vào `/camera/imu` vì D430 không có IMU (dữ liệu quán tính của drone sẽ lấy trực tiếp từ PX4 qua MAVROS / uORB micro-XRCE).

---

### Cách 3.3: Mở kiểm tra trực quan trên PC bằng `realsense-viewer`

1. Cài đặt hoặc mở `realsense-viewer`:
   ```bash
   realsense-viewer
   ```
2. Nhấp vào nút **"Add Source"** ở góc trên cùng bên trái của giao diện.
3. Chọn file `d430_hpad_flight1.bag`.
4. Bật hiển thị **2D** (xem luồng ảnh IR1 và Depth colormap) hoặc **3D Point Cloud** (xem đám mây điểm 3D của bãi đáp H-Pad và vật cản).

---

## 4. Bảng kiểm tra tích hợp (Checklist bàn giao)

- [x] Ghi thử nghiệm 1 file `.bag` khoảng 60s có chứa ArUco marker H-Pad di chuyển.
- [x] Kiểm tra dung lượng file trong `recordings/` (~1.0 - 2.0 GB/phút với chuẩn uncompressed raw 640x480@30FPS).
- [x] Thử chạy lại trên máy test bằng `python3 main_aruco_detector.py --bag recordings/<tên_file>.bag --no-repeat-bag`.
- [x] Nén `.tar.gz` hoặc copy file sang USB / `scp` sang Raspberry Pi để kiểm thử ROS 2 node.
