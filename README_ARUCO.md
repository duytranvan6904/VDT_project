# Hướng Dẫn Cài Đặt & Sử Dụng: ArUco Detection & PnP Pose Estimation (RealSense D435)

> **Người phụ trách**: Duy (Vision / Estimation Lead)  
> **Dự án**: Quadrotor H-Pad Tracking, Obstacle Avoidance & Precision Landing  
> **Cảm biến**: Intel RealSense D435 / D435i  

---

## 1. Danh Sách Package Cần Thiết & Cài Đặt

### 1.1. Các thư viện Python yêu cầu

1. **`pyrealsense2`**: SDK 2.0 Python Wrapper của Intel RealSense để truy xuất luồng Color (RGB) + Depth (Z16) và đọc thông số nội kim (Intrinsics Matrix $K$) từ phần cứng camera.
2. **`opencv-python` / `opencv-contrib-python`**: Nhận diện ArUco marker (`cv2.aruco`) và tính toán PnP Pose Estimation (`cv2.solvePnP`).
3. **`numpy`**: Tính toán ma trận và đại số tuyến tính 3D.
4. **`scipy`**: Chuyển đổi góc xoay (Ma trận xoay $\leftrightarrow$ Euler RPY $\leftrightarrow$ Quaternion).
5. **`PyYAML`**: Đọc/ghi file cấu hình calibration (nếu cần).

### 1.2. Hướng dẫn cài đặt trên máy cá nhân

Do máy đã cài sẵn **`librealsense`** (C++ API / shared libraries trong `/usr/local/lib`), bạn cài các Python package qua `pip` bằng lệnh sau:

```bash
pip install pyrealsense2 opencv-contrib-python numpy scipy matplotlib PyYAML
```

> **Lưu ý**: Nếu máy cài `librealsense` từ source với flag Python bindings, module `pyrealsense2` đã có sẵn. Bạn có thể kiểm tra bằng lệnh:
> ```bash
> python3 -c "import pyrealsense2 as rs; print(rs.__version__)"
> ```

---

## 2. Cấu Trúc Mã Nguồn

```text
VDT_project/
├── vision/
│   ├── __init__.py          # Package initializer
│   ├── realsense_stream.py  # Quản lý luồng RGB-D & Tự động lấy Camera Intrinsics
│   ├── aruco_detector.py    # Class ArUcoDetector (PnP Pose Estimation qua cv2.solvePnP)
│   └── utils.py             # Chuyển đổi RPY, Quaternion, Bounding Box cho Depth Masking
├── tests/
│   └── test_aruco_detector.py # Unit tests kiểm tra thuật toán với marker giả lập
├── main_aruco_detector.py   # Script thực thi live stream & hiển thị GUI/Console
├── requirements.txt         # Danh sách package phụ thuộc
└── README_ARUCO.md          # Hướng dẫn chi tiết
```

---

## 3. Các Tính Năng Nổi Bật Đã Triển Khai

1. **Tự động lấy thông số Nội kim (Camera Intrinsics)**:
   - Truy xuất trực tiếp $f_x, f_y, c_x, c_y$ và hệ số méo $D$ từ luồng Color của RealSense D435 qua SDK (`profile.get_stream().as_video_stream_profile().get_intrinsics()`).
2. **Thuật toán PnP Pose Estimation chính xác cao**:
   - Sử dụng `cv2.solvePnP` kết hợp phương pháp **`SOLVEPNP_IPPE_SQUARE`** chuyên biệt cho ArUco planar marker.
   - Trích xuất vị trí 3D $(X, Y, Z)$ tính bằng **mét**, khoảng cách Euclidean, góc Euler (Roll, Pitch, Yaw tính bằng **độ**) và Quaternion $(q_w, q_x, q_y, q_z)$.
3. **Trích xuất Bounding Box 2D cho Task Depth Masking (N3)**:
   - Tự động tính Bounding Box 2D `(x_min, y_min, w, h)` có mở rộng margin 15% để xóa vùng H-Pad trên luồng Depth map (tránh H-Pad bị coi là vật cản trong thuật toán APF).
4. **Kiểm chứng khoảng cách song song (Double-Validation)**:
   - So sánh khoảng cách $Z_{PnP}$ thu được từ giải PnP với khoảng cách $Z_{depth}$ đo trực tiếp bằng cảm biến độ sâu RealSense tại tọa độ pixel tâm marker.
5. **Hỗ trợ tương thích mọi phiên bản OpenCV**:
   - Tương thích từ OpenCV 4.5 đến 4.10+ (xử lý cả `detectMarkers` truyền thống và `cv2.aruco.ArucoDetector`).

---

## 4. Hướng Dẫn Chạy Chương Trình

### 4.1. Chạy Demo thực tế với Camera RealSense D435

Cắm camera RealSense D435 qua cổng USB 3.0 và chạy script:

```bash
python3 main_aruco_detector.py --marker-size 0.15 --dict DICT_4X4_50
```

Các tham số tùy chọn:
- `--marker-size`: Kích thước cạnh marker ArUco (tính bằng mét). Mặc định: `0.15` (15cm).
- `--dict`: Từ điển ArUco (`DICT_4X4_50`, `DICT_5X5_100`, `DICT_6X6_250`, v.v.). Mặc định: `DICT_4X4_50`.
- `--width`, `--height`, `--fps`: Độ phân giải và FPS stream (Mặc định: `640x480 @ 30FPS`).
- `--no-display`: Chạy ở chế độ không màn hình (Headless/CI mode).

### 4.2. Chạy Unit Test kiểm tra thuật toán

```bash
python3 -m unittest discover -s tests
```

---

## 5. Đóng Gói Sang ROS 2 Node (`vision_node`)

Các class `RealSenseCamera` và `ArUcoDetector` đã được mô đun hóa dạng Object-Oriented. Khi tích hợp vào ROS 2 Humble về sau, bạn chỉ cần import vào ROS node:

```python
from vision import ArUcoDetector, RealSenseCamera

# Publish topics:
# /hpad/pose -> geometry_msgs/msg/PoseStamped
# /hpad/bbox -> vision_msgs/msg/BoundingBox2D (phục vụ Depth Masking)
```
