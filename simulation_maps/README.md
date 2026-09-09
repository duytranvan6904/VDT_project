# PX4 SITL & ROS 2 Simulation Setup (Fast-Tracker & PX4-Avoidance Architecture)

Tài liệu này tổng hợp kiến trúc mô phỏng được chuẩn hóa dựa trên 2 dự án tham chiếu hàng đầu:
1. **ZJU FAST-Lab / Fast-Tracker** (Sử dụng luồng dữ liệu PointCloud2 từ cảm biến độ sâu + Odometry vị trí drone để quy hoạch quỹ đạo né vật cản).
2. **PX4 / PX4-Avoidance** (Kiến trúc chuẩn kết nối Gazebo SITL ↔ ROS 2 qua `ros_gz_bridge`).

---

## 🏗️ Nguyên lý Kiến trúc Mô phỏng

Trong các bài toán thuật toán tránh vật cản (APF, Fast-Tracker, Local Planner):
- Cảm biến độ sâu trên drone xuất dữ liệu trực tiếp dưới dạng **Đám mây điểm 3D (`PointCloud2`)** qua topic `/depth_camera/points`. Dữ liệu này chứa tọa độ X-Y-Z thực tế của các vật thể trước mắt drone.
- Vị trí và trạng thái của drone được đồng bộ tự động từ **Gazebo Odometry (`nav_msgs/msg/Odometry`)** qua topic `/model/x500_depth_0/odometry_with_covariance` về `/odom` và khung tọa độ `base_link` trong RViz2.

---

## 🚀 Hướng dẫn khởi chạy (3 Terminal + 1 bước chuẩn bị)

### Terminal 0: Sinh world một lần trước khi khởi động Gazebo
```bash
source /opt/ros/humble/setup.bash
python3 /home/duy/VDT_project/simulation_maps/launch_simulation.py --generate-world
```

Lệnh này ghi cùng một world SDF vào thư mục PX4 và `simulation_maps`. Không chạy lại
lệnh này trong lúc Gazebo đang mở; nếu chạy lại, phải khởi động lại PX4/Gazebo.

### Terminal 1: Khởi chạy PX4 SITL + Gazebo Sim
```bash
cd /home/duy/VDT_project/PX4-Autopilot
export GZ_SIM_RESOURCE_PATH="$PWD/Tools/simulation/gz/models:${GZ_SIM_RESOURCE_PATH:-}"
PX4_GZ_WORLD=obstacle_avoidance make px4_sitl gz_x500_depth
```
World được thêm `model://arucotag` tại `(4, 0, 0.02)` làm H-pad trong Terminal 0. Ảnh marker nằm ở
`PX4-Autopilot/Tools/simulation/gz/models/arucotag/arucotag.png` và model đã có collision
plane để dùng cho kiểm thử hạ cánh.

Gazebo được cấu hình sẵn `MinimalScene`, `InteractiveViewControl` và `GzSceneManager`.
Trong vùng **3D View**: kéo chuột trái để orbit, kéo chuột giữa để pan, lăn con lăn để
zoom; nhấp **Play (`▶`)** màu cam để chạy physics.

### Terminal 2: Khởi chạy Master Launch Node + RViz2
```bash
source /opt/ros/humble/setup.bash
python3 /home/duy/VDT_project/simulation_maps/launch_simulation.py
```

Terminal 2 phải được chạy **sau khi cửa sổ Gazebo đã mở và model `x500_depth_0`
đã xuất hiện**. Khi khởi chạy, terminal này phải in được các topic tương tự:

```text
/model/x500_depth_0/odometry_with_covariance
/camera
/depth_camera
/depth_camera/points
```

Nếu không thấy các dòng trên, hãy dừng toàn bộ mô phỏng và chạy lại Terminal 1;
không mở nhiều instance Gazebo/PX4 cùng lúc.

---

### Terminal 3: Khởi chạy QGroundControl

```bash
~/QGroundControl.AppImage
```

---

## 🖥️ Màn hình Quan sát RViz2

Cửa sổ RViz2 sẽ tự động mở lên với cấu hình hiển thị chuẩn:

| Tên hiển thị trong RViz2 | Topic | Loại dữ liệu | Ý nghĩa |
|--------------------------|-------|-------------|---------|
| **`GlobalMapPointCloud`** | `/map_generator/global_cloud` | `sensor_msgs/PointCloud2` | Bản đồ chướng ngại vật toàn cục (100% giống Gazebo) |
| **`SensorCameraPointCloud`** | `/depth_camera/points` | `sensor_msgs/PointCloud2` | Tầm nhìn camera độ sâu theo thời gian thực khi drone di chuyển |
| **`RGB Camera`** | `/camera` | `sensor_msgs/Image` | Ảnh RGB từ Oak-D-Lite trong model `x500_depth` |
| **`Depth Camera (mono8)`** | `/depth_camera/image_mono` | `sensor_msgs/Image` | Depth đã chuyển sang mono8 để xem trực tiếp trong RViz2 |
| **`DroneMarker`** | `/drone/marker` | `visualization_msgs/Marker` | Mô hình chiếc Drone 3D di chuyển thời gian thực theo tọa độ Gazebo |

---

## 📊 Theo dõi Trạng thái Chẩn đoán (Terminal 2)

Mỗi 3 giây, Terminal 2 sẽ in dòng log chẩn đoán trạng thái hệ thống:
```text
[INFO] [sim_master]: [STATUS] Gazebo Odom: [CONNECTED] | Camera PointCloud2: [RECEIVING] | Drone Pos: (0.00, 0.00, 3.00)
```
- **`Gazebo Odom: [CONNECTED]`**: Đã kết nối tọa độ thời gian thực của drone từ Gazebo.
- **`Camera PointCloud2: [RECEIVING]`**: Camera độ sâu trên drone đang quét chướng ngại vật và truyền về ROS 2.
- **`Drone Pos: (x, y, z)`**: Tọa độ X-Y-Z thực tế của drone.

### Kiểm tra camera nếu RViz2 vẫn báo No Image

```bash
ros2 topic list | grep -E 'camera|depth'
ros2 topic hz /camera
ros2 topic hz /depth_camera
ros2 topic echo --once /camera --field header
```

Camera Gazebo của `x500_depth` phát RGB ở `/camera`, depth ở `/depth_camera` và point
cloud ở `/depth_camera/points`. `launch_simulation.py` bridge các topic này, khởi chạy
`depth_to_image_node.py`, đồng thời phát TF `world -> base_link -> camera_link` để
RViz2 có thể resolve camera frame. Nếu dùng launch thủ công, cần chạy thêm:

```bash
python3 /home/duy/VDT_project/simulation_maps/depth_to_image_node.py
```

Trong RViz2, kiểm tra `Global Options > Fixed Frame = world`, `RGB Camera` bật và
`Topic = /camera`. Không chọn `Camera` display khi chưa bridge `CameraInfo`; display
`Image` đã được cấu hình sẵn và không phụ thuộc vào plugin camera rendering của RViz2.

### Kiểm tra liên kết Gazebo ↔ ROS 2 ↔ RViz2

Sau khi chạy Terminal 2, mở terminal thứ ba và kiểm tra:

```bash
source /opt/ros/humble/setup.bash
ros2 topic list | grep -E 'odom|camera|depth|hpad|drone'
ros2 topic hz /model/x500_depth_0/odometry_with_covariance
ros2 topic hz /depth_camera/points
ros2 topic echo --once /hpad/marker --field pose
```

Kết quả tối thiểu cần có:

| Kiểm tra | Kết quả đúng |
|---|---|
| Odometry | Có message liên tục, tần số > 10 Hz |
| Depth point cloud | Có message liên tục trên `/depth_camera/points` |
| H-pad | `/hpad/marker` trả về pose gần `(4.0, 0.0, 0.023)` |
| Drone RViz2 | `DroneMarker` di chuyển cùng vị trí drone trong Gazebo |

Nếu bridge odometry chết ngay, terminal launch sẽ báo `Odom ros_gz_bridge exited
immediately`. Khi đó kiểm tra đúng model name bằng:

```bash
gz model --list
gz topic -l | grep -E 'odometry|camera|depth'
```

Model mặc định của lệnh `gz_x500_depth` phải là `x500_depth_0`. Nếu tên khác,
sửa hằng `MODEL_NAME` ở đầu `launch_simulation.py` cho khớp rồi khởi động lại.
