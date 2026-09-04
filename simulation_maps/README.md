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

## 🚀 Hướng dẫn khởi chạy (2 Terminal)

### Terminal 1: Khởi chạy PX4 SITL + Gazebo Sim
```bash
cd /home/duy/VDT_project/PX4-Autopilot
PX4_GZ_WORLD=obstacle_avoidance make px4_sitl gz_x500_depth
```
*(Ngay khi Gazebo hiện ra, nhấp nút **Play (`▶`)** màu cam ở góc dưới bên trái).*

### Terminal 2: Khởi chạy Master Launch Node + RViz2
```bash
source /opt/ros/humble/setup.bash
python3 /home/duy/VDT_project/simulation_maps/launch_simulation.py
```

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
