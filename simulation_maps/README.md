# PX4 SITL & ROS 2 Simulation Setup (Fast-Tracker & PX4-Avoidance Architecture)

Tài liệu này tổng hợp kiến trúc mô phỏng được chuẩn hóa dựa trên 2 dự án tham chiếu hàng đầu:
1. **ZJU FAST-Lab / Fast-Tracker** (Sử dụng luồng dữ liệu PointCloud2 từ cảm biến độ sâu + Odometry vị trí drone để quy hoạch quỹ đạo né vật cản).
2. **PX4 / PX4-Avoidance** (Kiến trúc chuẩn kết nối Gazebo SITL ↔ ROS 2 qua `ros_gz_bridge`).

---

## 🏗️ Nguyên lý Kiến trúc Mô phỏng

Trong các bài toán thuật toán tránh vật cản (APF, Fast-Tracker, Local Planner):
- Cảm biến độ sâu trên drone xuất dữ liệu trực tiếp dưới dạng **Đám mây điểm 3D (`PointCloud2`)** qua topic `/depth_camera/points`. Dữ liệu này chứa tọa độ X-Y-Z thực tế của các vật thể trước mắt drone.
- Vị trí và trạng thái của drone được đồng bộ tự động từ **Gazebo Odometry (`nav_msgs/msg/Odometry`)** qua topic `/model/x500_depth_0/odometry_with_covariance` về `/odom` và khung tọa độ `base_link` trong RViz2.

### Bắt buộc: đồng bộ phiên bản Gazebo với ROS bridge

PX4 hiện dùng Gazebo Harmonic (`gz-msgs10`, `gz-transport13`). ROS Humble có
thể đồng thời cài bridge Fortress (`ignition-msgs8`, `ignition-transport11`);
bridge Fortress sẽ chạy nhưng không đọc được topic `gz.msgs.*`, thường lặp lỗi
`Unknown message type [8]/[9]` và báo `No subscribers` trong `gz topic -i`.

Kiểm tra backend hiện tại:

```bash
ldd /opt/ros/humble/lib/ros_gz_bridge/parameter_bridge \
  | grep -E 'gz-(msgs|transport)|ignition-(msgs|transport)'
```

Nếu kết quả là `libignition-*`, cài bridge Harmonic (gói này thay thế bridge
Fortress vì hai gói dùng chung tên ROS package):

```bash
sudo apt update
sudo apt install ros-humble-ros-gzharmonic-bridge
```

Kết quả đúng phải có `libgz-msgs10` và `libgz-transport13`. Launcher sẽ tự dừng
và báo lỗi rõ ràng nếu phát hiện bridge vẫn là bản Ignition.

---

## 🚀 Hướng dẫn khởi chạy (5 Terminal + 1 bước chuẩn bị)

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
export GZ_PARTITION=vdt_harmonic
export GZ_SIM_RESOURCE_PATH="$PWD/Tools/simulation/gz/models:${GZ_SIM_RESOURCE_PATH:-}"
PX4_GZ_WORLD=obstacle_avoidance make px4_sitl gz_x500_depth
```
World được thêm `model://arucotag` tại `(4, 0, 0.02)` làm H-pad trong Terminal 0. Mặt H-pad
hiện có kích thước `0.90 x 0.90 m`; texture là marker ID 42 trong
`vision/marker42.png`. Model có collision plane để dùng cho kiểm thử hạ cánh.

Gazebo được cấu hình sẵn `MinimalScene`, `InteractiveViewControl` và `GzSceneManager`.
Trong vùng **3D View**: kéo chuột trái để orbit, kéo chuột giữa để pan, lăn con lăn để
zoom; nhấp **Play (`▶`)** màu cam để chạy physics.

### Terminal 2: Khởi chạy Master Launch Node + RViz2
```bash
source /opt/ros/humble/setup.bash
export GZ_PARTITION=vdt_harmonic
python3 /home/duy/VDT_project/simulation_maps/launch_simulation.py
```

Terminal 2 phải được chạy **sau khi cửa sổ Gazebo đã mở và model `x500_depth_0`
đã xuất hiện**. Launcher sẽ tự đọc `gz topic -l`, tìm topic runtime (kể cả tên
được scope theo model/link/sensor), rồi remap về các topic ROS chuẩn. Terminal này
phải in được các topic camera/odometry và các dòng bridge tương tự:

```text
/model/x500_depth_0/odometry_with_covariance
/camera
/depth_camera
/depth_camera/points
```

Launcher tạo một tiến trình `ros_gz_bridge` riêng cho RGB, depth và point cloud;
điều này giúp Gazebo đăng ký subscriber cho từng sensor. Sau khi RViz2 đã mở,
xác nhận phía Gazebo đã có subscriber:

```bash
gz topic -i -t /camera
gz topic -i -t /depth_camera
gz topic -i -t /depth_camera/points
```

Mỗi lệnh phải có cả `Publishers` và ít nhất một `Subscribers` từ
`ros_gz_bridge`. Nếu chỉ có `Publishers` và hiện `No subscribers`, hãy dừng
launcher/RViz2 cũ, đóng Gazebo/PX4 rồi khởi động lại đúng thứ tự; bridge của
phiên cũ không tự nhận phần code mới.

Nếu Gazebo tạo topic dạng dài, ví dụ
`/model/x500_depth_0/link/camera_link/sensor/IMX214/image`, đó vẫn là trạng thái
bình thường; launcher sẽ tự bridge topic này về `/camera`.

Nếu không thấy các dòng trên, hãy dừng toàn bộ mô phỏng và chạy lại Terminal 1;
không mở nhiều instance Gazebo/PX4 cùng lúc.

---

### Terminal 3: Khởi chạy QGroundControl

```bash
~/QGroundControl.AppImage
```

---

### Terminal 4: Nhận diện ArUco trên camera Gazebo

H-Pad dùng marker ID `42` của dictionary `DICT_6X6_50`. Mặt H-Pad trong Gazebo
là `0.90 x 0.90 m`. File `marker42.png` có thêm dải tiêu đề ở phía trên, nên
vùng mã đen thực tế chiếm khoảng `0.812 m`; node dùng giá trị này cho PnP để
tọa độ không bị sai tỷ lệ. Nếu bản in thực tế đã crop tiêu đề và vùng mã đen
đúng 90 cm, chạy thêm `--ros-args -p marker_size_m:=0.90`. Node này đọc RGB,
`CameraInfo` và depth, sau đó xuất pose marker cùng ảnh đã vẽ kết quả:

```bash
source /opt/ros/humble/setup.bash
cd /home/duy/VDT_project
python3 simulation_maps/aruco_sim_node.py
```

Các topic cần kiểm tra:

```bash
ros2 topic echo --once /hpad/detected
ros2 topic echo --once /hpad/pose
ros2 topic echo --once /hpad/position_camera
ros2 topic hz /hpad/annotated
```

Trong RViz2, display `ArUco Annotated` dùng topic `/hpad/annotated`. Hai topic
`/hpad/pose` và `/hpad/position_camera` trả về tọa độ PnP theo
`camera_optical_frame`: `x` sang phải, `y` hướng xuống, `z` hướng trước camera.
Node cũng in dòng `[DETECTION] ... camera_optical xyz=(...) m`. Launcher đã
phát TF `world -> base_link -> camera_link -> camera_optical_frame`, nên node
tracking có thể đổi vị trí này sang frame `world`.

### Terminal 5: Điều khiển H-Pad bằng bàn phím

Node điều khiển gọi service `set_pose` của Gazebo, giới hạn vận tốc tối đa
`2 m/s` và publish ground truth tại `/hpad/ground_truth`:

```bash
source /opt/ros/humble/setup.bash
cd /home/duy/VDT_project
python3 simulation_maps/hpad_keyboard_controller.py --speed 2.0
```

Phím điều khiển:

```text
W / S       di chuyển theo +X / -X
A / D       di chuyển theo +Y / -Y
SPACE       dừng H-Pad
R           đưa H-Pad về (4.0, 0.0)
+ / -       tăng/giảm tốc độ
X           thoát
```

Nếu terminal báo service chưa sẵn sàng, controller sẽ tự chuyển sang gọi native
Gazebo `/world/obstacle_avoidance/set_pose`; vì vậy vẫn điều khiển được dù ROS
service bridge chưa đăng ký. Có thể kiểm tra cả hai đường:

```bash
ros2 service list | grep '/world/obstacle_avoidance/set_pose'
```

Có thể chạy bridge thủ công rồi dùng controller không tạo bridge thứ hai:

```bash
ros2 run ros_gz_bridge parameter_bridge \
  '/world/obstacle_avoidance/set_pose@ros_gz_interfaces/srv/SetEntityPose'
python3 simulation_maps/hpad_keyboard_controller.py --no-bridge --speed 2.0
```

Nếu muốn kiểm tra trực tiếp native Gazebo service:

```bash
gz service -i -s /world/obstacle_avoidance/set_pose
```

World phải có plugin `gz::sim::systems::UserCommands`, và world hiện tại đã
được cấu hình plugin này.

### Kiểm tra gimbal pitch

Do `x500_depth/model.sdf` đã đổi `CameraJoint` sang revolute và có
`JointPositionController`, cần khởi động lại PX4/Gazebo sau khi cập nhật model.
Có thể điều khiển qua ROS topic:

```bash
ros2 topic pub --once /model/x500_depth_0/command/gimbal_pitch \
  std_msgs/msg/Float64 "{data: -0.8}"
```

Góc được giới hạn trong khoảng `-90°` đến `+15°`. Nếu cần tách bridge để chẩn
đoán, dùng lệnh Gazebo tương đương:

```bash
gz topic -t /model/x500_depth_0/command/gimbal_pitch \
  -m gz.msgs.Double -p 'data:-0.8'
```

Sau khi gửi, kiểm tra `gz topic -i -t /model/x500_depth_0/command/gimbal_pitch`;
phải có subscriber `JointPositionController`.

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
ros2 topic hz /odom
ros2 topic hz /camera
ros2 topic hz /depth_camera
ros2 topic hz /depth_camera/points
ros2 topic echo --once /hpad/marker --field pose
```

Kết quả tối thiểu cần có:

| Kiểm tra | Kết quả đúng |
|---|---|
| Odometry | Có message liên tục trên `/odom`, tần số > 10 Hz |
| RGB camera | Có message liên tục trên `/camera` |
| Depth image | Có message liên tục trên `/depth_camera` |
| Depth point cloud | Có message liên tục trên `/depth_camera/points` |
| H-pad | `/hpad/marker` trả về pose gần `(4.0, 0.0, 0.023)` |
| Drone RViz2 | `DroneMarker` di chuyển cùng vị trí drone trong Gazebo |

Nếu Gazebo vẫn báo `No subscribers`, có thể kiểm tra riêng bridge sau khi đã
dừng các bridge cũ để tránh có nhiều publisher trùng topic:

```bash
ros2 run ros_gz_bridge parameter_bridge '/camera@sensor_msgs/msg/Image@gz.msgs.Image'
ros2 run ros_gz_bridge parameter_bridge '/depth_camera@sensor_msgs/msg/Image@gz.msgs.Image'
ros2 run ros_gz_bridge parameter_bridge '/depth_camera/points@sensor_msgs/msg/PointCloud2@gz.msgs.PointCloudPacked'
```

Trong terminal khác, kiểm tra lại `gz topic -i -t ...` và `ros2 topic hz ...`.

Nếu bridge odometry chết ngay, terminal launch sẽ báo `Odom ros_gz_bridge exited
immediately`. Khi đó kiểm tra đúng model name bằng:

```bash
gz model --list
gz topic -l | grep -E 'odometry|camera|depth'
```

Model mặc định của lệnh `gz_x500_depth` phải là `x500_depth_0`. Nếu tên khác,
sửa hằng `MODEL_NAME` ở đầu `launch_simulation.py` cho khớp rồi khởi động lại.
