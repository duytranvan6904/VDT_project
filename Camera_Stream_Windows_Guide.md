# XEM CAMERA TỪ PI 5 TRÊN LAPTOP (WINDOWS)

Môi trường: Ubuntu 24.04, ROS 2 Jazzy trên Raspberry Pi 5.
Laptop và Pi 5 nối chung mạng Tailscale. Laptop mở trình duyệt xem luồng ảnh trực tiếp từ Pi 5, không qua PX4.

---

## 1. Cài đặt trên Pi 5

```bash
sudo apt update
sudo apt install -y ros-jazzy-web-video-server
```

Nếu apt không có gói cho Jazzy thì clone `https://github.com/RobotWebTools/web_video_server.git` vào `ros2_ws/src` rồi `colcon build`.

## 2. Chạy trên Pi 5

```bash
source /opt/ros/jazzy/setup.bash
ros2 run web_video_server web_video_server --ros-args -p port:=8080
```

Có thể đưa node này vào `vdt_full.launch.py` để tự chạy khi khởi động hệ thống.

## 3. Lấy IP Tailscale của Pi 5

```bash
tailscale ip -4
```

Kiểm tra laptop nối trực tiếp tới Pi 5 (không qua relay):

```bash
tailscale ping pi5vdt
```

Kết quả `via DERP` nghĩa là đang đi vòng qua relay, độ trễ sẽ cao hơn. Kết quả `direct` là đạt.

## 4. Xem trên laptop

Mở Chrome hoặc Edge, thay `<IP_TS>` bằng IP Tailscale của Pi 5.

Danh sách topic đang phát:

```text
http://<IP_TS>:8080/
```

Ảnh có khung nhận diện marker:

```text
http://<IP_TS>:8080/stream?topic=/hpad/annotated&type=mjpeg&quality=60
```

Ảnh hồng ngoại IR1:

```text
http://<IP_TS>:8080/stream?topic=/camera/infra1/image_rect_raw&type=mjpeg&quality=60
```

Depth (đã chuyển mono8 bởi `depth_to_image_node`):

```text
http://<IP_TS>:8080/stream?topic=/depth_camera/image_mono&type=mjpeg&quality=60
```

Mỗi luồng mở một tab riêng để xem cùng lúc.

## 5. Kiểm tra khi lỗi

Topic có dữ liệu chưa:

```bash
ros2 topic hz /hpad/annotated
```

Cổng 8080 có đang nghe không:

```bash
ss -ltn | grep 8080
```

Nếu hình giật, giảm `quality` hoặc thêm `&width=480&height=360` vào URL. Pi 5 mã hóa JPEG bằng CPU, mỗi luồng mở thêm tốn thêm tải.