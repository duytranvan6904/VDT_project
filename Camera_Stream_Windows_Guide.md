# HƯỚNG DẪN XEM STREAM CAMERA TỪ PI 5 TRÊN MÁY TÍNH WINDOWS (GCS)

Tài liệu này hướng dẫn cài đặt và thiết lập phần mềm theo dõi hình ảnh camera (IR, RGB, Depth, Annotated Marker) chạy **độc lập trên máy tính Windows**, kết nối không dây (WiFi) hoặc cáp mạng (Ethernet) tới Raspberry Pi 5.

---

## GIẢI PHÁP 1: FOXGLOVE STUDIO (KHUYẾN NGHỊ SỐ 1 - CHUẨN CÔNG NGHIỆP ROBOTICS)

Foxglove Studio là giải pháp trực quan hóa mạnh mẽ nhất hiện nay cho ROS 2. Ứng dụng chạy trên Windows, kết nối tới Pi 5 qua giao thức WebSocket siêu nhẹ, không làm nóng hay tốn tài nguyên Pi 5.

### 1. Trên máy tính Windows:
* **Cách 1 (Không cần cài đặt)**: Mở trình duyệt Chrome hoặc Edge, truy cập:  
  👉 **https://studio.foxglove.dev**
* **Cách 2 (Cài Desktop App Windows)**:  
  Tải bản cài đặt Windows (`.exe`) từ trang chủ: [https://foxglove.dev/download](https://foxglove.dev/download) rồi cài đặt bình thường.

### 2. Trên Raspberry Pi 5 (Chạy lệnh qua SSH hoặc Terminal):
Cài đặt package `foxglove_bridge` chuẩn của ROS 2 Humble (viết bằng C++, chiếm chưa tới 2% CPU Pi 5):
```bash
sudo apt update
sudo apt install -y ros-humble-foxglove-bridge
```

Khởi chạy bridge:
```bash
source /opt/ros/humble/setup.bash
ros2 launch foxglove_bridge foxglove_bridge_launch.xml port:=8765
```

*(Mẹo: Có thể add node này vào file `vdt_full.launch.py` để tự động chạy cùng hệ thống).*

### 3. Kết nối và cấu hình Layout trên Windows:
1. Mở Foxglove Studio trên Windows.
2. Chọn **Open Connection** -> Chọn giao thức **Foxglove WebSocket**.
3. Điền địa chỉ IP của Pi 5:
   ```text
   ws://<IP_CỦA_PI_5>:8765
   ```
   *(Ví dụ: `ws://192.168.1.150:8765`)* -> Nhấn **Open**.
4. Bấm dấu `+` để thêm panel:
   - Chọn panel **Image**:
     - Chọn topic `/hpad/annotated`: Xem ảnh camera kèm khung nhận diện Marker ArUco và tâm bãi đáp.
     - Chọn topic `/camera/infra1/image_rect_raw`: Xem ảnh thô hồng ngoại từ RealSense.
     - Chọn topic `/depth_camera`: Foxglove tự động tô màu (Color Map) theo độ sâu từ gần đến xa (màu tím/xanh -> vàng/đỏ) cực kỳ trực quan.
   - Chọn panel **3D**: Xem mô hình drone, trục tọa độ TF, và vị trí bãi đáp trong không gian.
   - Chọn panel **Plot**: Xem đồ thị độ cao thời gian thực `/odom/pose/pose/position/z`.

---

## GIẢI PHÁP 2: WEB VIDEO SERVER (XEM TRỰC TIẾP TRÊN TRÌNH DUYỆT WEB WINDOWS)

Nếu máy Windows không muốn cài thêm phần mềm gì, chỉ cần mở trình duyệt web Chrome/Edge để xem stream mượt 30fps.

### 1. Trên Raspberry Pi 5:
Cài đặt node `web_video_server`:
```bash
sudo apt update
sudo apt install -y ros-humble-web-video-server
```

Chạy node:
```bash
source /opt/ros/humble/setup.bash
ros2 run web_video_server web_video_server --ros-args -p port:=8080
```

### 2. Trên máy tính Windows:
Mở trình duyệt (Chrome, Edge, Firefox), gõ trực tiếp URL:
* Xem trang danh sách topic camera đang phát:
  ```text
  http://<IP_CỦA_PI_5>:8080/
  ```
* Xem trực tiếp luồng ảnh đã nhận diện Marker:
  ```text
  http://<IP_CỦA_PI_5>:8080/stream?topic=/hpad/annotated
  ```
* Xem trực tiếp luồng ảnh hồng ngoại IR1:
  ```text
  http://<IP_CỦA_PI_5>:8080/stream?topic=/camera/infra1/image_rect_raw
  ```

---

## GIẢI PHÁP 3: TÍCH HỢP VIDEO VÀO QGROUNDCONTROL (NẾU DÙNG QGC TRÊN WINDOWS)

Nếu phi công đang dùng QGroundControl trên Windows để giám sát chuyến bay:
* Trên Pi 5, dùng GStreamer đẩy luồng H.264 qua UDP port 5600:
  ```bash
  gst-launch-1.0 v4l2src device=/dev/video0 ! video/x-raw,width=640,height=480,framerate=30/1 ! videoconvert ! x264enc tune=zerolatency bitrate=1000 speed-preset=ultrafast ! rtph264pay ! udpsink host=<IP_MAY_WINDOWS> port=5600
  ```
* Trên QGC Windows: Vào **Application Settings** -> **General** -> **Video** -> Chọn **Video Source: UDP h.264 Video Stream**, Port `5600`. Video sẽ hiện ở góc dưới màn hình bay của QGC.
