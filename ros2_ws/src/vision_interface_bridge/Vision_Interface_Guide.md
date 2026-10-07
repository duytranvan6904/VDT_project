# Vision Interface Bridge — Guide

## 1. Package cần cài đặt ngoài

- Package chuẩn ROS 2: `rclcpp`, `std_msgs`, `sensor_msgs`, `vision_msgs`.
- Phụ thuộc `vdt_msgs` để lấy định nghĩa `VisionMarker`.

## 2. Nguyên lý hoạt động

```text
/hpad/detected    --\
/hpad/bbox        --\
/camera_info      --> vision_interface_bridge
fsm/state         --/        │
                             ├─► vision/marker (vdt_msgs/VisionMarker)
                             │
                             └─► /mission/phase (String, 10Hz)
```

Node chuyển các topic của vision sang giao diện mà `input_state_cache` và các node vision cần.

### `vision/marker`

| Sự kiện đầu vào | Bản tin phát ra |
|---|---|
| `/hpad/bbox` đến | `marker_visible = true`, `pixel_align_error` = sai số căn chỉnh chuẩn hóa |
| `/hpad/detected` với `data = false` | `marker_visible = false`, `pixel_align_error = NaN` |
| `/hpad/detected` với `data = true` | Không phát gì, chờ `bbox` của cùng khung hình |

Node chỉ phát `marker_visible = true` khi có `bbox`. Lý do: `bbox` chỉ được publish khi phát hiện thành công, và có thể đến sau `detected` trong cùng khung hình. Nếu node phát ngay khi thấy `detected = true` thì sẽ dùng `bbox` của khung hình trước.

Node phát hiện cần đảm bảo:

- `/hpad/detected` được publish ở mọi khung hình.
- `/hpad/bbox` chỉ được publish khi phát hiện thành công.

Nếu node phát hiện dừng hẳn thì `vision/marker` không còn được publish, và `input_state_cache` sẽ báo `vision_timeout`.

### Sai số căn chỉnh `pixel_align_error`

Khoảng cách từ tâm `bbox` đến tâm ảnh, chuẩn hóa theo nửa kích thước ảnh theo từng trục:

$$e = \sqrt{\left(\frac{u - w/2}{w/2}\right)^2 + \left(\frac{v - h/2}{h/2}\right)^2}$$

- Tâm `bbox` ở giữa ảnh: `e = 0`.
- Tâm `bbox` ở giữa mép trái hoặc phải: `e = 1`.
- Tâm `bbox` ở góc ảnh: `e` xấp xỉ 1.414.
- Kích thước ảnh `w`, `h` lấy từ `camera_info`; trước khi nhận được thì dùng `image_width`, `image_height`.
- `camera_info` phải cùng camera với ảnh phát hiện marker (IR1), vì `bbox` tính theo pixel của ảnh IR1.

FSM hiện không dùng giá trị này (`align_error` của FSM là khoảng cách ngang UAV tới H-Pad tính trong `input_state_cache`). Giá trị này chỉ để tham khảo và kiểm tra tính hữu hạn.

### `/mission/phase`

Đổi `fsm/state` (`UInt8`) sang chuỗi mà EKF, APF và IBVS của vision hiểu:

| `fsm/state` | `/mission/phase` |
|---|---|
| 0 | `SEARCH` |
| 1 | `FOLLOW` |
| 2 | `APPROACH` |
| 3 | `LAND` |
| 4 (COMPLETE) hoặc giá trị khác | `IDLE` |
| Chưa nhận hoặc cũ hơn `fsm_state_timeout_sec` | `IDLE` |

Node publish ngay khi nhận `fsm/state` mới và lặp lại ở 10 Hz để node vision khởi động muộn vẫn nhận được phase hiện tại. Khi phase là `IDLE`, APF không xuất lệnh vận tốc.

Consumer của `/mission/phase`: `ekf_node`, `planner_node`, `planner_merge_node`, `ibvs_controller`.

## 3. Cách chạy

```bash
ros2 launch vision_interface_bridge vision_interface_bridge.launch.py
```

Hoặc chạy trực tiếp:

```bash
ros2 run vision_interface_bridge vision_interface_bridge_node
```

Chạy kèm tham số `camera_info_topic` trỏ tới `CameraInfo` của IR1 (cùng camera với ảnh phát hiện marker):

```bash
ros2 run vision_interface_bridge vision_interface_bridge_node --ros-args \
  -p camera_info_topic:=/camera/infra1/camera_info
```

Chạy kèm tham số tùy chỉnh:

```bash
ros2 run vision_interface_bridge vision_interface_bridge_node --ros-args \
  -p image_width:=640.0 \
  -p image_height:=480.0 \
  -p fsm_state_timeout_sec:=1.0
```

Danh sách tham số:

| Tên | Mặc định | Ý nghĩa |
|---|---|---|
| `detected_topic` | `/hpad/detected` | Topic cờ phát hiện (`std_msgs/Bool`) |
| `bbox_topic` | `/hpad/bbox` | Topic `vision_msgs/BoundingBox2D` |
| `camera_info_topic` | `/camera_info` | Topic `sensor_msgs/CameraInfo`, dùng để lấy kích thước ảnh |
| `marker_topic` | `vision/marker` | Topic đầu ra `VisionMarker` |
| `phase_topic` | `/mission/phase` | Topic đầu ra phase |
| `image_width` | 640.0 | Chiều rộng ảnh dự phòng khi chưa có `camera_info` |
| `image_height` | 480.0 | Chiều cao ảnh dự phòng khi chưa có `camera_info` |
| `fsm_state_timeout_sec` | 1.0 | `fsm/state` cũ hơn mức này thì phase là `IDLE` |

Danh sách topic:

| Topic | Hướng | Kiểu | QoS |
|---|---|---|---|
| `/hpad/detected` | vào | `std_msgs/Bool` | reliable |
| `/hpad/bbox` | vào | `vision_msgs/BoundingBox2D` | reliable |
| `/camera_info` | vào | `sensor_msgs/CameraInfo` | best effort |
| `fsm/state` | vào | `std_msgs/UInt8` | reliable |
| `vision/marker` | ra | `vdt_msgs/VisionMarker` | reliable |
| `/mission/phase` | ra | `std_msgs/String` | reliable |

## 4. Cách debug

Kiểm tra `vision/marker`:

```bash
ros2 topic echo /vision/marker
ros2 topic hz /vision/marker
```

Kiểm tra phase:

```bash
ros2 topic echo /mission/phase
```

Kiểm tra đầu vào:

```bash
ros2 topic echo /hpad/detected
ros2 topic echo /hpad/bbox
ros2 topic echo /fsm/state
```

Giả lập `fsm/state` để kiểm tra phase:

```bash
ros2 topic pub /fsm/state std_msgs/msg/UInt8 "{data: 1}" -r 10
```

Phase phải là `FOLLOW`. Dừng lệnh trên, sau hơn `fsm_state_timeout_sec` phase phải về `IDLE`.

Giả lập một `bbox` ở tâm ảnh 640x480:

```bash
ros2 topic pub /hpad/bbox vision_msgs/msg/BoundingBox2D \
  "{center: {position: {x: 320.0, y: 240.0}, theta: 0.0}, size_x: 80.0, size_y: 80.0}" -r 10
```

`vision/marker` phải có `marker_visible: true` và `pixel_align_error` xấp xỉ 0.

Giả lập mất marker:

```bash
ros2 topic pub /hpad/detected std_msgs/msg/Bool "{data: false}" -r 10
```

`vision/marker` phải có `marker_visible: false` và `pixel_align_error: .nan`.

### Chạy unit test logic tự động:

```bash
cd ros2_ws
colcon test --packages-select vision_interface_bridge --ctest-args -R bridge_logic_test
```

## 5. Xử lý sự cố thường gặp

1. **Không có `vision/marker`:**
   - Kiểm tra node phát hiện có đang publish `/hpad/detected` và `/hpad/bbox`.
   - Kiểm tra tên topic có khớp với `detected_topic`, `bbox_topic`.
2. **`marker_visible` luôn là `false` dù marker trong khung hình:**
   - Kiểm tra `/hpad/bbox` có được publish khi phát hiện (node chỉ phát `true` khi nhận `bbox`).
3. **`pixel_align_error` là `NaN` khi `marker_visible = true`:**
   - `image_width` hoặc `image_height` không hợp lệ (không dương) hoặc tâm `bbox` không hữu hạn. Kiểm tra `camera_info` và tham số dự phòng.
4. **`pixel_align_error` sai thang:**
   - Kích thước ảnh dự phòng khác độ phân giải thật khi chưa nhận `camera_info`. Kiểm tra `camera_info_topic` có đúng topic `CameraInfo` của IR1 (không dùng của camera màu, vì độ phân giải có thể khác ảnh phát hiện marker).
5. **`/mission/phase` luôn là `IDLE`:**
   - `fsm/state` chưa được publish hoặc cũ hơn `fsm_state_timeout_sec`. Kiểm tra `fsm_node` đang chạy và `ros2 topic hz /fsm/state`.
   - FSM đang ở COMPLETE (state 4), phase `IDLE` là đúng.
6. **APF không xuất lệnh vận tốc:**
   - APF chỉ chạy khi phase là `FOLLOW` hoặc `APPROACH` và `ekf/tracking_mode` là `TRACKING` hoặc `PREDICTING`. Kiểm tra cả hai.
7. **`input_state_cache` báo `vision_timeout` liên tục:**
   - `vision/marker` không được publish. Node phát hiện dừng, hoặc `use_sim_time` không đồng nhất giữa các node.