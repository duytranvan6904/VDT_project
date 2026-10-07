# VDT Messages — Guide

## 1. Mục đích và Phụ thuộc

Package `vdt_msgs` đóng vai trò là **Message Hub (Trung tâm định nghĩa giao diện dữ liệu)** độc lập cho toàn bộ workspace `VDT_project`.

### Mục đích kiến trúc:
- Gom toàn bộ 8 custom messages của hệ thống vào một package duy nhất.
- Triệt tiêu hoàn toàn **phụ thuộc vòng (Circular Dependencies)** giữa các package nghiệp vụ (như giữa `fsm_state_machine`, `input_state_cache`, `offboard_manager`).
- Đưa đồ thị phụ thuộc của workspace về dạng **Directed Acyclic Graph (DAG)** chuẩn hóa của ROS 2.
- Giúp các package như `px4_state_bridge`, `vision_interface_bridge`, `apf_planner` không bị phụ thuộc chéo vào logic điều khiển của package khác.

### Package phụ thuộc khi build:
- Package chuẩn ROS 2: `ament_cmake`, `rosidl_default_generators`, `std_msgs`.
- Runtime: `rosidl_default_runtime`.
- Không phụ thuộc vào bất kỳ package nội bộ nào khác trong workspace.

---

## 2. Danh mục và Ý nghĩa các Messages

Package định nghĩa 8 bản tin dữ liệu chia làm 4 nhóm chức năng chính:

```text
                                  vdt_msgs
                                     │
    ┌────────────────┬───────────────┴───────────────┬────────────────┐
    ▼                ▼                               ▼                ▼
[Nhóm Cảm biến]  [Nhóm State Cache]            [Nhóm Dẫn đường]   [Nhóm RC]
- AltEstimate    - InputSnapshot               - PlannerOutput    - RcFsmInput
- VisionMarker   - TimeoutFlags                - OffboardStatus   - RcChannelsRaw
```

---

### A. Nhóm Cảm biến & Cầu nối (Sensors & Bridges)

#### 1. `AltEstimate.msg`
Được phát bởi `px4_state_bridge` (topic `alt_estimator/state`, chu kỳ 20 Hz) dựa trên thông tin odometry và tiếp đất từ PX4.

| Trường | Kiểu | Đơn vị | Ý nghĩa |
|---|---|---|---|
| `altitude` | `float32` | m | Độ cao tuyệt đối trong frame `world` (ENU, z hướng lên), `NaN` nếu odometry không hợp lệ |
| `touchdown_flag` | `bool` | - | `true` khi PX4 báo đã tiếp đất (`vehicle_land_detected.landed`) |

#### 2. `VisionMarker.msg`
Được chuyển đổi và phát bởi `vision_interface_bridge` (topic `vision/marker`) từ kết quả phát hiện của `aruco_detector`.

| Trường | Kiểu | Đơn vị | Ý nghĩa |
|---|---|---|---|
| `marker_visible` | `bool` | - | `true` khi nhìn thấy H-Pad ArUco marker hợp lệ |
| `pixel_align_error` | `float32` | norm [-1, 1] | Sai số khoảng cách tâm marker tới tâm ảnh (chuẩn hóa theo nửa kích thước ảnh), `NaN` khi `marker_visible = false` |

---

### B. Nhóm Tập hợp Trạng thái (State Aggregator & Watchdog)

#### 3. `InputSnapshot.msg`
Bản tin quan trọng nhất hệ thống, do `input_state_cache` (topic `input_cache/snapshot`, chu kỳ 20 Hz) tổng hợp và phát nguyên tử cho `fsm_state_machine` và `gimbal_control`.

| Trường | Kiểu | Đơn vị | Ý nghĩa |
|---|---|---|---|
| `valid` | `bool` | - | Trạng thái UAV hợp lệ (`odom_valid && alt_valid`), không phụ thuộc việc có thấy target hay không |
| `marker_detected` | `bool` | - | `true` khi marker đang được nhìn thấy và dữ liệu vision còn tươi (`fresh`) |
| `align_error` | `float32` | m | Khoảng cách ngang giữa UAV và H-Pad ($d_{horiz}$), `NaN` nếu thiếu target hoặc odometry |
| `altitude` | `float32` | m | Độ cao hiện tại của UAV từ `alt_estimator`, `NaN` nếu sensor mất kết nối |
| `delta_h` | `float32` | m | Độ cao tương đối: UAV cao hơn H-Pad là dương ($z_{uav} - z_{target}$), `NaN` nếu không có target |
| `d_horiz` | `float32` | m | Khoảng cách mặt phẳng ngang tới H-Pad ($\sqrt{\Delta x^2 + \Delta y^2}$), `NaN` nếu không có target |
| `yaw_rate` | `float32` | rad/s | Tốc độ góc quanh trục yaw của UAV (dùng để chờ ổn định trước khi vào FOLLOW) |
| `touchdown` | `bool` | - | Đã chạm đất an toàn và dữ liệu độ cao còn tươi |
| `planner_timeout` | `bool` | - | `true` khi mất luồng `planner/velocity_setpoint` quá thời gian cho phép |

#### 4. `TimeoutFlags.msg`
Cung cấp chi tiết các cờ quá hạn dữ liệu từ `input_state_cache` (topic `input_cache/timeout_flags`).

| Trường | Kiểu | Ý nghĩa |
|---|---|---|
| `ekf_timeout` | `bool` | `true` khi mất EKF, dữ liệu stale hoặc tracking mode là `EXPIRED` |
| `vision_timeout` | `bool` | `true` khi mất topic `vision/marker` quá thời hạn cấu hình |
| `alt_timeout` | `bool` | `true` khi mất topic `alt_estimator/state` quá thời hạn cấu hình |
| `planner_timeout` | `bool` | `true` khi mất topic `planner/velocity_setpoint` quá thời hạn cấu hình |

---

### C. Nhóm Dẫn đường & Chấp hành (Guidance & Actuation)

#### 5. `PlannerOutput.msg`
Do `planner_merge_node` (package `apf_planner`) phát hành trên topic `planner/velocity_setpoint` để điều khiển bay ở các pha active.

| Trường | Kiểu | Đơn vị / Hệ trục | Ý nghĩa |
|---|---|---|---|
| `vx` | `float32` | m/s (ENU) | Vận tốc tuyến tính hướng Đông |
| `vy` | `float32` | m/s (ENU) | Vận tốc tuyến tính hướng Bắc |
| `vz` | `float32` | m/s (ENU) | Vận tốc tuyến tính hướng lên |
| `yaw` | `float32` | rad (ENU) | Góc heading mong muốn (hướng Đông là 0, ngược chiều kim đồng hồ là dương). `NaN` nghĩa là giữ nguyên yaw hiện tại |

#### 6. `OffboardStatus.msg`
Do `offboard_manager` phát hành trên topic `offboard/status` phản ánh trạng thái điều khiển cấp thấp với PX4.

| Trường | Kiểu | Đơn vị | Ý nghĩa |
|---|---|---|---|
| `offboard_active` | `bool` | - | `true` khi PX4 đã vào thành công chế độ OFFBOARD và armed |
| `heartbeat_age_sec` | `float32` | s | Thời gian trôi qua kể từ lần gửi heartbeat gần nhất tới PX4 |

---

### D. Nhóm Điều khiển bằng tay & An toàn (RC & Safety)

#### 7. `RcFsmInput.msg`
Do `rc_parser` trích xuất từ sóng SBUS và phát trên topic `rc/fsm_input` cho FSM.

| Trường | Kiểu | Ý nghĩa |
|---|---|---|
| `land_switch` | `bool` | `true` khi công tắc hạ cánh (kênh Land) bật gạt cho phép chuyển sang APPROACH/LAND |
| `kill_switch` | `bool` | `true` khi công tắc khẩn cấp (kênh Kill) được kích hoạt |

#### 8. `RcChannelsRaw.msg`
Do `rc_parser` phát trên topic `rc/channels_raw` chứa toàn bộ 16 kênh RC thô (microseconds).

| Trường | Kiểu | Ý nghĩa |
|---|---|---|
| `ch` | `int16[16]` | Độ rộng xung 16 kênh SBUS (khoảng 800 đến 2200 $\mu s$) |
| `valid` | `bool` | Khung truyền SBUS hợp lệ, không bị lỗi frame hoặc timeout UART |
| `failsafe` | `bool` | Cờ failsafe phần cứng do bộ thu RC phát hiện mất sóng |

---

## 3. Ma trận Sản sinh & Tiêu thụ (Producer / Consumer Matrix)

| Message | Gói sản sinh (Producer) | Topic mặc định | Gói tiêu thụ chính (Consumers) |
|---|---|---|---|
| `AltEstimate` | `px4_state_bridge` | `alt_estimator/state` | `input_state_cache` |
| `VisionMarker` | `vision_interface_bridge` | `vision/marker` | `input_state_cache` |
| `InputSnapshot` | `input_state_cache` | `input_cache/snapshot` | `fsm_state_machine`, `gimbal_control` |
| `TimeoutFlags` | `input_state_cache` | `input_cache/timeout_flags` | `offboard_manager`, `fsm_state_machine` |
| `PlannerOutput` | `apf_planner` (`planner_merge_node`) | `planner/velocity_setpoint` | `offboard_manager`, `input_state_cache` |
| `OffboardStatus` | `offboard_manager` | `offboard/status` | `offboard_safety_monitor` |
| `RcFsmInput` | `rc_parser` | `rc/fsm_input` | `fsm_state_machine` |
| `RcChannelsRaw` | `rc_parser` | `rc/channels_raw` | `kill_switch` |

---

## 4. Hướng dẫn sử dụng trong Code

### Trong C++ Node:
Trong file header hoặc source:
```cpp
#include "vdt_msgs/msg/input_snapshot.hpp"
#include "vdt_msgs/msg/alt_estimate.hpp"
#include "vdt_msgs/msg/planner_output.hpp"

// Khai báo Subscriber:
auto sub = this->create_subscription<vdt_msgs::msg::InputSnapshot>(
  "input_cache/snapshot", 10,
  std::bind(&MyNode::on_snapshot, this, std::placeholders::_1));

// Khai báo Publisher:
auto pub = this->create_publisher<vdt_msgs::msg::PlannerOutput>(
  "planner/velocity_setpoint", 10);
```

Trong `CMakeLists.txt`:
```cmake
find_package(vdt_msgs REQUIRED)
ament_target_dependencies(my_node rclcpp vdt_msgs)
```

Trong `package.xml`:
```xml
<depend>vdt_msgs</depend>
```

### Trong Python Node:
```python
from vdt_msgs.msg import InputSnapshot, PlannerOutput, RcFsmInput

msg = PlannerOutput()
msg.vx = 0.5
msg.vy = 0.0
msg.vz = -0.2
msg.yaw = float('nan')
```

---

## 5. Hướng dẫn Biên dịch và Kiểm tra

Build riêng package `vdt_msgs`:
```bash
cd ros2_ws
colcon build --packages-select vdt_msgs
source install/setup.bash
```

Kiểm tra các message đã được đăng ký chuẩn xác:
```bash
ros2 interface show vdt_msgs/msg/InputSnapshot
ros2 interface show vdt_msgs/msg/PlannerOutput
ros2 interface show vdt_msgs/msg/VisionMarker
```
