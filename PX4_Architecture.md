# Kiến trúc Flight Stack PX4

Tổng hợp từ mã nguồn PX4/PX4-Autopilot và docs.px4.io (nhánh main). Cấu trúc module/tham số có thể đổi giữa các phiên bản — cần bản cụ thể (v1.14, v1.15, v1.16...) thì đối chiếu đúng tài liệu phiên bản đó.

---

## 1. Nguồn tham khảo

| Chủ đề | Nguồn |
|---|---|
| Mã nguồn chính (BSD 3-Clause) | https://github.com/PX4/PX4-Autopilot |
| Kiến trúc tổng quan | https://docs.px4.io/main/en/concept/architecture |
| Kiến trúc hệ thống (FC + companion computer) | https://docs.px4.io/main/en/concept/px4_systems_architecture |
| uORB messaging | https://docs.px4.io/main/en/middleware/uorb · https://px4.io/px4-uorb-explained-part-1/ |
| Sơ đồ publish/subscribe | https://docs.px4.io/main/en/middleware/uorb_graph |
| EKF2 / ECL | https://docs.px4.io/main/en/advanced_config/tuning_the_ecl_ekf · `src/lib/ecl` |
| Control allocation | https://docs.px4.io/main/en/concept/control_allocation |
| Sơ đồ bộ điều khiển tầng | https://docs.px4.io/main/en/flight_stack/controller_diagrams |
| Navigator / mission | https://docs.px4.io/main/en/modules/modules_controller · `src/modules/navigator` |
| Logging / ULog | https://docs.px4.io/main/en/dev_log/logging · https://docs.px4.io/main/en/dev_log/ulog_file_format |
| uXRCE-DDS | https://docs.px4.io/main/en/middleware/uxrce_dds |
| Dataman | `src/modules/dataman` · https://bkueng.gitbooks.io/px4-devguide/content/en/middleware/modules_system.html |
| Tra cứu theo mã nguồn (không chính thức) | https://deepwiki.com/PX4/PX4-Autopilot |

Ưu tiên docs.px4.io và mã nguồn GitHub; nguồn khác chỉ để đối chiếu.

---

## 2. Kiến trúc tổng quát

PX4 gồm hai lớp: **flight stack** (ước lượng trạng thái, điều khiển bay) và **middleware** (lớp robot tổng quát, giao tiếp nội bộ/ngoại vi, tích hợp phần cứng). Mọi airframe (đa cánh quạt, cánh cố định, VTOL, tàu, rover, tàu ngầm) dùng chung codebase. Thiết kế reactive: thành phần độc lập, hoán đổi được, giao tiếp bằng message bất đồng bộ qua uORB.

NuttX là RTOS chính trên flight controller. Mỗi module chạy như task riêng, chia sẻ không gian địa chỉ chung. SITL cho phép chạy toàn bộ flight code trên máy tính, không cần phần cứng.

Hệ thống vật lý điển hình: flight controller (IMU/compass/baro tích hợp) — ESC/động cơ qua PWM/DroneCAN — cảm biến ngoại vi (GPS, la bàn, range sensor, optical flow) qua I2C/SPI/CAN/UART. Hệ nâng cao có thêm companion computer chạy Linux, nối flight controller qua serial/IP tốc độ cao (thường MAVLink), xử lý thị giác máy tính và định tuyến giao tiếp GCS/cloud.

---

## 3. Module chức năng chính

`src/modules` chứa task độc lập, `src/lib` chứa thư viện dùng chung.

| Module | Vai trò |
|---|---|
| sensors | Thu thập, hiệu chỉnh, tổng hợp IMU/baro/mag/GPS/airspeed thành topic uORB |
| ekf2 | EKF (thư viện ECL) ước lượng tư thế, vị trí, vận tốc, bias cảm biến, gió |
| commander | Quản lý trạng thái hệ thống: arm/disarm, chế độ bay, failsafe, health check |
| navigator | Mission (từ dataman), takeoff, RTL, geofence. Xuất `position_setpoint_triplet` |
| flight_mode_manager | Quản lý Flight Task theo hành vi bay (manual, mission, orbit...), chuyển setpoint navigator thành quỹ đạo |
| mc_pos_control / mc_att_control / mc_rate_control (và tương ứng fixed-wing/VTOL) | Chuỗi điều khiển tầng vị trí → tư thế → tốc độ góc |
| control_allocator | Chuyển mô-men/lực đẩy thành lệnh actuator theo hình học khung — thay mixer cũ từ v1.14 |
| manual_control | Xử lý input thủ công: RC, joystick qua MAVLink |
| dataman | Lưu trữ bền vững: waypoint mission, trạng thái mission, geofence |
| logger | Sinh log ULog |
| mavlink | Mã hóa/giải mã MAVLink cho GCS, companion computer, thiết bị ngoại vi |
| uxrce_dds_client | Cầu nối uORB sang ROS 2 qua XRCE-DDS |
| uorb | Quản lý message bus pub-sub, khởi động sớm nhất khi boot |

---

## 4. Luồng điều khiển

Cascaded control, tách vòng lặp theo tần số và mục tiêu vật lý:

1. Navigator/flight task sinh setpoint vị trí/vận tốc.
2. Position controller so setpoint với vị trí ước lượng từ EKF2, xuất setpoint tư thế + lực đẩy.
3. Attitude controller so quaternion mong muốn với ước lượng, xuất setpoint tốc độ góc.
4. Rate controller (vòng nhanh nhất) so tốc độ góc mong muốn với gyro đo được, xuất mô-men xoắn.
5. Control allocation chuyển mô-men/lực đẩy thành PWM/DShot cho từng động cơ/servo.

Tùy chế độ bay, vòng vị trí có thể bị bỏ qua (Acro chỉ dùng vòng tốc độ góc); multiplexer sau vòng ngoài quyết định.

---

## 5. Thuật toán điều khiển

Hỗn hợp P và PID. Vòng vị trí dùng P; vòng tốc độ góc dùng K-PID với chống windup, đầu ra giới hạn (thường -1 đến 1) tại control allocation. Đường vi phân dùng LPF giảm nhiễu — driver gyro cấp sẵn đạo hàm đã lọc. Tuning tư thế/cánh cố định nên làm ở tốc độ giữa stall speed và vận tốc tối đa để có biên ổn định lớn nhất.

Ngoài PID cascaded, PX4 hỗ trợ NMPC, INDI qua module tùy biến thay thế/bổ sung cho mc_pos_control/mc_att_control, dùng chung giao diện uORB.

---

## 6. Định vị và bám

EKF2 (thư viện ECL, trước là repo riêng PX4-ECL, nay hợp nhất vào PX4-Autopilot) ước lượng: quaternion NED → khung thân, vị trí, vận tốc, bias gia tốc kế/gyro, gió, bias khí áp. Dùng công thức error-state để ổn định ước lượng độ bất định phép quay (SO(3)).

EKF chạy trên delayed fusion time horizon để bù độ trễ cảm biến so với IMU — dữ liệu đệm FIFO, lấy đúng thời điểm cần; độ trễ cấu hình qua `EKF2_*_DELAY`. Complementary filter lan truyền trạng thái từ thời điểm hợp nhất tới hiện tại dựa trên IMU đệm.

Nguồn dữ liệu bám: GPS/GNSS (`EKF2_REQ_*`, `EKF2_GPS_CHECK`), range finder (độ cao/địa hình), vision/optical flow (`EKF2_EV_*`), airspeed + synthetic sideslip (gió, cánh cố định), drag specific force (đa cánh quạt). Terrain Hold và Range Aid dùng range finder có điều kiện để giữ độ cao gần mặt đất.

Bám quỹ đạo nằm ở lớp trên: navigator sinh `position_setpoint_triplet`, flight task/position controller chuyển thành lệnh bám waypoint, có acceptance radius và first order hold khi tới gần waypoint.

---

## 7. Quản lý bộ nhớ và lưu trữ

- **Parameters**: lưu non-volatile (file hệ thống hoặc flash tùy board), dùng cho tuning, hình học khung, giới hạn an toàn. Thư viện flashfs (`src/lib/parameters/flashparams`) cấp API đọc/ghi cho board lưu tham số trực tiếp trên flash.
- **dataman**: cơ sở dữ liệu đơn giản qua API C, nhiều backend (file SD, FLASH, FRAM, RAM cho test). Dữ liệu theo kiểu (waypoint, mission state, geofence, safe points), mỗi kiểu giới hạn số item cố định để truy cập nhanh. Đọc/ghi một item atomic; thao tác nhiều item cùng kiểu dùng khóa `dm_lock`.
- **Logging (ULog)**: định dạng tự mô tả, chứa cả định nghĩa cấu trúc message. Logger ghi cảm biến thô, trạng thái nội bộ (CPU, tư thế, EKF), message chuỗi (`PX4_INFO`/`PX4_ERR`). Topic uORB cần đăng ký trong `add_default_topics` hoặc file cấu hình SD mới được log mặc định.

---

## 8. Giao tiếp giữa module — uORB

uORB là cơ chế publish/subscribe bất đồng bộ nội bộ. Module gọi `orb_advertise` để công bố topic, `orb_subscribe` để đăng ký nhận. `orb_advertise_multi`/`orb_subscribe_multi` cho phép nhiều instance cùng topic (nhiều cảm biến cùng loại). uORB khởi động sớm nhất khi boot vì hầu hết module phụ thuộc vào nó; kiểm thử bằng `uorb_tests`.

Định nghĩa message nằm trong file `.msg` (thư mục `msg/`), đặt tên CamelCase, khai báo trong `msg/CMakeLists.txt`. Sơ đồ publish/subscribe (tự sinh từ mã nguồn) thể hiện quan hệ module–topic: nét đứt = publish, nét liền = subscribe, chấm-gạch = cả hai. Một số topic dùng chung nhiều publisher/subscriber như `parameter_update`, `mavlink_log`, `log_message`.

Công cụ debug: shell liệt kê trạng thái topic (tên, tần số, số subscriber, số message mất), system-wide replay tái hiện phiên bay từ log, gửi giá trị debug tùy biến, debug phần cứng qua SWD/JTAG với GDB hoặc Eclipse/JLink. Failure injection mô phỏng lỗi cảm biến/hệ thống để kiểm thử phản ứng an toàn.

---

## 9. Giao thức giao tiếp bên ngoài

- **MAVLink**: giao thức chính giữa flight controller và GCS (QGroundControl), companion computer, thiết bị ngoại vi (gimbal, ADS-B). Module mavlink mã hóa/giải mã, định tuyến qua serial/UDP cấu hình được (TELEM1/TELEM2 mặc định phục vụ GCS và companion computer).
- **uXRCE-DDS**: cho phép topic uORB publish/subscribe từ companion computer như topic ROS 2, dùng eProsima Micro XRCE-DDS. Client chạy trên PX4, agent chạy trên companion computer, trao đổi qua serial hoặc UDP; agent proxy dữ liệu vào không gian DDS toàn cục. Topic expose qua `dds_topics.yaml`, sinh mã lúc build; PX4-Autopilot export định nghĩa message vào repo `px4_msgs` cho ROS 2 dùng chung.
- **DroneCAN**: giao thức bus CAN hai chiều cho ESC, servo, cảm biến ngoại vi thông minh.
- **ROS 2 qua MAVROS**: thay thế cho uXRCE-DDS, không được docs.px4.io tài liệu hóa chính thức — thuộc dự án MAVROS riêng.

---

## 10. Phần bổ sung

- **Build system**: CMake. `src/lib` chứa thư viện dùng chung, `src/modules` chứa module chức năng, `boards/` chứa cấu hình theo board.
- **SITL**: chạy toàn bộ flight stack trên máy tính, kết hợp Gazebo hoặc mô phỏng khác, không cần phần cứng (`make px4_sitl gazebo`).
- **Commander và failsafe**: commander giám sát pin, GPS, liên kết RC/telemetry, health cảm biến theo thời gian thực; quyết định chuyển chế độ failsafe (RTL, land, hold) khi phát hiện bất thường — lớp an toàn tách biệt với bộ điều khiển bay.
- **Land detector**: phát hiện trạng thái rơi tự do và đã hạ cánh, mỗi loại khung máy bay có thuật toán riêng kế thừa từ lớp cơ sở chung, publish `vehicle_land_detected`.
- **Phân tích log sau chuyến bay**: Flight Review (dịch vụ web phân tích ULog), PlotJuggler, Foxglove.