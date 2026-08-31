# Kiến trúc Flight Stack PX4 — Tổng hợp từ mã nguồn và tài liệu chính thức

## 1. Nguồn trích dẫn đáng tin cậy

- Mã nguồn chính: https://github.com/PX4/PX4-Autopilot (giấy phép BSD 3-Clause, Dronecode Foundation/Linux Foundation quản lý)
- Tài liệu kiến trúc tổng quan: https://docs.px4.io/main/en/concept/architecture
- Tài liệu kiến trúc hệ thống (flight controller + companion computer): https://docs.px4.io/main/en/concept/px4_systems_architecture
- uORB messaging: https://docs.px4.io/main/en/middleware/uorb và series giải thích chi tiết https://px4.io/px4-uorb-explained-part-1/
- Sơ đồ publish/subscribe giữa các module (tự sinh từ mã nguồn): https://docs.px4.io/main/en/middleware/uorb_graph
- EKF2/ECL (bộ lọc định vị): https://docs.px4.io/main/en/advanced_config/tuning_the_ecl_ekf, mã nguồn thư viện tại src/lib/ecl trong PX4-Autopilot
- Control allocation (mixer thế hệ mới): https://docs.px4.io/main/en/concept/control_allocation
- Sơ đồ bộ điều khiển tầng (controller diagrams): https://docs.px4.io/main/en/flight_stack/controller_diagrams
- Navigator/mission: https://docs.px4.io/main/en/modules/modules_controller, mã nguồn src/modules/navigator
- Logging/ULog: https://docs.px4.io/main/en/dev_log/logging, đặc tả định dạng https://docs.px4.io/main/en/dev_log/ulog_file_format
- uXRCE-DDS (cầu nối ROS 2): https://docs.px4.io/main/en/middleware/uxrce_dds
- Dataman (lưu trữ bền vững cho mission/geofence): mã nguồn src/modules/dataman, doc mô tả tại https://bkueng.gitbooks.io/px4-devguide/content/en/middleware/modules_system.html
- Tổng hợp kiến trúc theo hướng đọc mã nguồn (không chính thức nhưng bám sát source, hữu ích để tra cứu nhanh từng module): https://deepwiki.com/PX4/PX4-Autopilot

Nguyên tắc dùng nguồn: ưu tiên docs.px4.io (tài liệu chính thức của dự án) và mã nguồn trực tiếp trên GitHub PX4/PX4-Autopilot; các nguồn khác (blog cá nhân, wiki bên thứ ba) chỉ nên dùng để đối chiếu, không thay thế mã nguồn khi cần độ chính xác kỹ thuật cao.

## 2. Kiến trúc tổng quát

PX4 chia làm hai lớp lớn: flight stack (ước lượng trạng thái và điều khiển bay) và middleware (lớp robot tổng quát, xử lý giao tiếp nội bộ/ngoại vi và tích hợp phần cứng). Toàn bộ airframe (đa cánh quạt, cánh cố định, VTOL, tàu, rover, tàu ngầm...) dùng chung một codebase. Thiết kế theo hướng phản ứng (reactive): chức năng được chia thành các thành phần độc lập, có thể hoán đổi, giao tiếp bằng truyền message bất đồng bộ qua uORB.

Về hệ điều hành: NuttX là RTOS chính chạy trên board flight controller — mã nguồn mở, nhẹ, hiệu năng ổn định. Mỗi module PX4 chạy như một task riêng (có file descriptor list riêng) nhưng chia sẻ không gian địa chỉ chung. Middleware còn có lớp mô phỏng (SITL) cho phép chạy toàn bộ flight code trên máy tính thường mà không cần phần cứng.

Về mặt vật lý, một hệ thống PX4 điển hình gồm: flight controller (chạy flight stack, có IMU/compass/baro tích hợp), ESC/động cơ nối qua PWM/DroneCAN, các cảm biến ngoại vi (GPS, la bàn, cảm biến khoảng cách, optical flow...) nối qua I2C/SPI/CAN/UART. Với hệ thống nâng cao hơn có thêm companion computer (mission computer) chạy Linux, kết nối với flight controller qua liên kết serial/IP tốc độ cao, thường dùng MAVLink; companion computer đảm nhiệm các tác vụ thị giác máy tính, còn giao tiếp với ground station/cloud thường được định tuyến qua companion computer.

## 3. Kiến trúc các module chức năng chính

Mã nguồn được chia module hóa dưới src/modules (chương trình chạy như task độc lập) và src/lib (thư viện dùng chung). Các module chính:

- **sensors**: thu thập, hiệu chỉnh, tổng hợp dữ liệu từ IMU/baro/mag/GPS/airspeed thành các topic uORB chuẩn hóa.
- **ekf2**: chạy bộ lọc Kalman mở rộng (dựa trên thư viện ECL) để ước lượng tư thế, vị trí, vận tốc, bias cảm biến, gió...
- **commander**: quản lý trạng thái hệ thống (arm/disarm, chế độ bay, failsafe, health check), là "bộ não" giám sát an toàn.
- **navigator**: chịu trách nhiệm các chế độ bay tự động — mission (đọc từ dataman), takeoff, RTL (return-to-launch), kiểm tra vi phạm geofence. Các chế độ con kế thừa từ lớp cơ sở chung NavigatorMode; navigator xuất bản position_setpoint_triplet cho bộ điều khiển vị trí sử dụng.
- **flight_mode_manager** (trước đây gọi là mc_pos_control cho phần flight task): quản lý các Flight Task — cài đặt module hóa cho từng kiểu hành vi bay (manual position, auto mission, orbit...), chuyển setpoint từ navigator thành quỹ đạo cụ thể cho từng loại phương tiện.
- **mc_pos_control / mc_att_control / mc_rate_control** (đa cánh quạt) và tương ứng cho fixed-wing/VTOL: chuỗi bộ điều khiển tầng vị trí → tư thế → tốc độ góc.
- **control_allocator**: nhận lệnh mô-men xoắn và lực đẩy mong muốn từ bộ điều khiển lõi, chuyển thành lệnh actuator cụ thể tùy hình học khung máy bay (thay thế cơ chế mixer cũ từ PX4 v1.14 trở đi).
- **manual_control**: chọn và xử lý input thủ công (RC, joystick qua MAVLink...).
- **dataman**: cơ sở dữ liệu bền vững đơn giản lưu waypoint mission, trạng thái mission, đa giác geofence.
- **logger**: sinh log bay định dạng ULog.
- **mavlink**: đóng gói/giải mã giao thức MAVLink, phục vụ giao tiếp với GCS, companion computer, các thiết bị ngoại vi MAVLink.
- **uxrce_dds_client**: cầu nối uORB sang ROS 2 qua giao thức XRCE-DDS.
- **uorb**: khởi động và quản lý message bus publish/subscribe nội bộ, khởi động sớm nhất trong quá trình boot vì hầu hết module khác phụ thuộc vào nó.

## 4. Luồng điều khiển (control flow)

PX4 dùng kiến trúc điều khiển tầng (cascaded control), tách biệt các vòng lặp theo tần số và mục tiêu vật lý khác nhau, mỗi tầng ngoài cung cấp setpoint cho tầng trong:

1. Navigator/flight task sinh setpoint vị trí hoặc vận tốc mong muốn.
2. Bộ điều khiển vị trí (position controller) so sánh setpoint với vị trí ước lượng từ EKF2, xuất ra setpoint tư thế (attitude) và lực đẩy.
3. Bộ điều khiển tư thế (attitude controller) so sánh quaternion mong muốn với quaternion ước lượng, xuất ra setpoint tốc độ góc.
4. Bộ điều khiển tốc độ góc (rate controller) — vòng lặp nhanh nhất, tần số cao nhất — so sánh tốc độ góc mong muốn với tốc độ góc đo từ gyro, xuất ra lệnh mô-men xoắn.
5. Control allocation chuyển mô-men xoắn/lực đẩy thành lệnh PWM/DShot cụ thể cho từng động cơ/servo theo hình học khung máy bay.

Tùy chế độ bay, vòng ngoài (vị trí) có thể bị bỏ qua (ví dụ chế độ Acro chỉ dùng vòng tốc độ góc); cơ chế multiplexer sau vòng ngoài quyết định việc này.

## 5. Thuật toán điều khiển

Các bộ điều khiển trong chuỗi tầng là hỗn hợp bộ điều khiển P và PID. Vòng vị trí thường dùng P; vòng tốc độ góc dùng cấu trúc K-PID với giới hạn windup cho thành phần tích phân, đầu ra được giới hạn (thường trong khoảng -1 đến 1) tại control allocation. Đường vi phân sử dụng bộ lọc thông thấp (LPF) để giảm nhiễu — driver gyro cung cấp sẵn đạo hàm đã lọc cho bộ điều khiển. Việc tinh chỉnh (tuning) bộ điều khiển tư thế/cánh cố định nên thực hiện ở tốc độ bay giữa vận tốc stall và vận tốc tối đa để có biên độ bay ổn định lớn nhất.

Ngoài chuỗi PID cascaded truyền thống, PX4 còn hỗ trợ các phương pháp điều khiển thay thế qua cơ chế module hóa (ví dụ các nghiên cứu mở rộng dùng NMPC, INDI thường được triển khai như module tùy biến thay thế hoặc bổ sung cho mc_pos_control/mc_att_control, tận dụng cùng giao diện uORB).

## 6. Định vị và bám (estimation & tracking)

Bộ ước lượng chính là EKF2, dựa trên thư viện ECL (Estimation & Control Library, trước đây là repo riêng PX4/PX4-ECL, nay đã hợp nhất vào PX4-Autopilot). EKF2 ước lượng: quaternion định hướng từ khung dẫn đường NED sang khung thân, vị trí, vận tốc, bias gia tốc kế/gyro, và có thể ước lượng gió, bias khí áp. Thuật toán dùng công thức "error-state" để ổn định việc ước lượng độ bất định của phép quay (không gian tiếp tuyến của SO(3)).

EKF chạy trên "chân trời hợp nhất trễ" (delayed fusion time horizon) để bù các độ trễ khác nhau của từng cảm biến so với IMU — dữ liệu mỗi cảm biến được đệm FIFO và lấy ra đúng thời điểm cần dùng; độ trễ bù cho từng cảm biến cấu hình qua tham số EKF2_*_DELAY. Một bộ lọc bù (complementary filter) lan truyền trạng thái từ thời điểm hợp nhất tới thời điểm hiện tại dựa trên dữ liệu IMU đã đệm.

Nguồn dữ liệu bám gồm GPS/GNSS (có kiểm tra chất lượng qua các tham số EKF2_REQ_*, EKF2_GPS_CHECK), cảm biến khoảng cách (range finder) cho ước lượng độ cao/địa hình, vision/optical flow (external vision qua các tham số EKF2_EV_*), airspeed và bên trượt tổng hợp (synthetic sideslip) cho ước lượng gió ở cánh cố định, cảm biến lực cản riêng (drag specific force) cho đa cánh quạt. Chế độ Terrain Hold và Range Aid dùng dữ liệu range finder có điều kiện để giữ độ cao khi bay gần mặt đất.

Việc bám quỹ đạo/mục tiêu (setpoint tracking) nằm ở lớp trên: navigator sinh position_setpoint_triplet, flight task/position controller chuyển thành lệnh bám theo waypoint, có bán kính chấp nhận (acceptance radius) và cơ chế nội suy bậc một (first order hold) khi tới gần waypoint.

## 7. Quản lý bộ nhớ và lưu trữ dữ liệu

- **Tham số hệ thống (parameters)**: lưu trong bộ nhớ non-volatile (file hệ thống hoặc flash filesystem tùy board), dùng cho cấu hình tuning, hình học khung máy bay, giới hạn an toàn... Thư viện flashfs trong src/lib/parameters/flashparams cung cấp API đọc/ghi cấp thấp cho board hỗ trợ lưu tham số trực tiếp trên flash.
- **dataman**: module cung cấp một "cơ sở dữ liệu" đơn giản qua API C, hỗ trợ nhiều backend (file trên thẻ SD, FLASH nếu board hỗ trợ, FRAM, hoặc RAM không bền vững — dùng cho test). Dữ liệu được tổ chức theo kiểu (waypoint mission, trạng thái mission, điểm geofence, điểm an toàn/safe points), mỗi kiểu có số lượng item tối đa cố định để đảm bảo truy cập ngẫu nhiên nhanh. Đọc/ghi một item luôn nguyên tử (atomic); nếu cần thao tác nguyên tử trên nhiều item cùng kiểu, có cơ chế khóa riêng theo kiểu (dm_lock).
- **Logging (ULog)**: định dạng tự mô tả (self-describing) — file log chứa cả định nghĩa cấu trúc message được log. Logger module log dữ liệu từ cảm biến thô, trạng thái nội bộ (tải CPU, tư thế, trạng thái EKF...), và message chuỗi (PX4_INFO/PX4_ERR). Không phải mọi topic uORB đều được log mặc định — cần đăng ký trong danh sách topic mặc định của logger (add_default_topics) hoặc file cấu hình trên thẻ SD.

## 8. Giao tiếp giữa các module — uORB

uORB là cơ chế publish/subscribe bất đồng bộ dùng cho giao tiếp liên-tiến-trình/liên-luồng nội bộ PX4. Mỗi topic có ít nhất một publisher và một subscriber; module gọi orb_advertise để công bố topic, orb_subscribe để đăng ký nhận. Có cơ chế orb_advertise_multi/orb_subscribe_multi cho phép nhiều thực thể độc lập của cùng một topic (hữu ích khi có nhiều cảm biến cùng loại). uORB tự khởi động sớm trong quá trình boot vì phần lớn module phụ thuộc vào nó; có thể chạy uorb_tests để kiểm thử.

Định nghĩa message nằm trong file .msg (thư mục msg/), theo quy ước đặt tên CamelCase, được khai báo trong msg/CMakeLists.txt. Tài liệu PX4 cung cấp một đồ thị publish/subscription (sinh tự động từ mã nguồn) thể hiện toàn bộ quan hệ module–topic: đường nét đứt là publish, nét liền là subscribe, nét chấm-gạch là vừa publish vừa subscribe. Một số topic có nhiều publisher/subscriber dùng chung như parameter_update, mavlink_log, log_message.

Về debug/interface giữa các module: PX4 cung cấp công cụ dòng lệnh (shell) để liệt kê trạng thái các topic (tên, tần số publish, số subscriber, số message mất...), công cụ replay toàn hệ thống (system-wide replay) để tái hiện lại một phiên bay từ log, cơ chế gửi giá trị debug tùy biến, và hỗ trợ debug phần cứng qua SWD/JTAG với GDB hoặc Eclipse/JLink. Failure injection cho phép mô phỏng lỗi cảm biến/hệ thống để kiểm thử phản ứng an toàn.

## 9. Giao thức giao tiếp bên ngoài

- **MAVLink**: giao thức chính giữa flight controller và ground control station (QGroundControl), companion computer, thiết bị ngoại vi (camera gimbal, ADS-B...). Module mavlink trong PX4 đảm nhiệm mã hóa/giải mã, định tuyến message qua các cổng serial/UDP cấu hình được (ví dụ TELEM1/TELEM2 mặc định phục vụ GCS và companion computer).
- **uXRCE-DDS**: middleware cho phép topic uORB được publish/subscribe từ companion computer như thể là topic ROS 2, dùng triển khai eProsima Micro XRCE-DDS. Gồm một client chạy trên PX4 và một agent chạy trên companion computer, trao đổi dữ liệu hai chiều qua liên kết serial hoặc UDP; agent đóng vai trò proxy đưa dữ liệu vào không gian DDS toàn cục. Tập topic được expose qua cầu nối này được cấu hình trong dds_topics.yaml, sinh mã lúc build; PX4-Autopilot export định nghĩa message tương ứng vào repo px4_msgs để ứng dụng ROS 2 dùng cùng định nghĩa.
- **DroneCAN**: giao thức bus CAN hai chiều dùng cho ESC, servo và cảm biến ngoại vi thông minh.
- **ROS 2 qua MAVROS**: một lựa chọn thay thế cho uXRCE-DDS, không được tài liệu hóa chính thức trong docs.px4.io mà thuộc dự án MAVROS riêng.

## 10. Các phần bổ sung liên quan

- **Hệ thống build**: dùng CMake, mã nguồn chia theo thư mục rõ ràng — src/lib chứa thư viện dùng chung, src/modules chứa các module chức năng độc lập, boards/ chứa cấu hình theo từng loại board phần cứng.
- **SITL (Software-In-The-Loop)**: cho phép chạy toàn bộ flight stack trên máy tính, kết hợp Gazebo hoặc các mô phỏng khác để kiểm thử thuật toán mà không cần phần cứng thật (make px4_sitl gazebo...).
- **Commander và cơ chế failsafe**: commander module giám sát tình trạng hệ thống theo thời gian thực (pin, GPS, liên kết RC/telemetry, health cảm biến), quyết định chuyển chế độ failsafe (RTL, land, hold...) khi phát hiện bất thường; đây là lớp an toàn tách biệt với các bộ điều khiển bay.
- **Land detector**: module riêng phát hiện trạng thái rơi tự do và trạng thái đã hạ cánh, mỗi loại khung máy bay (đa cánh quạt, cánh cố định, VTOL...) có thuật toán riêng kế thừa từ lớp cơ sở chung, publish topic vehicle_land_detected.
- **Công cụ phân tích log sau chuyến bay**: Flight Review (dịch vụ web phân tích tương tác từ ULog), cùng các công cụ bên thứ ba như PlotJuggler, Foxglove hỗ trợ trực quan hóa ULog.

Tài liệu này tổng hợp từ các trang chính thức docs.px4.io (nhánh "main", tức phiên bản phát triển mới nhất) và mã nguồn PX4/PX4-Autopilot trên GitHub tại thời điểm tổng hợp; cấu trúc module/tham số có thể thay đổi giữa các phiên bản, nên khi làm việc với một bản PX4 cụ thể (ví dụ v1.14, v1.15, v1.16) nên đối chiếu tài liệu đúng phiên bản tương ứng.
