# PX4 Sensor & ESC Calibration qua QGroundControl — Guide

## 1. Chuẩn bị

- Máy tính đã cài QGroundControl, nối Pixhawk 6C qua **USB trực tiếp** (không qua telemetry radio, không qua UART micro-XRCE-DDS — link đó dành cho companion computer, không phải cho calibrate).
- Tháo hết cánh quạt trước khi bắt đầu, đặc biệt bắt buộc trước bước calibrate ESC.
- Cắm pin thật vào khung (không chỉ cấp nguồn qua USB) — calibrate ESC cần nguồn động cơ thật, IMU/Mag/Baro thì USB là đủ.
- Chọn nơi calibrate Compass thoáng, cách xa kim loại lớn, xe hơi, cốt thép sàn nhà, loa/nam châm — nhiễu từ trường tại chỗ là nguyên nhân phổ biến nhất khiến compass calibrate xong vẫn lệch heading khi bay.
- Đặt khung máy bay trên mặt phẳng chuẩn (water level hoặc mặt bàn phẳng đã kiểm tra) cho bước Level Horizon.

## 2. Nguyên lý hoạt động

Thứ tự calibrate không tùy ý — mỗi bước sau phụ thuộc dữ liệu bước trước:

| Thứ tự | Cảm biến | Vì sao đúng thứ tự này |
|---|---|---|
| 1 | Accelerometer (IMU) | Thiết lập hệ quy chiếu trọng lực/tư thế; compass calibrate sau cần bù nghiêng (tilt-compensation) dựa trên accel đã calibrate |
| 2 | Gyroscope | Thường tự động, chỉ cần giữ máy bay đứng yên hoàn toàn trong vài giây để lấy offset |
| 3 | Compass (Mag) | Cần accel đã đúng để tính tilt-compensation khi xoay máy bay theo 3 trục |
| 4 | Level Horizon | Bù lệch giữa mặt phẳng gắn Pixhawk và mặt phẳng khung thật (FC hiếm khi gắn tuyệt đối phẳng) |
| 5 | Baro | PX4 tự lấy offset áp suất lúc khởi động, không có nút calibrate tay trong QGC — chỉ cần đứng yên vài giây sau khi cấp nguồn |
| 6 | ESC | Không liên quan cảm biến — đồng bộ điểm throttle min/max giữa tất cả ESC để chúng arm/disarm và phản hồi ga đồng đều |

## 3. Cách chạy

Mở QGroundControl → **Vehicle Setup → Sensors**.

**3.1 Accelerometer**
```
Sensors > Accelerometer > "Calibrate"
```
Làm theo 6 hướng QGC yêu cầu trên màn hình (nằm ngang, nghiêng trái, nghiêng phải, mũi lên, mũi xuống, lật ngược) — mỗi hướng giữ yên tới khi thanh tiến trình đầy rồi mới chuyển hướng tiếp theo.

**3.2 Gyroscope**
```
Sensors > Gyroscope > "Calibrate"
```
Đặt máy bay trên mặt phẳng, giữ hoàn toàn đứng yên trong lúc calibrate (thường chỉ vài giây).

**3.3 Compass**
```
Sensors > Compass > "Calibrate"
```
Cầm/xoay máy bay theo đúng chuyển động QGC minh họa (xoay đủ cả 3 trục, kiểu "figure-8" mở rộng) tại khu vực đã chọn ở bước chuẩn bị.

**3.4 Level Horizon**
```
Sensors > Level Horizon > "Set Level Horizon"
```
Chỉ bấm khi khung đã đặt đúng mặt phẳng chuẩn.

**3.5 Baro**
Không có nút riêng — chỉ cần cấp nguồn, giữ máy bay đứng yên vài giây trước khi arm để offset áp suất ổn định. Nếu bay ở độ cao lớn hơn nhiều so với lúc calibrate, kiểm tra lại `EKF2` altitude estimate có hợp lý trước khi bay.

**3.6 ESC**
```
Vehicle Setup > Power (hoặc Actuators, tùy phiên bản QGC) > "ESC Calibration"
```
Xác nhận đã tháo cánh quạt trong popup cảnh báo, cắm pin, làm theo trình tự QGC gửi throttle max rồi về min để từng ESC tự học range của mình.

## 4. Cách debug

Kiểm tra sau khi calibrate xong, trước khi bay:

- **Accel/Gyro:** Sensors tab hiển thị đồ thị — đặt máy bay đứng yên, giá trị accel tổng phải quanh 9.81 m/s², không dao động lớn (noise cao → rung động cơ/gắn FC lỏng, không phải lỗi calibrate).
- **Compass:** sau khi calibrate, QGC báo chất lượng calibration (progress + màu xanh là tốt). Nếu báo đỏ/kém, calibrate lại ở vị trí khác, tránh hẳn khu vực cũ.
- **Attitude/HUD:** nghiêng máy bay bằng tay, quan sát HUD trong QGC phản ánh đúng hướng nghiêng thực tế (roll/pitch không bị đảo dấu hoặc lệch trục).
- **Pre-arm check:** thử arm (chưa gắn cánh quạt), đọc log "PreArm" trong QGC — nếu còn cảnh báo `Accel/Gyro/Mag/Baro`, quay lại calibrate đúng cảm biến đó trước khi tiếp tục.
- **ESC:** vào **Actuators** tab, test từng động cơ riêng lẻ ở ga thấp — xác nhận đúng chiều quay theo sơ đồ motor mapping của khung, và các động cơ phản hồi đồng đều ở cùng mức ga test (không có động cơ nào trễ/giật so với các động cơ còn lại).
- Nếu sau khi calibrate xong mà `EKF2` vẫn báo `ekf2_healthy = false` (theo field dùng ở Module 9 — Offboard Safety Monitor), quay lại calibrate Accel + Compass trước, vì `ekf2_healthy` phụ thuộc trực tiếp chất lượng hai cảm biến này, không phải lỗi ở phần Offboard/FSM.
