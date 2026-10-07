# Takeoff — Guide

## 1. Package cần cài đặt ngoài

Cần package `px4_msgs`, clone và build từ nguồn vào workspace. Phiên bản phải khớp với firmware PX4 đang chạy. Micro XRCE-DDS Agent phải đang chạy để các topic `/fmu/*` hoạt động.

## 2. Nguyên lý hoạt động

Node chạy một lần: arm, ra lệnh PX4 takeoff, chờ đạt độ cao rồi thoát. Node không publish setpoint Offboard.

```text
/fmu/out/vehicle_status          --\
/fmu/out/vehicle_local_position  ---> takeoff (one-shot) ---> /fmu/in/vehicle_command
system/killed                    --/
```

### Trình tự

1. Chờ nhận `vehicle_status` và `vehicle_local_position` (tối đa 10 s).
2. Kiểm tra điều kiện: `system/killed` không phải `true` và UAV chưa `ARMED`.
3. Gửi `VEHICLE_CMD_COMPONENT_ARM_DISARM` (arm), chờ `ARMED` (tối đa 5 s).
4. Gửi `VEHICLE_CMD_NAV_TAKEOFF` với param4 đến param7 là NaN. PX4 cất cánh tại chỗ lên độ cao `MIS_TAKEOFF_ALT` so với vị trí hiện tại.
5. Chờ điều kiện sẵn sàng (tối đa `--climb-timeout`), rồi thoát.

### Điều kiện sẵn sàng

Một trong hai điều kiện đúng:

- `VehicleStatus.nav_state == OFFBOARD`: `offboard_manager` đã tiếp quản.
- `-z >= 0.9 * --alt` với `VehicleLocalPosition.z_valid`.

Sau khi đạt, PX4 Takeoff tự chuyển sang Hold, hoặc `offboard_manager` engage Offboard nếu đủ điều kiện. Node không gửi thêm lệnh nào.

### Xử lý lỗi

| Tình huống | Hành vi | Exit code |
|---|---|---|
| Không nhận `vehicle_status`/`local_position` sau 10 s | Báo lỗi, không gửi lệnh | 1 |
| `system/killed = true` | Báo lỗi, không gửi lệnh | 1 |
| UAV đã `ARMED` | Báo lỗi, bỏ qua lệnh | 1 |
| Arm không thành công sau 5 s | Báo lỗi, không gửi takeoff | 1 |
| Không đạt độ cao trong `--climb-timeout` | Gửi `VEHICLE_CMD_NAV_LAND`, báo lỗi | 1 |
| Nhận `system/killed = true` khi đang cất cánh | Báo lỗi, không gửi land | 1 |
| Đạt độ cao | In độ cao và `nav_state` | 0 |

### Phối hợp với offboard_manager

`offboard_node` chỉ engage khi UAV đã `ARMED` và `-z >= min_engage_altitude_m`, nên không xung đột với node này. Để việc chuyển sang Offboard xảy ra khi UAV đã gần tới độ cao takeoff, đặt `min_engage_altitude_m` thấp hơn `MIS_TAKEOFF_ALT` một khoảng nhỏ (ví dụ takeoff 3.0 m, engage 2.7 m). Không đặt quá sát độ cao takeoff, vì nếu PX4 dừng trong vùng sai số thì sẽ không bao giờ engage.

Sau khi `offboard_node` engage ở FSM SEARCH, UAV quay yaw theo `yaw_search_rate`. Muốn UAV chỉ đứng yên khi test, đặt `yaw_search_rate:=0.0`.

### Giới hạn

- Node chỉ kiểm tra `system/killed`. Không kiểm tra `safety/force_land` và `safety/inhibit_offboard`. Nếu một trong hai đang `true`, UAV vẫn có thể cất cánh và nằm lại ở PX4 Hold vì `offboard_node` không engage.
- Độ cao takeoff do tham số PX4 `MIS_TAKEOFF_ALT` quyết định. `--alt` chỉ dùng để xác nhận đã đạt độ cao và phải khớp với tham số này.
- Node không điều khiển gì sau khi thoát. Giữ độ cao do PX4 Hold hoặc `offboard_manager` đảm nhiệm.

## 3. Cách chạy

Chuẩn bị một lần:

```text
param set MIS_TAKEOFF_ALT 3.0
```

Lệnh trên chạy ở console PX4 SITL hoặc đặt trong QGroundControl (Parameters).

Trong `System_Params.yaml`, dưới `offboard_node`:

```yaml
min_engage_altitude_m: 2.7
```

Thứ tự chạy: khởi động Agent, PX4 và hệ thống (launch), rồi chạy takeoff ở terminal khác.

```bash
ros2 run takeoff takeoff --alt 3.0
```

Hoặc dùng script bọc, tự source ROS và workspace (đặt `VDT_WS` nếu workspace không nằm ở `~/ros2_ws`):

```bash
./takeoff.sh --alt 3.0
```

Không đưa lệnh này vào launch. Arm phải là hành động có chủ đích. Khi bay thật, giữ RC sẵn sàng chiếm quyền và công tắc kill trong tầm tay.

## 4. Tham số

| Tên | Mặc định | Ý nghĩa |
|---|---:|---|
| `--alt` | 3.0 | Độ cao kỳ vọng (m) để xác nhận đã đạt. Phải khớp `MIS_TAKEOFF_ALT` |
| `--climb-timeout` | 20.0 | Thời gian tối đa (s) chờ đạt độ cao trước khi gửi land |

Giá trị cố định trong code: chờ dữ liệu ban đầu 10 s, chờ arm 5 s, ngưỡng sẵn sàng `0.9 * alt`.

## 5. Cách debug

Theo dõi chuỗi chế độ bay:

```bash
ros2 topic echo /fmu/out/vehicle_status --field nav_state
```

Chuỗi đúng là AUTO_TAKEOFF rồi OFFBOARD (14). Nếu về AUTO_LOITER (4) thì Offboard đang bị chặn và PX4 Hold đang giữ UAV.

Xem lệnh node gửi ra PX4:

```bash
ros2 topic echo /fmu/in/vehicle_command
```

Xem độ cao và trạng thái arm mà node dùng để quyết định:

```bash
ros2 topic echo /fmu/out/vehicle_local_position --field z
ros2 topic echo /fmu/out/vehicle_status --field arming_state
```

Nếu báo không nhận được `vehicle_status`/`local_position`:

- Kiểm tra Micro XRCE-DDS Agent đang chạy và `ros2 topic list` có các topic `/fmu/out/*`.
- Kiểm tra tên topic trong `px4_msgs` có hậu tố phiên bản không. Nếu có, sửa cho khớp với `offboard_node`.

Nếu arm không thành công:

- Xem thông báo preflight trong QGroundControl (GPS/EKF chưa sẵn sàng, chưa hiệu chuẩn, công tắc safety).
- Kiểm tra `system/killed` không phải `true`.

Nếu arm được nhưng UAV không lên hoặc lên sai độ cao:

- Kiểm tra `MIS_TAKEOFF_ALT` đã đặt đúng và khớp `--alt`.
- Kiểm tra `ros2 topic echo /fmu/in/vehicle_command` có lệnh `NAV_TAKEOFF` (command 22).

Nếu UAV dừng ở độ cao thấp hơn mong muốn và `nav_state` đã là OFFBOARD: `offboard_node` đã engage sớm. Tăng `min_engage_altitude_m` lại gần độ cao takeoff.

Nếu node gửi land sau timeout: UAV không đạt `0.9 * alt` trong `--climb-timeout`. Kiểm tra `z_valid` của local position, độ cao `MIS_TAKEOFF_ALT`, và tăng `--climb-timeout` nếu tốc độ leo chậm.