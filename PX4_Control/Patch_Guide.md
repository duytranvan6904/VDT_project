# Guide sửa code mc_pos_control (Position/Velocity PID)

## 1. Nguồn tham khảo chính thức

- Source code module: https://github.com/PX4/PX4-Autopilot/tree/main/src/modules/mc_pos_control
- File chứa PID thật (P-controller vị trí + PID-controller vận tốc):
  https://github.com/PX4/PX4-Autopilot/blob/main/src/modules/mc_pos_control/PositionControl/PositionControl.cpp
- File module wrapper (đọc uORB, gọi PositionControl, publish setpoint):
  https://github.com/PX4/PX4-Autopilot/blob/main/src/modules/mc_pos_control/MulticopterPositionControl.cpp
- Doxygen class reference (mô tả input/output của `PositionControl`):
  https://px4.github.io/Firmware-Doxygen/d8/d5a/class_position_control.html
- PID Tuning Guide chính thức (đọc trước khi sửa code — 90% trường hợp chỉ cần tune tham số, không cần sửa source):
  https://docs.px4.io/main/en/config_mc/pid_tuning_guide_multicopter
- Controller layering (thứ tự rate → attitude → velocity/position):
  https://docs.px4.io/main/en/config_mc/pid_tuning_guide_multicopter_basic

Lưu ý: PX4 có 3 nhánh doc theo version (`main`, `v1.16`, `v1.15`...). Luôn đổi URL sang đúng version bạn đang checkout, ví dụ `docs.px4.io/v1.15/...`, vì tham số/API có thể khác giữa các bản.

## 2. Cấu trúc file cần biết trong `mc_pos_control/`

```
src/modules/mc_pos_control/
  MulticopterPositionControl.cpp    <- vòng lặp chính, đọc local_position, setpoint,
                                        gọi _control.setInputSetpoint()/update(),
                                        publish vehicle_attitude_setpoint
  MulticopterPositionControl.hpp
  PositionControl/
    PositionControl.cpp             <- PID thực sự nằm ở đây
    PositionControl.hpp
    ControlMath.cpp                 <- hàm toán phụ trợ (constrain, thrustToAttitude...)
```

## 3. Điểm neo (anchor) đã xác minh còn tồn tại trong source hiện tại

Vì số dòng thay đổi giữa các version, **không dựa vào số dòng cố định** — dùng `grep` tìm đúng chuỗi anchor này trong file `PositionControl.cpp` của bạn:

```bash
grep -n "_gain_vel_p" src/modules/mc_pos_control/PositionControl/PositionControl.cpp
grep -n "thrust_desired_NE" src/modules/mc_pos_control/PositionControl/PositionControl.cpp
```

Đoạn PID vận tốc thật sự (P + D + tích phân `_thr_int`) nằm ngay tại dòng có dạng:

```cpp
thrust_desired_NE(0) = _gain_vel_p(0) * vel_err(0) + _gain_vel_d(0) * _vel_dot(0) + _thr_int(0);
thrust_desired_NE(1) = _gain_vel_p(1) * vel_err(1) + _gain_vel_d(1) * _vel_dot(1) + _thr_int(1);
```

Ngay sau đó là khối anti-windup (comment nhắc tới "Anti-Reset Windup", L.Rundqwist 1990) — đây là nơi duy nhất nên can thiệp nếu mục tiêu là chỉnh hành vi PID vận tốc ngang.

## 4. Quy trình patch an toàn, không sửa trực tiếp lên nhánh chính

Không sửa thẳng vào file trong `PX4-Autopilot`. Luôn giữ thay đổi dưới dạng file `.patch` riêng, để khi PX4 ra version mới, bạn áp lại patch thay vì phải nhớ sửa tay lại từ đầu.

```bash
cd PX4-Autopilot
git checkout -b custom-posctl-patch

# ... sửa code theo template bên dưới ...

git diff > ../patches/001_custom_velocity_term.patch
git checkout main   # hoặc quay lại nhánh gốc, không giữ commit trực tiếp trên main
```

Áp patch lại sau khi PX4 cập nhật version mới:

```bash
git apply ../patches/001_custom_velocity_term.patch
# nếu conflict do version khác biệt nhiều:
git apply --reject ../patches/001_custom_velocity_term.patch
```

## 5. Tạo patch có khóa commit/version

Không chỉ lưu file `.patch` đơn lẻ. Dùng [templates/README.md](templates/README.md) và `create_px4_patch.py` để tạo bundle gồm patch, manifest JSON và checksum:

```bash
python3 PX4_Control/templates/create_px4_patch.py \
  --px4-dir /path/to/PX4-Autopilot \
  --output-dir /path/to/patches \
  --patch-name 001_custom_velocity_pid.patch \
  --base-ref v1.16.0
```

Manifest lưu chính xác `HEAD` commit, `git describe`, branch, file thay đổi và SHA-256. `apply_patch.sh` chỉ áp dụng patch khi checkout PX4 đang sạch, đúng commit, checksum đúng và `git apply --check` thành công. Nếu đổi PX4 release/commit, phải tạo patch bundle mới.

Các template trong `templates/` là hướng dẫn chèn code theo anchor, không phải patch hoàn chỉnh cho mọi PX4 version. Luôn review diff và build SITL sau khi áp dụng.