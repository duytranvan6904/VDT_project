# Landing Diagnostics — Guide

## 1. Cài đặt

```bash
cd VDT_Project/landing_diagnostics
pip install -r requirements.txt
```

## 2. Chuẩn bị dữ liệu

- File log CSV từ `flight_data_logger` sau chuyến bay thử (có cột `fsm_state`, `pos_*`, `vel_*`, `sp_vx/vy`).
- Cột bắt buộc được kiểm tra khi load: `timestamp`, `fsm_state`, position, velocity và `sp_vx/sp_vy/sp_vz`.
- Các giá trị số không chuyển được sang numeric, NaN hoặc vô hạn sẽ bị loại khỏi phân tích.
- Timestamp được giả định là microseconds, phải tăng dần trong phase LAND; dòng trùng timestamp bị loại. Cần tối thiểu 5 dòng LAND hợp lệ.
- File tham số hiện tại đọc từ QGroundControl > Parameters, tự điền vào `current_params.json`:

```json
{
  "MPC_XY_VEL_P_ACC": 1.5,
  "MPC_XY_VEL_I_ACC": 0.4,
  "MPC_XY_VEL_D_ACC": 0.0,
  "MPC_LAND_SPEED": 0.7,
  "MPC_LAND_ALT2": 5.0
}
```

## 3. Chạy

```bash
python3 -m landing_diagnostics.report flight_log_20260908.csv --params-json current_params.json
```

Không có `current_params.json` vẫn chạy được, chỉ là không có phần đề xuất giá trị cụ thể:

```bash
python3 -m landing_diagnostics.report flight_log_20260908.csv
```

## 4. Đọc kết quả

Nếu dữ liệu LAND hợp lệ không đủ 5 dòng hoặc timestamp không tăng dần, report dừng với `ValueError` thay vì tạo metric không đáng tin cậy. Correlation sẽ trả `0.0` nếu dữ liệu hằng hoặc kết quả không finite.

```
mean_err_xy   = 0.32 m/s   # sai số tốc độ ngang trung bình khi LAND
oscillation   = 0.8 Hz     # dao động quanh setpoint
altitude_corr = -0.61      # sai số tăng khi gần đất (ground effect)
horiz_drift   = 1.20 m     # tổng độ lệch vị trí từ đầu đến cuối LAND

Chan doan: underdamped / steady_state_offset / ground_effect / setpoint_or_estimator_issue
```

- `underdamped` hoặc `steady_state_offset` hoặc `ground_effect` → có đề xuất tham số PID/land speed mới, thử trên SITL trước khi lên phần cứng thật.
- `setpoint_or_estimator_issue` → **không chỉnh PID**, quay lại kiểm tra `offboard_manager` (setpoint có giật không) hoặc chất lượng EKF trước.

## 5. Áp thử tham số mới (trên SITL, không lên thẳng máy thật)

```bash
make px4_sitl gz_x500
param set MPC_XY_VEL_P_ACC <giá_trị_đề_xuất>
```

Bay lại kịch bản landing cũ trong SITL, log lại, chạy diagnostics lại — lặp tới khi `mean_err_xy` và `oscillation` giảm rõ rệt thì mới đưa lên phần cứng thật.

## 6. Unit test

Từ thư mục repository:

```bash
python3 -m pip install -r landing_diagnostics/requirements.txt pytest
python3 -m pytest landing_diagnostics/test_metrics.py
```

Test bao phủ schema CSV, dữ liệu LAND ngắn, NaN/vô hạn, timestamp trùng, FFT, phân loại drift và đề xuất tuning.