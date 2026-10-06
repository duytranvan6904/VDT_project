# EKF Adapter: Test Guide

## Chạy test

```bash
cd ros2_ws/src/ekf_adapter
python3 -m pip install pytest numpy
python3 -m pytest test -q
```

Chạy riêng một file:

```bash
python3 -m pytest test/test_ekf_mode_contract.py -q
```

Nếu import lỗi:

```bash
PYTHONPATH=ros2_ws/src/ekf_adapter python3 -m pytest ros2_ws/src/ekf_adapter/test -q
```

## Các file test

| File | Bước | Nội dung |
|---|---|---|
| `test_ekf_logic.py` | 1 | Test gốc: lõi, hình học, tracker, replay mô phỏng |
| `test_ekf_geometry_extra.py` | 1 | Quaternion, transform, công thức ma trận nhiễu R |
| `test_ekf_tracker_extra.py` | 1 | Gate động, dung sai stamp, tái bắt, kẹp vz, fuzz |
| `test_ekf_mode_contract.py` | 1, 2 | Tên mode phát ra khớp ISC/planner, bảng mode theo tuổi và phase |
| `test_ekf_scenarios.py` | 3 | Dữ liệu mô phỏng: quỹ đạo, mất marker, outlier, timeline mode |

## Kịch bản và tiêu chí pass

| Kịch bản | Tiêu chí pass |
|---|---|
| 5 quỹ đạo x 3 seed, có outlier 2% và mất marker 1 s | Sai số lọc nhỏ hơn sai số đo thô, không NaN |
| Mất marker 0.5, 1, 2, 3 s (straight, circle) | Sai số < 0.5 m sau 0.5 s kể từ lúc có lại marker |
| Outlier 0, 2, 5, 10% | Không quá một nửa outlier lọt vào filter |
| Mất marker 5 s đến 8 s (FOLLOW) | Trước: TRACKING. Sau 0.5 s mất: không TRACKING. Sau 2.2 s: EXPIRED/LOST. Có lại: TRACKING |
| Gate theo khoảng cách | Ngưỡng chấp nhận nằm trong `[gate_min_m, gate_max_m]`, không co lại khi gap tăng |
| Fuzz 5 seed x 300 mẫu ngẫu nhiên | State và covariance luôn hữu hạn, covariance đối xứng |

## Bảng mode chuẩn (theo README)

| Phase | Tuổi measurement | Mode |
|---|---|---|
| FOLLOW | <= 0.25 s (đang detect) | TRACKING |
| FOLLOW | <= 1.0 s | PREDICTING |
| FOLLOW | <= 2.0 s | PREDICTING_DEGRADED |
| FOLLOW | > 2.0 s hoặc chưa có | EXPIRED |
| APPROACH | <= 0.25 s (đang detect) | TRACKING |
| APPROACH | <= 0.5 s | PREDICTING |
| APPROACH | <= 1.0 s | PREDICTING_DEGRADED |
| APPROACH | > 1.0 s hoặc chưa có | EXPIRED |

Planner chỉ chạy khi mode là TRACKING hoặc PREDICTING. ISC coi EXPIRED là không hợp lệ.

## Khi test fail

- `test_published_names_are_in_system_vocabulary` hoặc `test_mode_matches_system_spec` fail: code đang phát `COASTING`/`LOST` thay vì bộ tên của README. Phải thống nhất tên mode giữa EKF, ISC và planner trước khi làm bước tiếp.
- Test gate fail: kiểm tra `TrackerConfig` mặc định và công thức ngân sách gate trong `ekf_logic.py`.
- `test_depth_minus_lateral_matches_model` fail: công thức R trong code lệch guide (sàn, hệ số, hoặc cách cộng nhiễu đẳng hướng).
- Test recovery fail ở dropout 3 s: tăng `candidate_window_s` hoặc kiểm tra `reacquire_frames`, vì mục tiêu có thể đã ra ngoài `gate_max_m`.

## Chưa có

- Bước 5 (test node ROS 2 `ekf_node`, `odom_tf_node`): cần source hai node.
- Bước 2 (hợp đồng EKF với ISC, planner, IBVS): làm sau khi các package đó có test.