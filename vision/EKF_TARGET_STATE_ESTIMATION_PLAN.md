# Kế hoạch thiết kế EKF ước lượng trạng thái H-Pad/ArUco

> **Phạm vi:** Vision/Estimation — mục tiêu là H-Pad di động mang ArUco marker.  
> **Đầu ra cần có:** vị trí, vận tốc, hiệp phương sai và trạng thái tin cậy của mục tiêu để FSM, APF và điều khiển gimbal sử dụng.  
> **Cập nhật:** 07/09/2026.

## 1. Bối cảnh và quyết định thiết kế

Nhiệm vụ của dự án là bám H-Pad di động, tránh vật cản và hạ cánh chính xác bằng một RealSense D435(i) RGB-D đặt trên gimbal pitch chủ động. Pipeline hiện hữu đã đáp ứng phần đo:

- `vision/realsense_stream.py` lấy RGB, depth **đã align về RGB**, cùng nội tại `K` và distortion từ SDK;
- `vision/aruco_detector.py` phát hiện ArUco, tinh chỉnh góc bằng sub-pixel và dùng `solvePnP(..., SOLVEPNP_IPPE_SQUARE)` để xuất `tvec = [x_c,y_c,z_c]` (m), orientation, corner và BBox;
- `vision/depth_masker.py` giãn polygon/BBox 15% rồi loại vùng H-Pad khỏi depth để APF không xem chính đích bám là vật cản.

Vì vậy bộ lọc **không thay thế ArUco/PnP**. Nó biến chuỗi phép đo PnP nhiễu, không đều thành trạng thái mục tiêu nhất quán trong hệ thế giới và tiếp tục dự đoán ngắn hạn khi không có marker.

Đề xuất trong thiết kế IBVS và yêu cầu mới được đưa vào ba hành vi sau:

1. **Short-term dead reckoning:** khi mất ArUco, dự đoán trạng thái 6 biến trong tối đa **1.5 s** để APF và gimbal không ngắt đột ngột.
2. **Active gimbal reacquisition:** chiếu vị trí dự đoán 3D và bất định xuống ảnh; dùng tâm/ROI dự đoán để đặt pitch gimbal hướng về vùng đó và ưu tiên tìm lại marker.
3. **An toàn/FSM:** quá `T_search` (cấu hình 3 s trong `APPROACH`, 5 s trong `FOLLOW`; có thể giảm theo flight-test), estimator không còn được coi là nguồn dẫn đường hợp lệ; FSM hover an toàn rồi chuyển `SEARCH` thay vì bám mù.

### Lưu ý về tên gọi EKF

Mô hình vận tốc hằng số sau khi phép đo được đổi sang `map` có `F` và `H` tuyến tính, nên lõi toán học là **Kalman Filter tuyến tính**. Tên lớp có thể giữ là `TargetStateEKF` để phù hợp kiến trúc dự án, bởi toàn hệ có phần phi tuyến: biến đổi camera–gimbal–drone–world, nội suy pose và chiếu 3D→2D. Không nên phức tạp hóa lõi bằng EKF nếu chưa cập nhật trực tiếp pixel/depth; việc đó giúp dễ tune và kiểm thử hơn.

## 2. Hợp đồng hệ tọa độ, thời gian và giao diện

Đây là điều kiện tiên quyết; không được trộn ENU với PX4 Local NED trong estimator.

| Thành phần | Hệ đề xuất | Quy ước / dữ liệu cần có |
|---|---|---|
| Bộ lọc, APF phía ROS | `map` ENU | mét; x-East, y-North, z-Up (hoặc quy ước ROS đã chọn, nhưng phải cố định toàn bộ) |
| Thân drone | `base_link` | pose từ odometry, nội suy đúng `stamp` ảnh |
| Camera | `camera_color_optical_frame` | chuẩn optical: x sang phải, y xuống, z hướng ra trước |
| Gimbal | `gimbal_link` | pitch thực đo/ước lượng, có offset cơ khí và giới hạn góc |
| PX4 | Local NED | chỉ đổi ENU↔NED tại node Offboard, không đổi bên trong EKF |

Chuỗi biến đổi cho một phép đo là:

\[
{}^{map}\mathbf p_{h}^{meas}(t)=
{}^{map}\mathbf T_{base}(t)
{}^{base}\mathbf T_{gimbal}(\theta_g(t))
{}^{gimbal}\mathbf T_{camera}
{}^{camera}\mathbf p_{h}^{PnP}(t).
\]

`T_base_gimbal` và `T_gimbal_camera` phải được hiệu chuẩn. Dùng **timestamp của RGB frame**; odometry và góc/feedback servo được nội suy hoặc lấy mẫu gần nhất với sai số thời gian được log. Không dùng pose drone “mới nhất” cho một ảnh cũ, vì lúc drone/gimbal quay sẽ tạo sai số giả thành vận tốc mục tiêu.

Giao diện ROS 2 tối thiểu đề xuất:

| Kênh | Nội dung |
|---|---|
| `/hpad/aruco_measurement` | marker ID, `PoseWithCovarianceStamped` trong camera frame, corners, reprojection RMS, `z_pnp`, `z_depth`, `detected` |
| `/drone/odom`, `/gimbal/state` | pose/twist drone và pitch có timestamp |
| `/hpad/state_filtered` | `PoseWithCovarianceStamped`, `TwistWithCovariance`, `mode`, `age_since_measurement`, `valid_for_control` |
| `/hpad/predicted_roi` | tâm pixel dự đoán, ellipse/ROI, depth dự đoán, covariance 2D để detector/gimbal dùng |
| `/hpad/estimator_diagnostics` | innovation, NIS, phép đo bị loại, latency, số frame mất dấu |

`mode` nên là `UNINITIALIZED`, `TRACKING`, `PREDICTING`, `PREDICTING_DEGRADED`, `EXPIRED`. Chỉ `TRACKING` và `PREDICTING` với `age <= 1.5 s` được cấp cho planner; hai mode sau chỉ phục vụ tìm lại/quan sát và không được dùng dẫn đường.

## 3. Thuật toán ước lượng trạng thái

### 3.1 Trạng thái, mô hình và nhiễu

Trạng thái H-Pad trong `map`:

\[
\mathbf x=[p_x,p_y,p_z,v_x,v_y,v_z]^T.
\]

Với `dt` thực tế giữa hai lần `predict`, dùng Constant Velocity (CV):

\[
\mathbf x_k^- =
\begin{bmatrix}\mathbf I_3&dt\mathbf I_3\\\mathbf0_3&\mathbf I_3\end{bmatrix}
\mathbf x_{k-1}, \qquad
\mathbf P_k^- = \mathbf F\mathbf P_{k-1}\mathbf F^T+\mathbf Q(dt).
\]

Nhiễu quá trình là white acceleration độc lập theo ba trục, tốt hơn `Q` hằng số theo frame-rate:

\[
\mathbf Q(dt)=
\begin{bmatrix}
dt^3/3\,\mathbf Q_a&dt^2/2\,\mathbf Q_a\\
dt^2/2\,\mathbf Q_a&dt\,\mathbf Q_a
\end{bmatrix},\quad
\mathbf Q_a=\operatorname{diag}(q_{ax},q_{ay},q_{az}).
\]

Khởi tạo `q_a` từ log gia tốc/đổi hướng thật của xe mang H-Pad, rồi tune bằng validation; không đóng đinh số liệu mô phỏng thành thông số bay thật. `dt` bị giới hạn hợp lý (ví dụ 1–100 ms); mất frame dài vẫn gọi predict, nhưng phải tăng bất định theo `Q(dt)` và kiểm tra timeout.

### 3.2 Phép đo và cập nhật

Sau biến đổi PnP sang `map`, phép đo vị trí là:

\[
\mathbf z_k=[p_x^{meas},p_y^{meas},p_z^{meas}]^T,
\qquad \mathbf H=[\mathbf I_3\ \mathbf0_3].
\]

Thực hiện predict ở mọi tick. Nếu một measurement hợp lệ đến, thực hiện update chuẩn với Joseph form để giữ `P` đối xứng dương bán xác định:

\[
\mathbf K=\mathbf P^-\mathbf H^T(\mathbf H\mathbf P^-\mathbf H^T+\mathbf R)^{-1},\quad
\mathbf x=\mathbf x^-+\mathbf K(\mathbf z-\mathbf H\mathbf x^-),
\]

\[
\mathbf P=(\mathbf I-\mathbf K\mathbf H)\mathbf P^-(\mathbf I-\mathbf K\mathbf H)^T+\mathbf K\mathbf R\mathbf K^T.
\]

`R` không nên cố định đơn giản. Tạo `R_pnp` theo các yếu tố đo được:

- RMS reprojection của `solvePnP`, số pixel/cạnh marker và góc nghiêng marker (marker nhỏ, xa, xiên → `R` lớn);
- chênh lệch `|z_pnp-z_depth|` sau khi lấy median depth hợp lệ quanh tâm/corners (depth chỉ là kiểm tra chất lượng ở phiên bản đầu, **không fuse trực tiếp** để tránh tương quan/nhiễu chưa mô hình hóa);
- covariance odometry, sai số extrinsic camera–gimbal, và sai số độ trễ đồng bộ. Chúng được cộng/bảo thủ hóa vào `R` trong `map`.

Khởi tạo sau `N_init = 10` detection liên tiếp theo FSM. `p` ban đầu là median/mean có trọng số của các measurement đã qua gate; `v` bằng 0 hoặc sai phân sau một cửa sổ ngắn, với phương sai vận tốc lớn. Không khởi tạo từ một detection đơn lẻ.

### 3.3 Kiểm tra chất lượng và loại outlier

Một frame chỉ được update khi tất cả điều kiện sau đạt:

1. Đúng `marker_id` của H-Pad, PnP thành công, `z_c > 0`, marker còn trong ảnh và size vật lý đúng.
2. Reprojection error và chênh lệch PnP–depth dưới ngưỡng đã hiệu chuẩn; depth invalid thì đánh dấu nhưng không tự loại PnP nếu các điều kiện khác tốt.
3. Innovation được gate theo Mahalanobis/NIS:
   \(d^2=\mathbf r^T\mathbf S^{-1}\mathbf r\), \(\mathbf r=\mathbf z-\mathbf H\mathbf x^-\). Khởi điểm dùng ngưỡng chi-square 3D 99.7% (`16.27`), sau đó xác nhận lại bằng phân bố NIS thực tế.
4. Không có timestamp lùi và transform tại timestamp ảnh tồn tại.

Phép đo trượt gate bị loại, đếm trong diagnostics và **không reset** trạng thái. Một chuỗi nhiều outlier hoặc `P` tăng quá giới hạn sẽ làm `valid_for_control=false`. Cần lưu cả raw measurement để điều tra, không chỉ lưu kết quả đã lọc.

### 3.4 Mất dấu, dự đoán và tìm lại marker

| Thời gian từ phép đo hợp lệ cuối | Mode | Hành vi |
|---:|---|---|
| 0–1.5 s | `PREDICTING` | CV predict; xuất state + covariance; planner giảm độ tin cậy/không tăng tốc đột ngột |
| 1.5 s–`T_search` | `PREDICTING_DEGRADED` | chỉ giữ hover hoặc policy an toàn của FSM; vẫn dùng vị trí dự đoán để quét gimbal tìm lại marker |
| >3 s `APPROACH`, >5 s `FOLLOW` | `EXPIRED` | FSM chuyển FOLLOW hoặc SEARCH theo quy tắc hiện có; không đưa pose dự đoán vào lệnh hạ cánh/bám |

Mỗi tick mất dấu, biến đổi mean và covariance vị trí dự đoán từ `map` về camera ở timestamp hiện tại:

\[
{}^c\hat{\mathbf p}={}^c\mathbf T_{map}\,[{}^{map}\hat{\mathbf p};1],
\qquad
\mathbf\Sigma_{uv}=\mathbf J_\pi\,\mathbf R_{cm}\mathbf\Sigma_p\mathbf R_{cm}^T\mathbf J_\pi^T,
\]

với phép chiếu `u = f_x x_c/z_c+c_x`, `v = f_y y_c/z_c+c_y`; `J_pi` là Jacobian của phép chiếu. Covariance pose drone/gimbal được cộng vào trước khi chiếu. Từ eigenvalue/eigenvector của `Σ_uv` dựng ellipse ROI; lấy `alpha_roi` ban đầu 3 và sweep 2–5 khi đánh giá. ROI bị clip theo ảnh và bị vô hiệu nếu `z_c <= 0` hoặc quá FOV.

Gimbal nhận **vị trí 3D dự đoán**, không chỉ một pixel: tính góc nhìn từ transform camera thực tế, đặt pitch về hướng giảm sai số ngắm, sau đó áp dụng giới hạn góc, rate limiter và EMA/PWM theo `IBVS_Implementation_Guide.md`. Dấu pitch phải được xác nhận bằng thử nghiệm bench vì phụ thuộc chiều lắp servo. Khi ArUco trở lại, detector chạy full-frame hoặc ưu tiên ROI, nhưng vẫn phải chạy kiểm tra ID/quality/gate trước update — ROI không được biến thành “đo giả”.

## 4. Nguồn dữ liệu kiểm nghiệm và cách dùng

| Nguồn | Có gì | Dùng cho | Giới hạn |
|---|---|---|---|
| **Dữ liệu mô phỏng tự sinh** | trajectory 3D target, pose drone/gimbal, timestamp, nhiễu PnP, dropout có nhãn ground truth | unit/integration, Monte Carlo, tune `Q/R`, tái lập test | phải mô phỏng đúng latency và bias thực tế |
| **PX4 SITL + Gazebo hiện có** (`simulation_maps/`) | odometry drone, point cloud/depth, map obstacle; bổ sung model H-Pad động và topic ground truth của nó | kiểm thử ROS 2 end-to-end, latency, FSM, gimbal/FOV và an toàn | cần bổ sung H-Pad/ArUco có chuyển động; Gazebo không thay thế nhiễu camera thật |
| **RealSense D435(i) + H-Pad thật** | RGB, depth align, PnP và angle servo thật | đo `R`, kiểm tra calibration, blur/ánh sáng/occlusion | chưa có ground truth tuyệt đối nếu chỉ có camera |
| **Vicon/OptiTrack hoặc motion-capture cục bộ** | trajectory 6DoF ground truth H-Pad và drone đồng bộ | đánh giá RMSE cuối cùng, calibration và flight-test | lựa chọn ưu tiên cho KPI định lượng ngoài SITL |
| **DPJAIT** — [Zenodo 10800806](https://doi.org/10.5281/zenodo.10800806) | real + simulated RGB, Vicon ground truth, intrinsic/extrinsic, các sequence FPV/ArUco | offline replay của transform, đồng bộ timestamp, CV filter và metrics có ground truth | mục tiêu là drone/ArUco bố trí cảnh, **không phải H-Pad động cùng pipeline D435**; dùng benchmark bổ sung, không dùng thay flight test |

Nguồn DPJAIT được ghi ngay trong `vision/Multimodal dataset for indoor 3D for drone tracking.pdf`; dữ liệu có real/simulation, camera calibration, ArUco và Vicon reference. Tài liệu `SMART-TRACK ...pdf` trong cùng thư mục là căn cứ cho cơ chế chiếu mean/covariance thành ROI để reacquisition, không phải một dataset đầu vào.

Mỗi recorder cần tạo ROS bag/CSV có schema tối thiểu:

```text
stamp_ns, run_id, frame_id,
target_gt_map[x,y,z,vx,vy,vz] (nếu có),
aruco_detected, pnp_camera[x,y,z], reproj_rms, z_depth,
drone_pose_map, drone_twist, gimbal_pitch,
state_pred_map, state_filt_map, P_diag, nis, accepted,
roi[u,v,major,minor,angle], estimator_mode, cpu_ms
```

Ground truth và estimate phải được nội suy về **cùng timestamp** trước khi tính lỗi; không so sample 30 Hz với sample mocap 100 Hz theo index.

## 5. Các bước triển khai chi tiết

1. **Chốt calibration và TF.** Đo marker side length, xác định ID H-Pad; lấy `K,D` thật từ RealSense. Hiệu chuẩn `base→gimbal→camera`, offset/chiều pitch servo; phát `tf2` tĩnh/động và một test đặt marker tại điểm đã biết. Chọn một quy ước `map` ENU và viết converter duy nhất ở biên PX4.
2. **Chuẩn hóa measurement.** Mở rộng kết quả `ArUcoDetector` thành message có `stamp`, `frame_id`, `tvec`, corner, marker ID, reprojection RMS và `z_depth` median. Chỉ publish H-Pad ID; depth masking vẫn nhận detection cần mask. Đo latency capture→PnP và gắn timestamp nguồn, không dùng thời gian publish.
3. **Tạo lõi độc lập ROS.** Thêm `vision/target_state_ekf.py` với `initialize`, `predict(stamp)`, `try_update(measurement)`, `project_to_camera(...)` và snapshot không mutable. Thêm `vision/measurement_quality.py` cho `R`, depth check, NIS gate. Lõi nhận NumPy/dataclass để test không cần camera/ROS.
4. **Kiểm thử đơn vị và property test.** Tạo `tests/test_target_state_ekf.py`: ma trận `F/Q`, predict chính xác với vận tốc hằng, update giảm trace(P), Joseph form giữ PSD, gate loại outlier, covariance tăng khi dropout, và projection 3D→pixel/ROI ở các góc ảnh.
5. **Viết generator/replay offline.** Tạo trajectory thẳng, circle/figure-eight, ziczac, stop–go và đổi hướng; velocity không quá 5 m/s. Tiêm nhiễu đổi theo range, bias, jitter/latency, dropout 0.5/1/1.5/3/5 s và pose drone ±5°. Xuất cùng schema log để cùng một evaluator chạy cho mô phỏng, SITL và real.
6. **Tune theo dữ liệu.** Chia sequence thành tune/validation theo từng trajectory, không tune trên sequence dùng báo cáo. Ước lượng `R` theo bin khoảng cách/góc nghiêng và quét `q_a`, ngưỡng NIS, `alpha_roi`; kiểm tra NIS/NEES để covariance không quá tự tin.
7. **Bọc ROS 2 node.** Node dùng buffer odometry/gimbal, transform đúng timestamp; publish state, ROI, diagnostics. Khi `PREDICTING`, publish intent gimbal/reacquisition; khi expired, phát event cho FSM, không tự quyết mode bay.
8. **Tích hợp SITL.** Bổ sung H-Pad/ArUco động có ground-truth topic vào world, replay các kịch bản có obstacle; kiểm tra APF nhận filtered state còn depth vẫn đã mask H-Pad. Sau đó chạy phần cứng trên ground/treo tether trước flight.
9. **Chốt an toàn flight test.** Gắn giới hạn `T_predict`, `T_search`, covariance maximum, velocity/yaw/gimbal rate limit và kill/RC override. Chỉ cho `LAND` dựa trên measurement mới/`TRACKING`, không dựa riêng on predicted state.

## 6. Kế hoạch kiểm nghiệm và tiêu chí đánh giá

### Tầng A — xác minh toán học và chất lượng phần mềm

- Deterministic test: không nhiễu, CV đúng → position/velocity phải khớp tolerance số học.
- Outlier test: một PnP nhảy 2–5 m phải bị NIS gate loại; state không nhảy theo measurement đó.
- Dropout test: không update → `trace(P)` tăng, mode đổi đúng 1.5 s và expiration đúng ngưỡng FSM.
- Transform/projection test: target trước camera ra pixel hợp lệ; target sau camera không tạo ROI; đổi yaw/pitch drone đúng hướng projected pixel.
- Regression: chạy `python3 -m unittest discover -s tests` cùng với test ArUco/masking hiện có.

### Tầng B — mô phỏng có ground truth (Monte Carlo)

Mỗi trajectory chạy tối thiểu 30 seed, với 5 nhóm: tuyến tính CV, sinusoid/figure-eight, stop–go, đổi hướng mạnh và tốc độ tới 5 m/s. Mỗi nhóm thêm: Gaussian noise, 1–3% outlier, image/odom jitter, occlusion từng đoạn 0.5–5 s và gimbal/drone attitude perturbation. So sánh ba baseline:

1. raw PnP (không filter);
2. CV KF/EKF chỉ predict khi lost;
3. CV KF/EKF + covariance-projected ROI + active reacquisition.

Báo cáo median, mean, P95 và 95% CI theo seed; không chỉ chọn một run đẹp.

### Tầng C — SITL và hardware-in-the-loop

- **SITL:** chạy `SEARCH → FOLLOW → APPROACH → LAND` với H-Pad động, obstacle và che ArUco nhân tạo. Thu bag cả state estimator, camera, odometry, gimbal, FSM, command và truth. Kiểm tra mất dấu ở follow/approach dẫn về state an toàn đúng timeout.
- **Bench RealSense:** dịch H-Pad trên ray/xe đẩy với các cự ly, ánh sáng, tilt, blur; dùng thước/encoder hoặc Vicon/OptiTrack. Đánh giá sai số PnP và xác định bảng `R(range, angle)`.
- **Flight test tăng dần:** hover + target tĩnh → target chậm không obstacle → occlusion có kiểm soát → follow động → approach. Mỗi mức chỉ đi tiếp khi safety and quality gate đạt.

### Metric, biểu đồ và ngưỡng chấp nhận

| Metric | Cách đo | Mục tiêu ban đầu |
|---|---|---|
| Position RMSE/MAE/P95 | `||p_est(t)-p_gt(t)||` sau đồng bộ thời gian | RMSE FOLLOW < **1.0 m** theo KPI dự án; báo cáo riêng theo range/occlusion |
| Velocity RMSE | `||v_est-v_gt||`, truth đã lọc/nội suy phù hợp | dùng để tune, chưa đặt ngưỡng trước khi đo động học H-Pad thật |
| Lỗi trong và sau occlusion | max/RMSE ở 0–1.5 s lost; thời gian hồi phục sau detection | không divergence; state chỉ dùng control trong cửa sổ cho phép |
| Reacquisition | tỷ lệ tìm lại marker, latency từ loss đến detection, % ROI chứa marker truth | so với full-frame; không được làm giảm safety/FPS |
| NIS/NEES consistency | distribution innovation/state error so với `P,R` | phần lớn nằm trong confidence đã chọn; dùng để chỉnh `Q/R` |
| Gimbal/FOV | pixel error, thời gian target ở FOV, rate/pitch saturation | theo KPI IBVS: `|e_u|,|e_v| < 40 px` trong điều kiện công bố |
| Real-time | `cpu_ms` P50/P95, end-to-end timestamp | estimator + projection hướng tới <2 ms; toàn chuỗi sensor→command <500 ms |
| Safe fallback | log transition/command sau lost dài | 100% run chuyển đúng mode; không LAND bằng state `EXPIRED` |

Biểu đồ bắt buộc của mỗi run: trajectory XY/XYZ (truth, raw, filtered), error theo thời gian với vùng occlusion, `±3σ` position, NIS/gate decision, `trace(P)`, pixel/ROI trên frame, gimbal pitch, latency và timeline mode FSM. Một run được coi thành công không chỉ vì RMSE thấp: transform, covariance consistency, timeout và safe fallback đều phải đúng.

## 7. Deliverable theo thứ tự

1. `target_state_ekf.py`, quality/transform utilities và unit tests pass.
2. Generator + evaluator + dữ liệu replay có truth, báo cáo baseline/Monte Carlo.
3. ROS 2 estimator node với diagnostics, state và predicted ROI.
4. SITL H-Pad moving + rosbag + báo cáo transition safety.
5. Calibration/flight-test log có ground truth (mocap nếu khả dụng), bảng tune `Q/R` và báo cáo KPI cuối.

Mọi thông số (`q_a`, `R` theo range, gate, timeouts, ROI scale, limits) phải nằm trong YAML có version và được lưu cùng rosbag/CSV của mỗi run để kết quả tái lập được.
