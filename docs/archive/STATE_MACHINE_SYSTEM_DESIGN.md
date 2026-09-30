# Thiết kế chuẩn State Machine cho pipeline Drone Tracking–Avoidance

**Phiên bản:** 1.1-practical-follow-profile  
**Ngày:** 2026-09-29  
**Phạm vi:** PX4/Gazebo pipeline gồm depth camera, detector ArUco/H-Pad, EKF, IBVS, servo gimbal, APF và Offboard Commander.

> Đây là tài liệu kiến trúc làm chuẩn trước khi tiếp tục chỉnh code. Không tự ý tăng gain, giảm timeout hoặc thêm điều kiện chuyển trạng thái nếu chưa đối chiếu với sơ đồ và các invariant an toàn ở tài liệu này.

## 1. Mục tiêu

Pipeline phải bảo đảm:

1. Mất tracking thì dừng translation trước, giữ độ cao và giảm yaw có kiểm soát.
2. Detector thấy target trong SEARCH thì dừng sweep bằng một yaw setpoint cố định; không cập nhật setpoint bằng yaw thực tế từng chu kỳ.
3. Chỉ vào FOLLOW sau khi có bbox mới, EKF chấp nhận measurement và yaw đã giảm tốc.
4. Vào FOLLOW sau tái bắt bằng vận tốc nhỏ rồi ramp lên.
5. Target đứng yên không tạo yaw command lớn, đảo chiều liên tục hoặc làm mất hover.
6. Target di chuyển không bị từ chối chỉ vì bbox thay đổi nhiều pixel.
7. Chỉ một node được quyền xuất setpoint cuối cùng xuống PX4.

## 2. Phân quyền điều khiển

| Thành phần | Quyền duy nhất | Không được làm |
|---|---|---|
| Detector | raw detection, bbox, camera measurement | Không chọn phase, không điều khiển PX4 |
| EKF | accepted measurement, target state, tracking mode | Không phát velocity/yaw |
| IBVS | yêu cầu yaw và gimbal pitch | Không phát translation cuối |
| APF/IAPF | velocity ENU tránh vật cản và tới follow-goal | Không làm nguồn yaw cuối |
| Mission FSM | phase, quyền translation, setpoint cuối | Không thay EKF bằng APF |
| Offboard Commander | slew limit, ENU→NED, gửi PX4 | Không tự chọn target/phase |
| PX4 | điều khiển động lực học | Không nhận nhiều nguồn setpoint cạnh tranh |

Đường dữ liệu bắt buộc:

~~~text
Detector -> EKF -> Mission FSM -> final velocity/yaw -> Offboard -> PX4
    |          |          ^
    |          +-> APF --+
    +-> IBVS -> yaw/gimbal
~~~

APF yaw chỉ là debug. Đường yaw cuối luôn là:

~~~text
IBVS /ibvs/yaw_cmd -> FSM /mission/yaw_setpoint -> Offboard -> PX4
~~~

## 3. Ba lớp trạng thái

Không dùng một biến phase để chứa toàn bộ ý nghĩa. Thiết kế gồm:

### 3.1 Mission state

~~~text
IDLE -> TAKEOFF_HOLD -> SEARCH -> FOLLOW -> APPROACH -> LAND
                         ^          |
                         +----------+
~~~

### 3.2 Visual confidence state

~~~text
NO_MEASUREMENT -> RAW_DETECTION -> REACQUIRE_HOLD
                                      |
                                      v
                                ACCEPTED_TRACKING
                                  |      |      |
                            PREDICTING  DEGRADED EXPIRED
~~~

### 3.3 Motion authority state

~~~text
HOVER_LOCK       velocity = 0
YAW_ONLY         XY = 0, chỉ yaw/gimbal
LIMITED_FOLLOW   APF velocity nhân speed_scale nhỏ
FULL_FOLLOW      APF velocity trong giới hạn
SAFE_STOP        velocity = 0, giữ yaw/độ cao an toàn
~~~

Trong ROS hiện tại có thể giữ phase public gồm IDLE, SEARCH, FOLLOW, APPROACH, LAND. Các trạng thái con nên là biến nội bộ hoặc diagnostic topic để tránh phá interface của các node đang dùng.

## 4. Sơ đồ chuyển trạng thái

~~~mermaid
stateDiagram-v2
    [*] --> IDLE
    IDLE --> TAKEOFF_HOLD: takeoff + odom hợp lệ
    TAKEOFF_HOLD --> SEARCH: altitude đạt + hover ổn định
    TAKEOFF_HOLD --> SAFE_STOP: odom/altitude lỗi

    SEARCH --> SEARCH: chưa có detection, sweep chậm
    SEARCH --> REACQUIRE_HOLD: raw detection mới
    REACQUIRE_HOLD --> SEARCH: mất detection hoặc timeout
    REACQUIRE_HOLD --> YAW_ONLY: EKF TRACKING + yaw rate thấp
    YAW_ONLY --> FOLLOW: confirm 0.25-0.35 s
    YAW_ONLY --> SEARCH: measurement invalid

    FOLLOW --> LIMITED_FOLLOW: vừa tái bắt hoặc sai số lớn
    LIMITED_FOLLOW --> FULL_FOLLOW: yaw/bbox ổn định
    FOLLOW --> PREDICT_HOLD: measurement gap
    PREDICT_HOLD --> LIMITED_FOLLOW: tracking trở lại
    PREDICT_HOLD --> SEARCH: EXPIRED quá grace

    FOLLOW --> APPROACH: operator LAND
    APPROACH --> FOLLOW: cancel/target invalid
    APPROACH --> LAND: alignment + altitude đạt
    LAND --> IDLE: touchdown

    IDLE --> SAFE_STOP: link/clock/odom lỗi
    SEARCH --> SAFE_STOP: link/clock/odom lỗi
    FOLLOW --> SAFE_STOP: link/clock/odom lỗi
~~~

## 5. Chi tiết từng trạng thái

### 5.1 IDLE

Output:

- vx = vy = vz = 0.
- Giữ yaw hiện tại.
- Gimbal về góc an toàn đã hiệu chuẩn, không nhảy góc.
- APF không được phát translation.

Không được chuyển thẳng IDLE → FOLLOW chỉ vì raw detected = true.

### 5.2 TAKEOFF_HOLD

- Chưa cấp quyền APF.
- Giữ XY = 0 và yaw cố định.
- Độ cao do takeoff/altitude controller điều khiển.
- Hoàn tất khi altitude >= 2.5 m hoặc >= 80% takeoff altitude.
- Odom phải liên tục tối thiểu 0.5 s và attitude đã ổn định.

### 5.3 SEARCH_SWEEP

Mục đích là tìm target khi chưa có accepted measurement.

Giới hạn ban đầu:

- vx = vy = 0.
- Giữ độ cao.
- search yaw rate: khoảng 0.20 rad/s, tương đương 11.5 deg/s.
- Không dùng 0.3–0.6 rad/s trong test an toàn đầu tiên.
- Khi mới vào SEARCH, giữ yaw thực đã latch trong 0.30 s để tránh phát lệnh
  `ibvs_yaw` cũ trước khi IBVS reset theo phase mới.
- Gimbal giữ last_valid_pitch trong 0.5–1.0 s đầu sau mất tracking; sau đó sweep chậm trong envelope đã kiểm chứng.

Khi có raw detection:

- Dừng cộng search_yaw_rate ngay.
- Không để IBVS tích lũy một search command cũ.
- Chuyển sang REACQUIRE_HOLD.

### 5.4 REACQUIRE_HOLD

Đây là trạng thái bắt buộc khi detector bắt lại target trong lúc drone đang quay.

Ngay khi trigger:

1. Chụp hold_yaw = actual_yaw đúng một lần.
2. Gửi cùng hold_yaw liên tục; không gán lại bằng actual_yaw mỗi vòng.
3. Dừng APF và translation.
4. Giữ độ cao.
5. Reset hoặc tạm dừng search accumulator trong IBVS.
6. Không reset gimbal đột ngột về góc ngang.

Điều kiện xác nhận khuyến nghị:

| Điều kiện | Giá trị |
|---|---:|
| detection còn hiệu lực | liên tục hoặc mất không quá 0.75 s |
| bbox | phải có mẫu mới sau trigger; tuổi tối đa 0.35 s |
| ROI ảnh 640x480 | 40 < u < 600, 40 < v < 440 |
| EKF mode | TRACKING |
| yaw rate | abs(yaw_rate) < 8 deg/s |
| confirm duration | 0.25–0.35 s |
| hold timeout | 4.0 s cảnh báo; không tự resume sweep khi detection vẫn true |

Không dùng bbox step cố định, ví dụ dưới 35 px, làm điều kiện bắt buộc. Bbox có thể nhảy do servo settling, pixel quantization hoặc target chuyển động. Yaw rate, freshness và EKF acceptance đáng tin cậy hơn.

Nếu xác nhận thành công: sang YAW_ONLY_RECOVERY.  
Nếu detection mất ngắn: tiếp tục giữ yaw cố định trong grace 0.75 s; nếu EKF đã
ở `TRACKING` và bbox còn mới thì vẫn được xác nhận.  
Nếu detection mất quá 0.75 s: resume SEARCH sweep từ yaw hiện tại. Nếu hết 4 s
mà detection vẫn true nhưng EKF/yaw chưa đạt điều kiện, tiếp tục hover và giữ
yaw; phát cảnh báo thay vì quay lại sweep khi target vẫn nằm trong ảnh.

### 5.5 YAW_ONLY_RECOVERY

Mục đích là vừa vào lại FOLLOW nhưng chưa cho translation làm nhiễu camera.

- Thời gian khóa ban đầu: 0.35 s, giữ yaw latch từ REACQUIRE (không bám yaw
  thực từng chu kỳ).
- vx = vy = 0 trong 0.20 s đầu; sau đó chỉ mở translation qua LIMITED_FOLLOW.
- Không dùng APF để “đuổi bù” trong YAW_ONLY.
- Yaw do IBVS, rate limit khoảng 0.35–0.50 rad/s; vẫn phải qua slew/acceleration limiter.
- Gimbal do IBVS, tốc độ không vượt servo thực tế.

Sang LIMITED_FOLLOW khi:

- EKF vẫn TRACKING.
- bbox còn mới.
- abs(u-u0) < 80–100 px hoặc sai số đang giảm.
- yaw error không tăng liên tục 3–5 chu kỳ.

Nếu target còn valid nhưng chưa đạt điều kiện, giữ public phase FOLLOW nhưng
duy trì soft gate; chỉ `follow_entry_hold_time=0.20 s` đầu là XY=0. Không quay
lại SEARCH chỉ vì chưa căn giữa trong vài chu kỳ đầu.

### 5.6 LIMITED_FOLLOW

IBVS sở hữu yaw/gimbal; APF sở hữu translation; FSM chỉ cấp quyền và scale vận tốc.

| Điều kiện | speed_scale |
|---|---:|
| vừa tái bắt hoặc yaw error lớn | 0.45 ban đầu, ramp trong 0.3–0.5 s |
| abs(u-u0) > 120 px | 0.45–0.70 |
| 60 < abs(u-u0) <= 120 px | ramp tuyến tính |
| abs(u-u0) <= 60 px và yaw ổn định | 0.70–1.00 |

Với v_max = 1.2 m/s, scale 0.45 tương ứng khoảng 0.54 m/s trước
Offboard slew limiter, đủ để không chậm hơn đáng kể so với H-Pad 0.5 m/s.
Không cho speed scale giảm về 0 chỉ vì sai số ảnh lớn; khi ảnh xấu phải dừng
translation bằng điều kiện tracking/freshness, không dùng một gain ảnh quá nhạy.

Không dùng sai số v-v0 để khóa toàn bộ translation ngang; sai số dọc chủ yếu do gimbal pitch xử lý.

### 5.7 FULL_FOLLOW

Điều kiện vào:

- EKF TRACKING.
- bbox mới, không sát mép ảnh.
- yaw error dưới 7–10 deg.
- sai số ngang ổn định hoặc giảm.
- không có obstacle emergency.

Profile ban đầu:

- v_max = 1.0–1.2 m/s trong mô phỏng; khi bay thật phải xác nhận lại theo tải và
  giới hạn động lực học của airframe.
- max_accel = 1.0 m/s2 trong mô phỏng; giảm xuống nếu attitude/position log cho
  thấy overshoot, không giảm v_max trước khi kiểm tra gia tốc.
- ramp vào tốc độ đầy đủ khoảng 0.3–0.5 s.
- Không tăng k_att và v_max cùng lúc.

### 5.8a. Moving-target lead

Khi EKF cung cấp vận tốc mục tiêu hợp lệ, APF được phép dùng một lead ngắn trong
FOLLOW để giảm độ trễ đuổi theo:

- lead time mặc định: 0.25 s;
- lead bị chặn ở 0.75 m;
- vận tốc mục tiêu được lọc EMA và deadband 0.08 m/s;
- không dùng lead khi EKF không ở TRACKING hoặc khi đã vào PREDICTING_DEGRADED/EXPIRED.

Lead chỉ thay đổi follow-goal của APF, không thay đổi yaw owner, không bypass FSM
và không được phép tạo translation khi tracking không hợp lệ.

### 5.9 PREDICT_HOLD

Kích hoạt khi measurement hợp lệ mất nhưng EKF chưa EXPIRED.

| EKF mode | Translation | Yaw | Gimbal |
|---|---:|---|---|
| PREDICTING | ramp về 0 trong 0.15–0.25 s | giữ command cuối | giữ pitch cuối |
| PREDICTING_DEGRADED | 0 | giữ yaw cuối | giữ pitch cuối |
| EXPIRED | 0 | giữ yaw cuối trong grace | giữ pitch cuối |

Không cho APF tiếp tục chạy theo target prediction degraded. Nếu target quay lại trong 0.5–0.8 s, trở về LIMITED_FOLLOW, không nhảy thẳng FULL_FOLLOW.

### 5.10 SEARCH sau mất tracking dài

1. Xóa APF velocity request.
2. Xóa yaw feedforward của target cũ.
3. Đặt search command bắt đầu tại yaw thực tế.
4. Giảm search rate trong 0.5 s đầu.
5. Giữ pitch cuối trong 0.5–1.0 s rồi mới sweep.

Không dùng target state cũ để tính yaw hình học trong SEARCH.

### 5.11 APPROACH và LAND

APPROACH chỉ kích hoạt bởi operator/mission command rõ ràng.

- Chỉ descent khi EKF TRACKING hoặc measurement mới được chấp nhận.
- Mất tracking: dừng descent, giữ độ cao.
- Yaw không lấy từ APF.
- descent ban đầu: 0.2–0.3 m/s.
- LAND: khóa yaw hiện tại, translation ngang = 0, chỉ descent.
- Clock/odom/PX4 link lỗi: SAFE_STOP, không descent mù.

## 6. Quy tắc chuyển trạng thái

| Từ | Sang | Điều kiện bắt buộc | Hành động |
|---|---|---|---|
| IDLE | TAKEOFF_HOLD | takeoff + odom | zero XY, giữ yaw |
| TAKEOFF_HOLD | SEARCH | altitude + hover ổn định | reset search từ yaw hiện tại |
| SEARCH | REACQUIRE_HOLD | raw detection mới | latch yaw, zero APF |
| REACQUIRE_HOLD | YAW_ONLY | EKF TRACKING + yaw rate thấp + confirm | reset IBVS yaw |
| REACQUIRE_HOLD | SEARCH | mất detection/timeout | clear command cũ |
| YAW_ONLY | LIMITED_FOLLOW | target còn valid | APF scale thấp |
| LIMITED_FOLLOW | FULL_FOLLOW | yaw/bbox ổn định | ramp tốc độ |
| FOLLOW | PREDICT_HOLD | measurement gap | zero translation |
| PREDICT_HOLD | LIMITED_FOLLOW | measurement hợp lệ | scale thấp |
| PREDICT_HOLD | SEARCH | EXPIRED quá grace | reset target command |
| FOLLOW | APPROACH | operator LAND | giữ safety gate |
| APPROACH | LAND | alignment + altitude | khóa yaw, descent |
| LAND | IDLE | touchdown | clear requests |

Không dùng một biến duy nhất để quyết định:

- detected = true không đủ để vào FOLLOW.
- target state tồn tại không đủ để chạy APF.
- một frame detected = false không đủ để vào SEARCH.
- khoảng cách gần không đủ để descent.

## 7. Các trường hợp cần xử lý

### Case A — Mất tracking ngắn, target đứng yên

~~~text
FULL_FOLLOW
 -> PREDICT_HOLD 0.15–0.25 s
 -> accepted TRACKING
 -> LIMITED_FOLLOW 0.3–0.5 s
 -> FULL_FOLLOW
~~~

Không vào SEARCH nếu mất dưới grace period.

### Case B — Mất tracking dài

~~~text
FOLLOW
 -> PREDICT_HOLD
 -> EXPIRED
 -> SEARCH_SWEEP
 -> REACQUIRE_HOLD
 -> YAW_ONLY_RECOVERY
 -> LIMITED_FOLLOW
 -> FULL_FOLLOW
~~~

Mục tiêu thời gian từ detection đến FOLLOW: 0.35–0.80 s khi target đứng yên.
Lâu hơn 1.2 s phải kiểm tra detector/EKF hoặc freshness trước khi tăng yaw gain.

### Case C — Target di chuyển trong SEARCH

- Không dùng bbox step cố định để từ chối.
- Latch yaw ngắn.
- Chờ EKF measurement mới.
- Vào LIMITED_FOLLOW với scale khoảng 0.45, ramp lên trong 0.3–0.5 s.
- Nếu bbox về mép ảnh, giảm translation nhưng duy trì yaw/gimbal.

### Case D — Detection flicker

- Một frame false không reset ngay nếu gap <0.5–0.75 s.
- Không dùng bbox cũ quá timeout.
- Flicker kéo dài: PREDICT_HOLD rồi SEARCH.

### Case E — EKF nhảy state hoặc bị gate

- Không cho APF dùng state mới ngay.
- Giữ yaw/pitch cuối.
- Chờ 3–5 measurement hợp lệ liên tục.
- Xóa target velocity feedforward nếu jump vượt ngưỡng.

### Case F — Target gần ngay dưới drone

- FOLLOW pitch envelope có thể tới -80…-88 deg nếu servo hỗ trợ.
- Không reset SEARCH ngay về -20 deg nếu làm target ra khỏi FOV.
- Trong YAW_ONLY ưu tiên gimbal pitch, giữ XY.

### Case G — Wrap yaw qua ±pi

Mọi yaw error phải wrap:

~~~python
error = (command - actual + pi) % (2*pi) - pi
~~~

Không trừ trực tiếp hai yaw chưa wrap.

### Case H — Obstacle

APF chỉ thay đổi translation. IBVS vẫn là nguồn yaw. Nếu repulsive force làm velocity đổi dấu liên tục, giảm scale hoặc SAFE_STOP; không đổi nguồn yaw.

### Case I — Clock/TF/odom lỗi

- SAFE_STOP ngay, velocity = 0.
- Không SEARCH yaw nếu không có odom.
- Không dùng latest TF thay measurement timestamp.
- Resume sau khi clock/TF/odom ổn định ít nhất 1 s.

## 8. Giới hạn safety profile ban đầu

| Nhóm | Tham số | Giá trị ban đầu |
|---|---|---:|
| FSM | control rate | 20 Hz |
| EKF | prediction rate | 50 Hz |
| IBVS | command rate | 30 Hz |
| SEARCH | yaw rate | 0.20 rad/s (11.5 deg/s) |
| REACQUIRE | confirm time | 0.25–0.35 s |
| REACQUIRE | yaw rate threshold | 20–25 deg/s |
| REACQUIRE | hold timeout | 4.0 s |
| REACQUIRE | lost grace | 0.75 s |
| SEARCH | phase-entry yaw hold | 0.30 s |
| FOLLOW | max speed | 1.0–1.2 m/s (simulation profile) |
| FOLLOW | max accel | 1.0 m/s2 (simulation profile) |
| FOLLOW | target lead | 0.25 s, max 0.75 m |
| ALIGN | minimum APF scale | 0.45 sau khi accepted tracking |
| YAW | IBVS rate limit | 0.35–0.50 rad/s |
| ALTITUDE | follow altitude | 3.0 m |
| APPROACH | descent speed | 0.2–0.3 m/s |

Đây là profile chẩn đoán mô phỏng, không phải giới hạn bảo đảm an toàn cho bay thật.

## 9. Invariant bắt buộc

1. tracking_mode != TRACKING => APF translation cuối = 0, trừ profile prediction đã phê duyệt.
2. reacquire_active => yaw setpoint là giá trị đã latch, không phải actual_yaw cập nhật liên tục.
3. SEARCH => velocity XY = 0.
4. REACQUIRE/YAW_ONLY => velocity XY = 0 hoặc recovery profile đã định.
5. FOLLOW mới được nhận APF translation.
6. Chỉ FSM publish mission velocity/yaw.
7. Chỉ Offboard gửi setpoint xuống PX4.
8. Mỗi SEARCH -> FOLLOW phải reset search yaw accumulator và target velocity feedforward.
9. Mất clock/TF/odom ưu tiên SAFE_STOP.
10. Không tăng nhiều gain/timeout trong cùng một lần test.

## 10. Pseudocode chuẩn

~~~python
def update_fsm(inputs):
    if not inputs.safe_io:
        return SAFE_STOP

    if state == IDLE:
        return TAKEOFF_HOLD if takeoff_ready(inputs) else IDLE

    if state == TAKEOFF_HOLD:
        return SEARCH if hover_ready(inputs) else TAKEOFF_HOLD

    if state == SEARCH:
        if fresh_raw_detection(inputs):
            hold_yaw = inputs.actual_yaw       # latch một lần
            stop_apf()
            reset_ibvs_search_accumulator()
            return REACQUIRE_HOLD
        return SEARCH_SWEEP_SLOW

    if state == REACQUIRE_HOLD:
        publish_fixed_yaw(hold_yaw)
        stop_xy()
        hold_altitude()

        if detection_lost_too_long(inputs):
            return SEARCH_SWEEP_SLOW
        if reacquire_timeout(inputs):
            return SEARCH_SWEEP_SLOW
        if accepted_tracking(inputs) and yaw_rate_low(inputs):
            if stable_timer >= CONFIRM_TIME:
                reset_ibvs_yaw_to_actual()
                return YAW_ONLY_RECOVERY
        return REACQUIRE_HOLD

    if state in (YAW_ONLY_RECOVERY, LIMITED_FOLLOW, FULL_FOLLOW):
        if accepted_tracking(inputs):
            scale = visual_alignment_scale(inputs)
            return next_follow_substate(inputs), apf_velocity(inputs) * scale
        return PREDICT_HOLD

    if state == PREDICT_HOLD:
        stop_xy()
        hold_last_yaw_and_pitch()
        if accepted_tracking(inputs):
            return LIMITED_FOLLOW
        if expired_beyond_grace(inputs):
            clear_old_target_commands()
            return SEARCH_SWEEP_SLOW
        return PREDICT_HOLD
~~~

## 11. Kế hoạch triển khai

### Bước 0 — Baseline

Ghi đồng thời:

~~~text
/clock
/odom
/hpad/detected
/hpad/bbox
/ekf/tracking_mode
/ekf/target_state
/ibvs/yaw_cmd
/mission/yaw_setpoint
/mission/velocity_setpoint
/joint_states
~~~

Xác nhận không có duplicate node/publisher.

### Bước 1 — SEARCH/REACQUIRE chỉ yaw

- Không bật APF translation.
- Kiểm tra yaw setpoint cố định sau detection.
- Đo yaw rate, overshoot và thời gian dừng.
- Target đứng yên trước.

### Bước 2 — YAW_ONLY_RECOVERY

- Translation = 0 trong 0.20 s đầu.
- PASS nếu vào recovery <= 0.8 s và không quay lại SEARCH trong 5 s.

### Bước 3 — LIMITED_FOLLOW

- v_max = 1.0–1.2 m/s.
- scale bắt đầu khoảng 0.45, ramp 0.3–0.5 s; không vượt giới hạn gia tốc.
- PASS nếu target còn trong FOV, attitude không giật.

### Bước 4 — FULL_FOLLOW target đứng yên

- Tăng v_max từng bước 0.8 -> 1.0 -> 1.2 m/s.
- Chỉ đổi một tham số mỗi lần; giữ max_accel = 1.0 m/s2 trong profile mô phỏng.
- PASS nếu không vào SEARCH ngoài mất detection thật.

### Bước 5 — Target di chuyển

- Bắt đầu tốc độ thấp.
- Kiểm tra EKF velocity, bbox, yaw và APF riêng.
- Chỉ sau khi pass mới tăng tốc target.

### Bước 6 — Obstacle

- Giữ nguyên tracking profile đã pass.
- Thêm obstacle từng loại.
- APF chỉ thay đổi translation, không đổi nguồn yaw.

## 12. Tiêu chí PASS/FAIL

### PASS

- Detection lại -> yaw setpoint cố định trong <= 50 ms.
- Overshoot mô phỏng < 5–10 deg.
- Reacquire -> FOLLOW trong 0.35–0.80 s khi target đứng yên.
- Không có yaw command tích lũy tiếp sau REACQUIRE_HOLD.
- Không reversal do search command cũ lớn hơn 10 deg.
- SEARCH/REACQUIRE không có translation.
- FOLLOW có velocity ramp.
- Target mất thật mới vào SEARCH.

### FAIL nghiêm trọng

- Raw detection làm drone quay mạnh hơn.
- yaw_setpoint bám actual_yaw trong REACQUIRE.
- APF chạy khi EKF EXPIRED.
- Có hơn một nguồn yaw cuối.
- Target đứng yên nhưng yaw command đổi liên tục > 10 deg/s.
- Mất tracking làm roll/pitch tăng mạnh hoặc mất altitude.

## 13. Checklist trước mỗi lần test

- [ ] Chỉ có một PX4/Gazebo instance.
- [ ] Không còn Python pipeline node cũ.
- [ ] /clock chạy, mọi node use_sim_time=true.
- [ ] TF camera -> world đúng timestamp.
- [ ] Odom ổn định trước arm.
- [ ] /mission/phase chỉ có một publisher.
- [ ] /mission/yaw_setpoint chỉ do FSM xuất.
- [ ] /apf/yaw_cmd không được Offboard subscribe.
- [ ] SEARCH có velocity XY = 0.
- [ ] REACQUIRE có yaw latch cố định.
- [ ] FOLLOW có speed scale và ramp.
- [ ] Bag recorder chạy trước takeoff.
- [ ] Có land/stop thủ công.

## 14. Kết luận

Chuỗi xử lý chuẩn phải là:

~~~text
Mất tracking
 -> giảm translation
 -> giữ yaw/pitch cuối
 -> EXPIRED
 -> SEARCH chậm
 -> raw detection
 -> latch yaw cố định
 -> accepted EKF + yaw settled
 -> YAW_ONLY_RECOVERY
 -> LIMITED_FOLLOW
 -> FULL_FOLLOW
~~~

Mọi thay đổi code sau này phải chỉ rõ nó tác động vào bước nào, node nào là owner, timeout nào thay đổi và invariant an toàn nào vẫn được giữ nguyên.
