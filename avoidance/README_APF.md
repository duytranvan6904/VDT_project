# VDT_project
 Hệ thống bám mục tiêu động thời gian thực và tránh vật cản tự động cho Quadrotors sử dụng ROS 2, PX4 Autopilot, YOLO + ByteTrack, EKF và Improved APF.
# APF Planner for 1 Quadrotor

Hàm `APFplanner_1_Obstacle` sử dụng phương pháp **Artificial Potential Field (APF)** để tạo lệnh vận tốc cho UAV 2D, dựa trên vị trí hiện tại, vị trí đích và vị trí các vật cản.

## Cú pháp

```matlab
[V_cmd, yaw_cmd, Stop] = APF_Planner(pos, goal, obs, numobs)
```

## Đầu vào

| Tên | Kích thước | Mô tả | Đơn vị |
|---|---|---|---|
| `pos` | `1×3` | Vị trí hiện tại của UAV `[N E D]` | m |
| `goal` | `1×3` | Vị trí đích `[N E D]` | m |
| `obs` | `N×3` | Tọa độ các vật cản `[N E D]` | m |
| `numobs` | `1×1` | Số lượng vật cản được xét | - |

Trong đó:

- `N`: số lượng vật cản.
- Hệ tọa độ sử dụng là **NED (North-East-Down)**.
- `obs(i,:)` là tọa độ của vật cản thứ `i`.

Ví dụ:

```matlab
pos = [0 0 0];
goal = [20 10 -5];

obs = [
    5  2  0;
    10 5 -2;
    15 8 -3
];

numobs = 3;
```

## Đầu ra

| Tên | Kích thước | Mô tả | Đơn vị |
|---|---|---|---|
| `V_cmd` | `1×3` | Lệnh vận tốc `[VN VE VD]` | m/s |
| `yaw_cmd` | `1×1` | Góc yaw mong muốn | rad |
| `Stop` | `1×1` | Cờ báo UAV đã đến đích | - |

### `V_cmd`

```text
V_cmd = [VN VE VD]
```

Trong đó:

- `VN`: vận tốc theo hướng North
- `VE`: vận tốc theo hướng East
- `VD`: vận tốc theo hướng Down

Giá trị vận tốc được giới hạn bởi:

```matlab
v_max = 2;   % m/s
```

### `yaw_cmd`

Góc yaw được tính theo hướng chuyển động ngang của UAV:

```matlab
yaw_cmd = atan2(VE, VN);
```

### `Stop`

```text
Stop = 0  → UAV chưa đến đích
Stop = 1  → UAV đã đến đích
```

UAV được xem là đến đích khi khoảng cách tới goal nhỏ hơn:

```matlab
0.15 m
```

## Tham số chính

```matlab
d0   = 5;       % Khoảng cách ảnh hưởng của vật cản (m)
v_max = 2;      % Vận tốc tối đa (m/s)
katt = 10;      % Hệ số lực hút
krep = 90000;   % Hệ số lực đẩy
```

Vật cản chỉ tạo lực đẩy khi:

```text
distance ≤ d0
```

Tóm lại:

```text
Input:
pos    [1×3]
goal   [1×3]
obs    [N×3]
numobs [1×1]

        ↓

     APF Planner

        ↓

Output:
V_cmd   [1×3]
yaw_cmd [1×1]
Stop    [1×1]
```
# IAPF Planner for UAV 3D

Hàm `IAPF_Planner_3D_MultiObs` triển khai thuật toán **Improved Artificial Potential Field (IAPF)** để tạo lệnh vận tốc cho UAV trong không gian 3D.

Thuật toán gồm ba phần chính:

- Lực hút đưa UAV về phía goal.
- Lực đẩy giúp UAV tránh vật cản và giảm hiện tượng goal nằm gần obstacle nhưng UAV không tới được đích.
- Cơ chế xử lý local minimum và oscillation để UAV thoát vùng cân bằng cục bộ và giảm đổi hướng liên tục khi bay gần vật cản.

## Cú pháp

```matlab
[V_cmd, yaw_cmd, Stop] = IAPF_Planner_3D_MultiObs(pos, goal, obs, katt, krep, d0, v_max, numobs)
```

## Đầu vào

| Tên | Kích thước | Ý nghĩa | Đơn vị |
|---|---:|---|---|
| `pos` | `1×3` | Vị trí hiện tại của UAV `[N E D]` | m |
| `goal` | `1×3` | Vị trí đích `[N E D]` | m |
| `obs` | `N×3` | Tọa độ các vật cản `[N E D]` | m |
| `katt` | `1×1` | Hệ số lực hút | - |
| `krep` | `1×1` | Hệ số lực đẩy | - |
| `d0` | `1×1` | Khoảng cách ảnh hưởng của vật cản | m |
| `v_max` | `1×1` | Vận tốc cực đại của UAV | m/s |
| `numobs` | `1×1` | Số lượng vật cản được xét | - |

Trong đó:

- `N` là số lượng vật cản.
- `obs(i,:)` là tọa độ của vật cản thứ `i`.
- Hệ tọa độ sử dụng là **NED (North-East-Down)**.

Ví dụ:

```matlab
pos  = [0 0 0];
goal = [20 10 -5];

obs = [
     5  2  0;
    10  5 -2;
    15  8 -3
];

katt   = 10;
krep   = 100;
d0     = 5;
v_max  = 2;
numobs = 3;
```

## Đầu ra

| Tên | Kích thước | Ý nghĩa | Đơn vị |
|---|---:|---|---|
| `V_cmd` | `1×3` | Lệnh vận tốc `[VN VE VD]` | m/s |
| `yaw_cmd` | `1×1` | Góc yaw mong muốn | rad |
| `Stop` | `1×1` | Cờ báo UAV đã tới goal | - |

### `V_cmd`

```text
V_cmd = [VN VE VD]
```

Trong đó:

- `VN`: vận tốc theo hướng North.
- `VE`: vận tốc theo hướng East.
- `VD`: vận tốc theo hướng Down.

### `yaw_cmd`

Góc yaw được tính theo hướng chuyển động ngang của UAV:

```matlab
yaw_cmd = atan2(V_cmd(2), V_cmd(1));
```

### `Stop`

```text
Stop = 0  -> UAV chưa tới goal
Stop = 1  -> UAV đã tới goal
```

---

## Mô tả thuật toán

### 1. Tính khoảng cách và hướng tới goal

Từ vị trí hiện tại `pos` và vị trí đích `goal`, thuật toán xác định:

- Khoảng cách từ UAV tới goal.
- Vector hướng từ UAV tới goal.
- Vector đơn vị `dir_goal` dùng để xác định hướng di chuyển mong muốn.

Nếu khoảng cách tới goal nhỏ hơn ngưỡng cho trước, UAV được xem là đã tới đích và `Stop = 1`.

---

### 2. Tính Attractive Force

Attractive force có nhiệm vụ kéo UAV về phía goal.

Độ lớn lực hút phụ thuộc vào:

- Khoảng cách từ UAV tới goal.
- Hệ số `katt`.

Khi UAV ở xa goal, lực hút lớn hơn. Khi UAV tiến gần goal, lực hút giảm dần.

---

### 3. Tính Improved Repulsive Force

Repulsive force có nhiệm vụ đẩy UAV ra xa vật cản.

Mỗi vật cản chỉ tạo lực đẩy khi UAV nằm trong vùng ảnh hưởng:

```text
distance_to_obstacle <= d0
```

Lực đẩy được chia thành hai thành phần:

- `F_rep1`: đẩy UAV ra xa vật cản.
- `F_rep2`: bổ sung thành phần hướng về goal.

Cách này giúp giảm hiện tượng UAV không thể tới goal khi goal nằm gần vật cản.

Tổng lực đẩy được tính bằng tổng đóng góp của tất cả vật cản.

---

### 4. Tính tổng lực APF

Tổng lực điều hướng cơ bản được tính từ:

```text
F_total = F_att + F_rep
```

Trong điều kiện bình thường, hướng của `F_total` được dùng làm hướng chuyển động của UAV.

---

### 5. Phát hiện Local Minimum

Local minimum xảy ra khi lực hút và lực đẩy gần như cân bằng nhau làm tổng lực rất nhỏ.

Thuật toán kiểm tra:

```text
norm(F_total) < F_enter
```

đồng thời UAV vẫn còn cách goal và đang ở gần vật cản.

Khi thỏa điều kiện này, chế độ tìm hướng tiếp tuyến được kích hoạt.

---

### 6. Hysteresis cho Local Minimum

Để tránh việc tangent liên tục bật và tắt quanh cùng một ngưỡng, thuật toán sử dụng hai ngưỡng:

```matlab
F_enter
F_exit
```

Trong đó:

```text
F_exit > F_enter
```

Logic:

```text
norm(F_total) < F_enter
        -> bật tangent

F_enter <= norm(F_total) <= F_exit
        -> giữ tangent

norm(F_total) > F_exit
        -> tắt tangent
```

Cơ chế này giúp hệ thống ổn định hơn khi UAV vừa thoát khỏi local minimum.

---

### 7. Xác định Vector Pháp Tuyến

Khi local minimum được phát hiện, thuật toán tìm vật cản gần UAV nhất.

Vector pháp tuyến được lấy theo hướng từ vật cản tới UAV:

```text
normal = pos - obs_near
```

Sau đó chuẩn hóa để thu được vector đơn vị `n`.

Vector này được dùng để xây dựng mặt phẳng tiếp tuyến quanh vật cản.

---

### 8. Xây dựng Tangent Plane

Trong không gian 3D, có nhiều hướng tiếp tuyến quanh vật cản.

Thuật toán xây dựng hai vector cơ sở `e1` và `e2` nằm trên mặt phẳng tiếp tuyến.

Từ hai vector này, nhiều hướng tiếp tuyến khác nhau có thể được sinh ra để tìm hướng thoát local minimum tốt nhất.

---

### 9. Sinh các Tangent Candidate

Thuật toán tạo `N_tangent` hướng tiếp tuyến phân bố đều trên mặt phẳng tiếp tuyến.

Ví dụ:

```matlab
N_tangent = 12;
```

tương ứng với 12 hướng candidate khác nhau.

Mỗi hướng được đánh giá trước khi chọn làm hướng thoát local minimum.

---

### 10. Đánh giá Tangent Candidate

Mỗi tangent candidate được đánh giá theo ba tiêu chí:

#### Hướng về goal

Candidate được ưu tiên nếu nó giúp UAV tiếp tục tiến về phía goal.

#### Khoảng trống phía trước

Thuật toán dự đoán một số vị trí phía trước theo hướng candidate và kiểm tra khoảng cách tới các vật cản.

Hướng có khoảng trống lớn hơn sẽ được ưu tiên.

#### Duy trì hướng trước đó

Hướng tangent trước được lưu lại trong `tangent_prev`.

Candidate gần với hướng trước sẽ được ưu tiên để tránh việc UAV liên tục đổi phía khi vòng qua obstacle.

---

### 11. Chọn Tangent Tốt Nhất

Sau khi tính score cho tất cả candidate, hướng có score lớn nhất được chọn:

```text
best_tangent
```

Lực tiếp tuyến được tạo:

```text
F_tan = k_tan * best_tangent
```

Khi local minimum đang active:

```text
F_cmd_raw = F_total + F_tan
```

Khi không có local minimum:

```text
F_cmd_raw = F_total
```

Như vậy tangent chỉ được sử dụng khi cần thiết, không được cộng liên tục vào lực tổng hợp.

---

### 12. Giảm Oscillation

Khi UAV bay gần vật cản, hướng APF có thể thay đổi nhanh giữa các bước tính toán.

Thuật toán lưu hướng chuyển động trước đó:

```text
prev_move_dir
```

sau đó so sánh với hướng mới.

Nếu góc thay đổi nhỏ, hướng mới được sử dụng với trọng số tương đối lớn.

Nếu góc thay đổi lớn, thuật toán tăng trọng số của hướng trước để tránh UAV đổi hướng đột ngột.

Cơ chế này giúp giảm hiện tượng zig-zag khi UAV bay gần vật cản.

---

### 13. Speed Scheduling

Sau khi xác định hướng bay, thuật toán điều chỉnh độ lớn vận tốc.

UAV sẽ:

- Giảm tốc khi tiến gần goal.
- Giảm tốc khi tiến gần vật cản.
- Duy trì một vận tốc tối thiểu khi đang thoát local minimum.

Vận tốc cuối cùng được giới hạn bởi:

```matlab
v_max
```

---

### 14. Tính `V_cmd`

Sau khi xác định hướng và độ lớn vận tốc:

```text
V_cmd = speed * move_direction
```

Kết quả:

```text
V_cmd = [VN VE VD]
```

được gửi tới tầng điều khiển UAV.

---

### 15. Tính `yaw_cmd`

Yaw được tính dựa trên hướng vận tốc trong mặt phẳng North-East:

```matlab
yaw_cmd = atan2(V_cmd(2), V_cmd(1));
```

Nhờ đó UAV có thể quay theo hướng chuyển động mong muốn.

---

## Luồng xử lý tổng quát

```text
Input:
pos, goal, obs
katt, krep, d0
v_max, numobs

        |
        v

Tính khoảng cách tới goal

        |
        v

Tính Attractive Force

        |
        v

Tính Improved Repulsive Force

        |
        v

F_total = F_att + F_rep

        |
        v

Kiểm tra Local Minimum

        |
        +----------------------+
        |                      |
       Có                     Không
        |                      |
        v                      |
Tìm tangent tốt nhất           |
        |                      |
        v                      |
F_total + F_tan                |
        |                      |
        +----------+-----------+
                   |
                   v

Giảm Oscillation

                   |
                   v

Speed Scheduling

                   |
                   v

Output:
V_cmd
yaw_cmd
Stop
```

---

## Các tham số chính

```matlab
goal_tol      = 0.30;   % Ngưỡng xác định UAV đã tới goal
F_enter       = 0.10;   % Ngưỡng bật tangent
F_exit        = 0.30;   % Ngưỡng tắt tangent
goal_min_dist = 0.50;   % Khoảng cách tối thiểu tới goal để xét local minimum

k_tan         = 1.0;    % Hệ số lực tiếp tuyến
N_tangent     = 12;     % Số tangent candidate
N_pred        = 3;      % Số bước dự đoán phía trước

w_goal        = 1.0;    % Trọng số hướng về goal
w_clear       = 1.0;    % Trọng số khoảng trống
w_prev        = 0.60;   % Trọng số duy trì hướng trước
```

---

## Tóm tắt

```text
Input:
pos      [1×3]
goal     [1×3]
obs      [N×3]
katt     [1×1]
krep     [1×1]
d0       [1×1]
v_max    [1×1]
numobs   [1×1]

          |
          v

      IAPF Planner

          |
          v

Output:
V_cmd    [1×3]
yaw_cmd  [1×1]
Stop     [1×1]
```

Thuật toán IAPF hiện tại kết hợp:

```text
Attractive Force
        +
Improved Repulsive Force
        +
Local Minimum Detection
        +
Dynamic Tangent Search
        +
Forward-looking
        +
Oscillation Suppression
        +
Speed Scheduling
```

để tạo lệnh vận tốc 3D cho UAV tránh vật cản và tiến tới goal.
