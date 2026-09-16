# VDT_project
 Hệ thống bám mục tiêu động thời gian thực và tránh vật cản tự động cho Quadrotors sử dụng ROS 2, PX4 Autopilot, YOLO + ByteTrack, EKF và Modified APF.
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
APF Planner for UAV 3D
Hàm `APF\\\_Planner` triển khai phương pháp Improved Artificial Potential Field (IAPF) để tạo lệnh vận tốc cho UAV trong không gian 3D.
Thuật toán hiện tại gồm ba phần chính:
Cải tiến lực đẩy để giảm hiện tượng Goal Non-Reachable with Obstacle Nearby (GNRON).
Phát hiện local minimum và chỉ kích hoạt vector tiếp tuyến động khi tổng lực APF nhỏ hơn một ngưỡng.
Giảm local path oscillation bằng directional weighting và cơ chế forward-looking tham khảo từ bài báo.
> Lưu ý: phần dynamic tangent search, tangent memory và hysteresis là phần mở rộng của implementation hiện tại. Bài báo gốc sử dụng local exploration factor để xử lý local minimum.
Cú pháp
```matlab
\\\[V\\\_cmd, yaw\\\_cmd, Stop] = APF\\\_Planner( ...
    pos, goal, obs, katt, krep, d0, v\\\_max, numobs)
```
Đầu vào
Tên	Kích thước	Mô tả	Đơn vị
`pos`	`1×3`	Vị trí hiện tại của UAV `\\\[N E D]`	m
`goal`	`1×3`	Vị trí đích `\\\[N E D]`	m
`obs`	`N×3`	Tọa độ các vật cản `\\\[N E D]`	m
`katt`	`1×1`	Hệ số lực hút	-
`krep`	`1×1`	Hệ số lực đẩy	-
`d0`	`1×1`	Khoảng cách ảnh hưởng của vật cản	m
`v\\\_max`	`1×1`	Vận tốc cực đại của UAV	m/s
`numobs`	`1×1`	Số lượng vật cản được xét	-
Trong đó:
`N`: số lượng vật cản.
Hệ tọa độ sử dụng là NED (North-East-Down).
`obs(i,:)` là tọa độ tâm của vật cản thứ `i`.
Ví dụ:
```matlab
pos  = \\\[0 0 0];
goal = \\\[20 10 -5];

obs = \\\[
     5  2  0;
    10  5 -2;
    15  8 -3
];

katt   = 10;
krep   = 100;
d0     = 5;
v\\\_max  = 2;
numobs = 3;
```
Đầu ra
Tên	Kích thước	Mô tả	Đơn vị
`V\\\_cmd`	`1×3`	Lệnh vận tốc `\\\[VN VE VD]`	m/s
`yaw\\\_cmd`	`1×1`	Góc yaw mong muốn	rad
`Stop`	`1×1`	Cờ báo UAV đã đến đích	-
`V\\\_cmd`
```text
V\\\_cmd = \\\[VN VE VD]
```
Trong đó:
`VN`: vận tốc theo hướng North.
`VE`: vận tốc theo hướng East.
`VD`: vận tốc theo hướng Down.
`yaw\\\_cmd`
Góc yaw được tính từ hướng chuyển động ngang:
```matlab
yaw\\\_cmd = atan2(V\\\_cmd(2), V\\\_cmd(1));
```
`Stop`
```text
Stop = 0  -> UAV chưa đến đích
Stop = 1  -> UAV đã đến đích
```
UAV được xem là đến đích khi:
```text
d\\\_goal <= goal\\\_tol
```
Trong code hiện tại:
```matlab
goal\\\_tol = 0.30;    % m
```
Tham số chính
```matlab
goal\\\_tol      = 0.30;    % Ngưỡng xác định UAV đã tới goal (m)

F\\\_enter       = 0.10;    % Ngưỡng bật tangent
F\\\_exit        = 0.30;    % Ngưỡng tắt tangent
goal\\\_min\\\_dist = 0.50;    % Khoảng cách tối thiểu tới goal để xét local minimum (m)

k\\\_tan         = 1.0;     % Hệ số tangential force
N\\\_tangent     = 12;      % Số tangent candidate
N\\\_pred        = 3;       % Số bước dự đoán forward-looking

w\\\_goal        = 1.0;     % Trọng số hướng về goal
w\\\_clear       = 1.0;     % Trọng số khoảng trống phía trước
w\\\_prev        = 0.60;    % Trọng số duy trì tangent trước
```
1. Attractive Force
Vector từ UAV tới goal:
```text
goal\\\_vec = goal - pos
```
Khoảng cách tới goal:
```text
d\\\_goal = norm(goal - pos)
```
Lực hút được tính:
```text
F\\\_att = 2 \\\* katt \\\* (goal - pos)
```
Trong MATLAB:
```matlab
goal\\\_vec = g - p;
F\\\_att = 2.0 \\\* katt \\\* goal\\\_vec;
```
`F\\\_att` luôn có xu hướng đưa UAV về phía goal.
2. Improved Repulsive Force cho GNRON
Trong APF truyền thống, khi goal nằm gần vật cản, lực đẩy có thể vẫn lớn ngay cả khi UAV đã gần goal. Khi đó UAV có thể không tới được đích.
Thuật toán sử dụng hai thành phần lực đẩy:
```text
F\\\_rep = F\\\_rep1 + F\\\_rep2
```
`F\\\_rep1`
`F\\\_rep1` là thành phần lực đẩy chính, có hướng ra xa vật cản.
Với vật cản thứ `i`:
```text
obs\\\_vec = pos - obs(i,:)
d\\\_obs   = norm(obs\\\_vec)
dir\\\_away = obs\\\_vec / d\\\_obs
```
Hệ số phụ thuộc khoảng cách tới goal:
```text
weight\\\_rep = 1/(1 + exp(-d\\\_goal)) - 0.5
```
Lực đẩy:
```text
F\\\_rep1 =
    2 \\\* krep
    \\\* (1/d\\\_obs - 1/d0)
    \\\* (1/d\\\_obs^2)
    \\\* weight\\\_rep
    \\\* dir\\\_away
```
Khi UAV tiến gần goal:
```text
d\\\_goal -> 0
```
thì:
```text
weight\\\_rep -> 0
```
do đó ảnh hưởng của `F\\\_rep1` giảm dần.
`F\\\_rep2`
`F\\\_rep2` là thành phần có hướng về goal:
```text
F\\\_rep2 =
    krep
    \\\* (1/d\\\_obs - 1/d0)^2
    \\\* exp(-d\\\_goal)/(1 + exp(-d\\\_goal))^2
    \\\* dir\\\_goal
```
Trong đó:
```text
dir\\\_goal = (goal - pos) / norm(goal - pos)
```
Tổng lực đẩy từ tất cả vật cản:
```text
F\\\_rep = sum(F\\\_rep1 + F\\\_rep2)
```
Tổng lực APF:
```text
F\\\_total = F\\\_att + F\\\_rep
```
Vật cản chỉ tạo lực đẩy khi:
```text
d\\\_obs <= d0
```
3. Phát hiện Local Minimum
Local minimum xuất hiện khi lực hút và lực đẩy gần như triệt tiêu nhau:
```text
F\\\_att + F\\\_rep \\\~= 0
```
Trong implementation, local minimum được phát hiện dựa trên độ lớn của tổng lực:
```text
norm(F\\\_total) < F\\\_enter
```
Tangent chỉ được kích hoạt khi đồng thời thỏa mãn:
```text
norm(F\\\_total) < F\\\_enter
d\\\_goal > goal\\\_min\\\_dist
min\\\_obs\\\_dist < d0
```
Trong code:
```matlab
if (F\\\_total\\\_norm < F\\\_enter) \\\&\\\& ...
   (d\\\_goal > goal\\\_min\\\_dist) \\\&\\\& ...
   obstacle\\\_near

    tangent\\\_active = true;
end
```
4. Hysteresis
Để tránh tangent liên tục bật và tắt quanh ngưỡng local minimum, thuật toán sử dụng hai ngưỡng:
```matlab
F\\\_enter = 0.10;
F\\\_exit  = 0.30;
```
Logic:
```text
norm(F\\\_total) < F\\\_enter
        |
        v
   Tangent ON
        |
        v
F\\\_enter <= norm(F\\\_total) <= F\\\_exit
        |
        v
   Tangent vẫn ON
        |
        v
norm(F\\\_total) > F\\\_exit
        |
        v
   Tangent OFF
```
Điều kiện quan trọng:
```text
F\\\_exit > F\\\_enter
```
5. Vector pháp tuyến của vật cản
Với vật cản gần nhất có tâm `obs\\\_near`, vector từ tâm vật cản tới UAV là:
```text
normal\\\_vec = pos - obs\\\_near
```
Vector pháp tuyến đơn vị:
```text
n = normal\\\_vec / norm(normal\\\_vec)
```
Hay:
```text
n = (pos - obs\\\_near) / norm(pos - obs\\\_near)
```
`n` có hướng:
```text
Obstacle  -------->  UAV
            n
```
Nếu vật cản được mô hình hóa như hình cầu, `n` cũng chính là hướng pháp tuyến bề mặt.
6. Mặt phẳng tiếp tuyến
Vector tiếp tuyến `t` phải vuông góc với vector pháp tuyến `n`:
```text
dot(t, n) = 0
```
Trong không gian 3D có vô số vector thỏa mãn điều kiện này, vì vậy thuật toán không sử dụng một tangent cố định mà tìm tangent tốt nhất trên toàn mặt phẳng tiếp tuyến.
7. Xây dựng Tangent Plane
Đầu tiên chọn một vector tham chiếu `ref` ít song song nhất với `n`.
Sau đó tạo hai vector cơ sở:
```text
e1 = cross(n, ref)
e2 = cross(n, e1)
```
`e1` và `e2`:
Vuông góc với `n`.
Vuông góc với nhau.
Cùng nằm trên tangent plane.
8. Sinh các Tangent Candidate
Số candidate:
```matlab
N\\\_tangent = 12;
```
Mỗi candidate được tạo bởi:
```text
t\\\_candidate =
    cos(theta) \\\* e1
    + sin(theta) \\\* e2
```
với:
```text
theta = 2\\\*pi\\\*(k-1)/N\\\_tangent
```
Khi `N\\\_tangent = 12`, khoảng cách góc giữa hai candidate liên tiếp là:
```text
360 deg / 12 = 30 deg
```
Như vậy thuật toán kiểm tra toàn bộ 360 độ trên tangent plane.
9. Hàm đánh giá Tangent
Mỗi tangent được đánh giá theo ba tiêu chí:
```text
score =
    w\\\_goal  \\\* goal\\\_score
    + w\\\_clear \\\* clear\\\_score
    + w\\\_prev  \\\* prev\\\_score
```
`goal\\\_score`
```text
goal\\\_score = dot(t\\\_candidate, dir\\\_goal)
```
Ý nghĩa:
```text
goal\\\_score \\\~  1  -> hướng tốt về goal
goal\\\_score \\\~  0  -> gần vuông góc goal
goal\\\_score <  0  -> có xu hướng đi ngược goal
```
`prev\\\_score`
```text
prev\\\_score = dot(t\\\_candidate, tangent\\\_prev)
```
Mục đích là hạn chế việc đổi phía vật cản liên tục:
```text
+t -> -t -> +t -> -t
```
Candidate gần với tangent trước sẽ có score lớn hơn.
`clear\\\_score`
Thuật toán dự đoán một số điểm phía trước theo candidate đang xét.
```matlab
N\\\_pred = 3;
```
Vị trí dự đoán:
```text
p\\\_test =
    pos
    + (h/N\\\_pred)
    \\\* look\\\_ahead
    \\\* t\\\_candidate
```
Tại mỗi điểm dự đoán, khoảng cách tới tất cả vật cản được tính.
Khoảng cách nhỏ nhất:
```text
min\\\_pred\\\_dist
```
được dùng để đánh giá clearance:
```text
clear\\\_score = min\\\_pred\\\_dist / d0
```
Candidate có vùng phía trước thoáng hơn sẽ được ưu tiên.
10. Chọn Tangent Tốt Nhất
Sau khi đánh giá tất cả candidate:
```text
best\\\_tangent = candidate có score lớn nhất
```
Hay:
```text
best\\\_tangent = arg max(score)
```
Tangent được lưu lại:
```matlab
tangent\\\_prev = best\\\_tangent;
```
để sử dụng ở bước tiếp theo.
11. Tangential Force
Lực tiếp tuyến:
```text
F\\\_tan = k\\\_tan \\\* best\\\_tangent
```
Nếu đang ở local minimum:
```text
F\\\_cmd\\\_raw = F\\\_total + F\\\_tan
```
Nếu không có local minimum:
```text
F\\\_cmd\\\_raw = F\\\_total
```
Do đó vector tiếp tuyến không được cộng liên tục vào APF.
12. Giảm Local Path Oscillation
Sau khi có `F\\\_cmd\\\_raw`, hướng mới được chuẩn hóa:
```text
w\\\_new = F\\\_cmd\\\_raw / norm(F\\\_cmd\\\_raw)
```
Hướng điều khiển ở bước trước được lưu trong:
```text
prev\\\_move\\\_dir
```
Góc thay đổi hướng được tính:
```text
delta\\\_alpha = acos(dot(prev\\\_move\\\_dir, w\\\_new))
```
Thuật toán chia thành ba trường hợp.
Góc thay đổi nhỏ
```text
0 deg <= delta\\\_alpha <= 30 deg
```
Sử dụng:
```matlab
m\\\_prev = 0.50;
m\\\_new  = 0.50;
```
Hướng lệnh:
```text
w\\\_cmd = 0.50\\\*w\\\_prev + 0.50\\\*w\\\_new
```
Góc thay đổi trung bình
```text
30 deg < delta\\\_alpha <= 60 deg
```
Sử dụng:
```matlab
m\\\_prev = 0.70;
m\\\_new  = 0.30;
```
Hướng lệnh:
```text
w\\\_cmd = 0.70\\\*w\\\_prev + 0.30\\\*w\\\_new
```
Góc thay đổi lớn
```text
delta\\\_alpha > 60 deg
```
Sử dụng:
```matlab
m\\\_prev = 0.90;
m\\\_new  = 0.10;
```
Hướng lệnh:
```text
w\\\_cmd = 0.90\\\*w\\\_prev + 0.10\\\*w\\\_new
```
Khi APF đột ngột yêu cầu thay đổi hướng lớn, thuật toán giữ ảnh hưởng của hướng trước nhiều hơn để hạn chế zig-zag.
> Các giá trị `0.50/0.50`, `0.70/0.30` và `0.90/0.10` là lựa chọn của implementation hiện tại. Bài báo chỉ mô tả quan hệ giữa các trọng số, không công bố trực tiếp các giá trị số này.
13. Speed Scheduling
Sau khi xác định hướng bay, thuật toán tính độ lớn vận tốc.
Giảm tốc khi gần goal:
```text
goal\\\_speed\\\_factor = d\\\_goal / d0
```
Giới hạn:
```text
0.15 <= goal\\\_speed\\\_factor <= 1
```
Giảm tốc khi gần vật cản:
```text
obs\\\_speed\\\_factor = min\\\_obs\\\_dist / d0
```
Giới hạn:
```text
0.25 <= obs\\\_speed\\\_factor <= 1
```
Vận tốc cuối:
```text
v\\\_cmd\\\_mag =
    v\\\_max
    \\\* goal\\\_speed\\\_factor
    \\\* obs\\\_speed\\\_factor
```
Khi đang thoát local minimum:
```text
v\\\_cmd\\\_mag >= 0.35 \\\* v\\\_max
```
Lệnh vận tốc:
```text
V\\\_cmd = v\\\_cmd\\\_mag \\\* move\\\_dir
```
14. Yaw Command
Yaw được tính từ thành phần vận tốc North và East:
```matlab
yaw\\\_cmd = atan2(V\\\_cmd(2), V\\\_cmd(1));
```
Tương đương:
```text
yaw\\\_cmd = atan2(VE, VN)
```
15. Luồng xử lý tổng quát
```text
Input:
pos, goal, obs
katt, krep, d0
v\\\_max, numobs

        |
        v

Calculate attractive force

        |
        v

Calculate improved repulsive force

        |
        v

F\\\_total = F\\\_att + F\\\_rep

        |
        v

Check local minimum

        |
        +--------------------+
        |                    |
       Yes                   No
        |                    |
        v                    |
Find dynamic tangent         |
Forward-looking search       |
        |                    |
        v                    |
F\\\_total + F\\\_tan              |
        |                    |
        +---------+----------+
                  |
                  v

Oscillation suppression

                  |
                  v

Speed scheduling

                  |
                  v

Output:
V\\\_cmd
yaw\\\_cmd
Stop
```
16. Gợi ý tuning
UAV không thoát được local minimum
Tăng:
```matlab
k\\\_tan
```
hoặc tăng:
```matlab
F\\\_enter
```
Tangent kích hoạt quá sớm
Giảm:
```matlab
F\\\_enter
```
Tangent bật/tắt liên tục
Tăng khoảng cách giữa hai ngưỡng:
```matlab
F\\\_enter = 0.10;
F\\\_exit  = 0.50;
```
UAV đổi phía vật cản liên tục
Tăng:
```matlab
w\\\_prev
```
UAV vòng quá xa goal
Tăng:
```matlab
w\\\_goal
```
hoặc giảm:
```matlab
w\\\_clear
```
UAV vẫn oscillation
Tăng ảnh hưởng của hướng trước:
```matlab
m\\\_prev\\\_2 = 0.80;
m\\\_new\\\_2  = 0.20;

m\\\_prev\\\_3 = 0.95;
m\\\_new\\\_3  = 0.05;
```
UAV đổi hướng quá chậm
Giảm `m\\\_prev\\\_2`, `m\\\_prev\\\_3` hoặc tăng `m\\\_new\\\_2`, `m\\\_new\\\_3`.
17. Khác biệt so với bài báo
GNRON
Improved repulsive force sử dụng ý tưởng của Eq. (8) và Eq. (9) trong bài báo.
Local Minimum
Bài báo sử dụng local exploration factor:
```text
X\\\_new = X + epsilon
```
Implementation hiện tại thay bằng:
```text
Detect local minimum
        |
        v
Find nearest obstacle
        |
        v
Calculate obstacle normal
        |
        v
Construct tangent plane
        |
        v
Generate tangent candidates
        |
        v
Evaluate:
goal + clearance + previous tangent
        |
        v
Select best tangent
        |
        v
Add F\\\_tan
```
Oscillation
Directional weighting và forward-looking được xây dựng dựa trên ý tưởng của Eq. (10) đến Eq. (13) trong bài báo.
18. Tài liệu tham khảo
Qiang Han, Xingyuan Ma, Hanlin Liu, Yibo Xu, Yunxiang Xie, Qianguo Yang, Fanqin Meng, “Improved Artificial Potential Field Method for UAV Path Planning,” IEEE Access, 2025.
DOI:
```text
10.1109/ACCESS.2025.3620220
```
