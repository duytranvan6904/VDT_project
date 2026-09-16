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
Improved APF Planner for UAV 3D (IAPF_Planner_3D_MultiObs.m)
1. Mục tiêu
README này mô tả thuật toán Improved Artificial Potential Field (IAPF) được triển khai trong MATLAB/Simulink cho UAV 3D.
Thuật toán hiện tại tập trung vào ba vấn đề chính:
Goal Non-Reachable with Obstacle Nearby (GNRON): cải tiến lực đẩy để UAV vẫn có thể tiến tới goal khi goal nằm gần obstacle.
Local Minimum: chỉ kích hoạt cơ chế thoát bằng vector tiếp tuyến động khi tổng lực APF nhỏ hơn một ngưỡng.
Local Path Oscillation: hạn chế thay đổi hướng đột ngột bằng directional weighting và cơ chế forward-looking tham khảo từ bài báo.
Phần vector tiếp tuyến động, tangent memory và hysteresis là phần mở rộng của implementation hiện tại; bài báo gốc dùng local exploration factor cho local minimum.
---
2. Hàm MATLAB
```matlab
function [V_cmd, yaw_cmd, Stop] = APF_Planner( ...
    pos, goal, obs, katt, krep, d0, v_max, numobs)
```
Input
Biến	Kích thước	Ý nghĩa
`pos`	`1x3`	Vị trí hiện tại UAV `[N E D]`
`goal`	`1x3`	Vị trí đích `[N E D]`
`obs`	`Nx3`	Tọa độ các obstacle `[N E D]`
`katt`	scalar	Hệ số attractive force
`krep`	scalar	Hệ số repulsive force
`d0`	scalar	Khoảng cách ảnh hưởng của obstacle
`v_max`	scalar	Vận tốc cực đại
`numobs`	scalar	Số obstacle đang xét
Output
Biến	Kích thước	Ý nghĩa
`V_cmd`	`1x3`	Vận tốc đặt `[VN VE VD]`
`yaw_cmd`	scalar	Góc yaw đặt
`Stop`	scalar	`1` nếu đã tới goal, ngược lại `0`
---
3. Hệ tọa độ
Thuật toán dùng hệ tọa độ NED:
```text
N : North
E : East
D : Down
```
Do đó:
```matlab
V_cmd = [VN VE VD];
yaw_cmd = atan2(V_cmd(2), V_cmd(1));
```
---
4. Cấu trúc tổng thể
```text
UAV position
    |
    v
Attractive force
    |
    v
Improved repulsive force
    |
    v
F_total = F_att + F_rep
    |
    +-----------------------------+
    |                             |
||F_total|| < F_enter ?           |
    |                             |
   Yes                            No
    |                             |
    v                             |
Dynamic tangent search            |
Forward-looking                   |
    |                             |
F_cmd_raw = F_total + F_tan       |
    |                             |
    +--------------+--------------+
                   |
                   v
        Oscillation suppression
        Directional weighting
                   |
                   v
             move_direction
                   |
                   v
            Speed scheduling
                   |
                   v
          V_cmd, yaw_cmd, Stop
```
---
5. Attractive Force
Vector từ UAV tới goal:
[
\mathbf d_g = X_g-X
]
Khoảng cách tới goal:
[
d_g=|X_g-X|
]
Vector đơn vị hướng về goal:
[
\hat{\mathbf g}
\frac{X_g-X}{|X_g-X|}
]
Potential attractive:
[
U_{att}=k_{att}d_g^2
]
Lực attractive:
[
\boxed{
F_{att}=2k_{att}(X_g-X)
}
]
Trong code:
```matlab
goal_vec = g - p;
F_att = 2.0 * katt * goal_vec;
```
---
6. Improved Repulsive Force để xử lý GNRON
Trong APF truyền thống, nếu goal nằm gần obstacle, repulsive force có thể vẫn mạnh khi UAV đã ở gần goal. Khi đó UAV có thể không tới được đích.
Thuật toán hiện tại dùng lực đẩy cải tiến theo ý tưởng Eq. (8) và Eq. (9) của bài báo:
[
\boxed{
F_{rep}=F_{rep1}+F_{rep2}
}
]
6.1 Vector từ obstacle tới UAV
Với obstacle thứ (i):
[
r_i=X-X_{o,i}
]
Khoảng cách:
[
d_i=|X-X_{o,i}|
]
Vector đơn vị hướng ra khỏi obstacle:
[
\boxed{
\hat n_i=
\frac{X-X_{o,i}}
{|X-X_{o,i}|}
}
]
Trong code:
```matlab
obs_vec = p - obs_i;
d_obs = norm(obs_vec);
dir_away = obs_vec / d_obs;
```
6.2 Thành phần (F_{rep1})
Trong implementation hiện tại:
[
F_{rep1}
2k_{rep}
\left(
\frac{1}{d_i}-\frac{1}{d_0}
\right)
\frac{1}{d_i^2}
w_g
\hat n_i
]
với:
[
w_g=
\frac{1}{1+e^{-d_g}}-\frac12
]
Khi:
[
d_g\rightarrow0
]
thì:
[
w_g\rightarrow0
]
nên ảnh hưởng của lực đẩy chính giảm khi UAV tiến gần goal.
6.3 Thành phần (F_{rep2})
[
F_{rep2}
k_{rep}
\left(
\frac{1}{d_i}-\frac{1}{d_0}
\right)^2
\frac{e^{-d_g}}
{(1+e^{-d_g})^2}
\hat g
]
Trong code:
```matlab
sigmoid_derivative = ...
    exp_goal / ((1.0 + exp_goal)^2);

coeff2 = ...
    krep ...
    * (1.0/d_safe - 1.0/d0)^2 ...
    * sigmoid_derivative;

F_rep2 = coeff2 * dir_goal;
```
6.4 Tổng lực APF
[
F_{rep}
\sum_i
(F_{rep1,i}+F_{rep2,i})
]
và:
[
\boxed{
F_{total}=F_{att}+F_{rep}
}
]
---
7. Phát hiện Local Minimum
Local minimum được xem là xuất hiện khi attractive force và repulsive force gần như triệt tiêu:
[
F_{att}+F_{rep}\approx0
]
hay:
[
\boxed{
|F_{total}|\approx0
}
]
Tangent chỉ được kích hoạt khi:
[
\boxed{
|F_{total}|<F_{enter}
}
]
đồng thời:
[
d_{goal}>goal_min_dist
]
và obstacle gần nhất vẫn nằm trong vùng ảnh hưởng:
[
d_{obs}<d_0
]
Trong code:
```matlab
if (F_total_norm < F_enter) && ...
   (d_goal > goal_min_dist) && ...
   obstacle_near

    tangent_active = true;
end
```
---
8. Hysteresis
Hai ngưỡng:
```matlab
F_enter = 0.10;
F_exit  = 0.30;
```
được dùng để tránh tangent bật/tắt liên tục.
Logic:
```text
||F_total|| < F_enter
        |
        v
  Tangent ON
        |
        v
F_enter < ||F_total|| < F_exit
        |
        v
  Tangent vẫn ON
        |
        v
||F_total|| > F_exit
        |
        v
  Tangent OFF
```
Vì:
[
F_{exit}>F_{enter}
]
nên hệ thống có vùng hysteresis.
---
9. Vector pháp tuyến obstacle
Với obstacle gần nhất có tâm (X_o), vector pháp tuyến được lấy:
[
\boxed{
n=
\frac{X-X_o}
{|X-X_o|}
}
]
Nó hướng từ obstacle ra UAV.
Nếu obstacle được mô hình hóa như hình cầu thì đây chính là pháp tuyến bề mặt tại hướng UAV.
Nếu obstacle là cuboid hoặc hình dạng phức tạp, nên thay tâm obstacle bằng điểm gần UAV nhất trên bề mặt.
---
10. Tangent Plane
Mọi vector tiếp tuyến (t) phải thỏa:
[
\boxed{
t^Tn=0
}
]
Trong 3D, điều kiện này tạo thành một mặt phẳng tiếp tuyến chứ không tạo ra một tangent duy nhất.
Thuật toán vì vậy tạo nhiều candidate và chọn hướng tốt nhất.
---
11. Xây dựng cơ sở tangent plane
Chọn một vector tham chiếu `ref` ít song song nhất với (n).
Sau đó:
[
e_1=n\times ref
]
[
e_2=n\times e_1
]
Hai vector (e_1,e_2) vuông góc nhau và cùng nằm trên tangent plane.
---
12. Sinh các tangent candidate
Số candidate:
```matlab
N_tangent = 12;
```
Candidate thứ (k):
[
\boxed{
t_k=
\cos\theta_k e_1+
\sin\theta_k e_2
}
]
với:
[
\theta_k=
\frac{2\pi(k-1)}
{N_{tangent}}
]
Nếu:
[
N_{tangent}=12
]
thì góc giữa hai candidate liên tiếp là:
[
30^\circ
]
Như vậy thuật toán tìm kiếm toàn bộ 360° trên tangent plane.
---
13. Hàm đánh giá tangent
Mỗi tangent được đánh giá bằng:
[
\boxed{
J=
w_{goal}J_{goal}
+
w_{clear}J_{clear}
+
w_{prev}J_{prev}
}
]
13.1 Goal score
[
\boxed{
J_{goal}=t_k^T\hat g
}
]
Gần `1`: hướng tốt về goal.
Gần `0`: gần vuông góc goal.
Âm: đi ngược goal.
13.2 Previous tangent score
Tangent trước được lưu trong:
```matlab
tangent_prev
```
Score:
[
\boxed{
J_{prev}=t_k^Tt_{prev}
}
]
Mục đích là ngăn việc đổi liên tục:
```text
+t -> -t -> +t -> -t
```
13.3 Forward-looking clearance
Với:
```matlab
N_pred = 3;
```
thuật toán kiểm tra nhiều điểm phía trước:
[
X_{pred,h}
X+
\frac{h}{N_{pred}}
L_{look}
t_k
]
với:
[
h=1,...,N_{pred}
]
Tại mỗi điểm dự đoán, tính khoảng cách tới tất cả obstacle.
Giá trị nhỏ nhất:
[
d_{min,pred}
]
được chuẩn hóa:
[
\boxed{
J_{clear}
\frac{d_{min,pred}}{d_0}
}
]
Candidate có clearance lớn hơn sẽ được ưu tiên.
---
14. Chọn tangent tối ưu
Sau khi đánh giá tất cả candidate:
[
\boxed{
t^*
\arg\max_{t_k}J(t_k)
}
]
Sau đó lưu:
```matlab
tangent_prev = best_tangent;
```
---
15. Tangential Force
[
\boxed{
F_{tan}=k_{tan}t^*
}
]
Nếu local minimum đang active:
[
\boxed{
F_{cmd,raw}=F_{total}+F_{tan}
}
]
Nếu không:
[
\boxed{
F_{cmd,raw}=F_{total}
}
]
Điểm quan trọng: tangent không được cộng liên tục, chỉ hoạt động khi thuật toán phát hiện local minimum.
---
16. Xử lý Oscillation
Từ:
[
F_{cmd,raw}
]
ta tạo hướng mới:
[
w_{new}
\frac{F_{cmd,raw}}
{|F_{cmd,raw}|}
]
Hướng bước trước:
[
w_{prev}
]
Góc thay đổi hướng:
[
\boxed{
\Delta\alpha
\cos^{-1}
(w_{prev}^T w_{new})
}
]
Cơ chế này tham khảo directional weighting trong Eq. (10)-(11) của bài báo.
---
17. Directional Weighting
Bài báo chia (\Delta\alpha) thành ba vùng.
17.1 Góc nhỏ
[
0\leq\Delta\alpha\leq30^\circ
]
Implementation:
```matlab
m_prev = 0.50;
m_new  = 0.50;
```
[
w_{cmd}
0.5w_{prev}
+
0.5w_{new}
]
17.2 Góc trung bình
[
30^\circ<\Delta\alpha\leq60^\circ
]
Implementation:
```matlab
m_prev = 0.70;
m_new  = 0.30;
```
[
w_{cmd}
0.7w_{prev}
+
0.3w_{new}
]
17.3 Góc lớn
[
\Delta\alpha>60^\circ
]
Implementation:
```matlab
m_prev = 0.90;
m_new  = 0.10;
```
[
w_{cmd}
0.9w_{prev}
+
0.1w_{new}
]
Khi hướng mới thay đổi quá mạnh, thuật toán ưu tiên hướng cũ nhiều hơn để giảm zig-zag.
> Lưu ý: bài báo chỉ nêu quan hệ giữa các trọng số:
>
> \[
> m_i=m_i',\quad
> m_i>m_i',\quad
> m_i\gg m_i'
> \]
>
> Các giá trị `0.5/0.5`, `0.7/0.3`, `0.9/0.1` là lựa chọn của implementation hiện tại, không phải số được công bố trực tiếp trong bài báo.
---
18. Speed Scheduling
Sau khi tìm được hướng điều khiển, thuật toán tính độ lớn vận tốc.
18.1 Gần goal
[
k_g=
\frac{d_{goal}}{d_0}
]
giới hạn:
[
0.15\leq k_g\leq1
]
18.2 Gần obstacle
[
k_o=
\frac{d_{obs,min}}{d_0}
]
giới hạn:
[
0.25\leq k_o\leq1
]
18.3 Vận tốc cuối
[
\boxed{
v_{cmd}
v_{max}k_gk_o
}
]
Nếu đang thoát local minimum:
[
v_{cmd}\geq0.35v_{max}
]
Cuối cùng:
[
\boxed{
V_{cmd}=v_{cmd}w_{cmd}
}
]
---
19. Yaw Command
Trong NED:
[
\boxed{
\psi_{cmd}
\operatorname{atan2}(V_E,V_N)
}
]
Trong MATLAB:
```matlab
yaw_cmd = atan2(V_cmd(2), V_cmd(1));
```
---
20. Các parameter chính
Parameter	Giá trị khởi tạo	Chức năng
`goal_tol`	`0.30 m`	Ngưỡng coi UAV đã tới goal
`F_enter`	`0.10`	Kích hoạt tangent
`F_exit`	`0.30`	Tắt tangent
`goal_min_dist`	`0.50 m`	Tránh coi vùng sát goal là local minimum
`k_tan`	`1.0`	Cường độ tangential force
`N_tangent`	`12`	Số tangent candidate
`N_pred`	`3`	Số bước forward-looking
`look_ahead`	`0.25*d0`	Khoảng nhìn trước
`w_goal`	`1.0`	Trọng số hướng về goal
`w_clear`	`1.0`	Trọng số clearance
`w_prev`	`0.60`	Trọng số giữ tangent cũ
`theta_small`	`30°`	Ngưỡng đổi hướng nhỏ
`theta_large`	`60°`	Ngưỡng đổi hướng lớn
---
21. Gợi ý tuning
UAV không thoát local minimum
Tăng:
```matlab
k_tan
```
hoặc:
```matlab
F_enter
```
Tangent kích hoạt quá sớm
Giảm:
```matlab
F_enter
```
Tangent bật/tắt liên tục
Tăng khoảng hysteresis, ví dụ:
```matlab
F_enter = 0.10;
F_exit  = 0.50;
```
UAV đổi phía obstacle liên tục
Tăng:
```matlab
w_prev
```
UAV vòng quá xa goal
Tăng:
```matlab
w_goal
```
hoặc giảm:
```matlab
w_clear
```
UAV vẫn zig-zag
Tăng ảnh hưởng hướng trước:
```matlab
m_prev_2 = 0.80;
m_new_2  = 0.20;

m_prev_3 = 0.95;
m_new_3  = 0.05;
```
UAV quay quá chậm
Giảm `m_prev_2`, `m_prev_3` hoặc tăng `m_new_2`, `m_new_3`.
---
22. Khác biệt so với bài báo
GNRON
Phần improved repulsive force dùng ý tưởng Eq. (8)-(9) của bài báo.
Local minimum
Bài báo dùng local exploration factor (
arepsilon):
[
X_{new}=X+\varepsilon
]
Implementation hiện tại thay bằng:
```text
Detect local minimum
        |
        v
Find nearest obstacle
        |
        v
Construct tangent plane
        |
        v
Generate tangent candidates
        |
        v
Evaluate goal + clearance + previous tangent
        |
        v
Select best tangent
        |
        v
Add F_tan
```
Dynamic tangent search là phần mở rộng của thuật toán hiện tại.
Oscillation
Directional weighting và forward-looking được tham khảo từ Eq. (10)-(13) của bài báo.
Các trọng số số học cụ thể trong code là lựa chọn implementation và cần được tune theo mô hình UAV.
---
23. Pseudocode
```text
INPUT:
    pos
    goal
    obs

1. Calculate vector/distance to goal

2. Calculate attractive force

3. For each obstacle:
       Calculate obstacle distance

       If obstacle inside influence distance:
           Calculate Frep1
           Calculate Frep2
           Accumulate repulsive force

4. F_total = F_att + F_rep

5. Detect local minimum

       If |F_total| < F_enter:
           tangent_active = true

       If tangent_active and |F_total| > F_exit:
           tangent_active = false

6. If tangent_active:

       Find nearest obstacle
       Calculate normal vector
       Construct tangent plane
       Generate N tangent candidates

       For each candidate:
           Calculate goal score
           Calculate previous tangent score
           Predict several points ahead
           Calculate clearance score

       Select tangent with maximum score

       F_cmd_raw = F_total + F_tan

   Else:

       F_cmd_raw = F_total

7. Normalize F_cmd_raw -> w_new

8. If obstacle nearby:
       Calculate delta_alpha
       Select directional weights
       Blend w_prev and w_new

   Else:
       w_cmd = w_new

9. Calculate speed from:
       distance to goal
       distance to nearest obstacle

10. V_cmd = speed * w_cmd

11. yaw_cmd = atan2(VE, VN)

OUTPUT:
       V_cmd
       yaw_cmd
       Stop
```
---
24. Tài liệu tham khảo
Qiang Han, Xingyuan Ma, Hanlin Liu, Yibo Xu, Yunxiang Xie, Qianguo Yang, Fanqin Meng,  
“Improved Artificial Potential Field Method for UAV Path Planning,”  
IEEE Access, 2025.  
DOI: `10.1109/ACCESS.2025.3620220`
Các ý tưởng được tham khảo:
Target distance-weighted repulsive field
(F_{rep1}), (F_{rep2}) cho GNRON
Local-minimum detection
Directional weighting
Forward-looking decision mechanism
Local path oscillation suppression
Implementation hiện tại bổ sung:
Dynamic tangent search
Tangent memory
Hysteresis
Candidate scoring theo goal, clearance và hướng trước
