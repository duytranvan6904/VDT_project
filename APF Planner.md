# APF Planner for 3D Quadrotor

Hàm `APF_Planner` sử dụng phương pháp **Artificial Potential Field (APF)** để tạo lệnh vận tốc cho UAV 3D, dựa trên vị trí hiện tại, vị trí đích và vị trí các vật cản.

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