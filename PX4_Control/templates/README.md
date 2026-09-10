# PX4 Patch Templates

Bộ template này tạo patch có thể tái lập cho một checkout `PX4-Autopilot`. Không chạy các script này trong `VDT_project`; chạy từ môi trường Linux có Git và Python 3.

## Quy trình tạo patch

1. Checkout đúng PX4 release/commit và ghi lại commit:

```bash
cd PX4-Autopilot
git checkout v1.16.0
git rev-parse HEAD
git status --short
```

2. Sửa PX4 bằng các template:

- `velocity_pid_patch_template.cpp`: logic thay đổi velocity PID.
- `param_template.c`: khai báo parameter PX4.

Các đoạn trong template cần được chèn vào source PX4 thật theo anchor của release đang dùng. Template không tự đoán vị trí vì API PX4 thay đổi giữa các release.

3. Đảm bảo chỉ có thay đổi mong muốn, sau đó tạo patch:

```bash
python3 /path/to/VDT_project/PX4_Control/templates/create_px4_patch.py \
  --px4-dir /path/to/PX4-Autopilot \
  --output-dir /path/to/patches \
  --patch-name 001_custom_velocity_pid.patch \
  --base-ref v1.16.0
```

Script tạo ba file:

- `001_custom_velocity_pid.patch`: patch binary/full-index từ `git diff`.
- `001_custom_velocity_pid.json`: commit, tag/describe, branch, file đã sửa và SHA-256.
- `001_custom_velocity_pid.patch.sha256`: checksum patch.

Nếu checkout có file untracked, script dừng để tránh tạo bundle thiếu file. Dùng `--allow-untracked` chỉ khi biết rõ các file đó không thuộc patch.

## Quy trình áp patch

Đặt ba file cạnh nhau trong thư mục `patches`, rồi chạy từ root PX4:

```bash
PX4_DIR=/path/to/PX4-Autopilot \
PATCH_DIR=/path/to/patches \
bash /path/to/VDT_project/PX4_Control/templates/apply_patch.sh
```

Script sẽ kiểm tra:

1. PX4 checkout sạch.
2. `HEAD` đúng `px4_commit` trong manifest.
3. SHA-256 đúng.
4. `git apply --check` thành công.

Sau khi áp dụng:

```bash
git diff --check
git diff --stat
make px4_sitl gz_x500
```

## Khi đổi PX4 version

Không áp patch cũ mù quáng lên release khác. Checkout release mới, đọc lại anchor trong `Patch_Guide.md`, cập nhật template/patch, tạo bundle mới và ghi commit mới bằng `create_px4_patch.py`. Nếu patch không khớp, script apply phải dừng; không dùng `--reject` cho build bay thật nếu chưa review từng hunk.
