#!/usr/bin/env bash
set -euo pipefail

PATCH_DIR="${PATCH_DIR:-../patches}"
PX4_DIR="${PX4_DIR:-.}"

shopt -s nullglob
patches=("$PATCH_DIR"/*.patch)
if [ "${#patches[@]}" -eq 0 ]; then
    echo "Khong tim thay patch trong $PATCH_DIR" >&2
    exit 1
fi

if [ -n "$(git -C "$PX4_DIR" status --porcelain)" ]; then
    echo "PX4 checkout phai sach truoc khi ap patch." >&2
    exit 1
fi

for patch in "${patches[@]}"; do
    metadata="${patch%.patch}.json"
    checksum="${patch}.sha256"
    echo "Kiem tra: $patch"

    if [ ! -f "$metadata" ] || [ ! -f "$checksum" ]; then
        echo "Thieu metadata/checksum cho $patch; tao bang create_px4_patch.py." >&2
        exit 1
    fi

    expected_commit=$(python3 -c 'import json,sys; print(json.load(open(sys.argv[1]))["px4_commit"])' "$metadata")
    actual_commit=$(git -C "$PX4_DIR" rev-parse HEAD)
    if [ "$expected_commit" != "$actual_commit" ]; then
        echo "Sai PX4 commit: patch=$expected_commit, checkout=$actual_commit" >&2
        exit 1
    fi

    sha256sum --check "$checksum"
    git -C "$PX4_DIR" apply --check "$patch"
    git -C "$PX4_DIR" apply "$patch"
done

echo "Da ap dung patch. Kiem tra diff va build SITL:"
echo "  git diff --check"
echo "  make px4_sitl gz_x500"