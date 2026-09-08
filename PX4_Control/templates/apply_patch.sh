#!/bin/bash
set -e

PATCH_DIR="../patches"
PX4_DIR="."

for patch in "$PATCH_DIR"/*.patch; do
    echo "Ap dung: $patch"
    git -C "$PX4_DIR" apply --check "$patch" || {
        echo "Patch khong khop version hien tai: $patch"
        exit 1
    }
    git -C "$PX4_DIR" apply "$patch"
done

echo "Da ap dung xong tat ca patch. Chay lai build SITL de test:"
echo "  make px4_sitl gz_x500"