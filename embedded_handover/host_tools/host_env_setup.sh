#!/usr/bin/env bash
# ==============================================================================
# Setup Environment for Host Computer (Máy tính trạm) & Raspberry Pi 5
# Mạng nội bộ phát qua Điện thoại Cá nhân (Mobile Hotspot)
# ==============================================================================

echo "=========================================================="
echo "🔧 Cấu hình Môi trường ROS 2 & Mạng cho Máy trạm GCS"
echo "=========================================================="

# 1. Source ROS 2 (Mặc định Humble)
if [ -f "/opt/ros/humble/setup.bash" ]; then
    source /opt/ros/humble/setup.bash
    echo "[OK] Sourced /opt/ros/humble/setup.bash"
elif [ -f "/opt/ros/iron/setup.bash" ]; then
    source /opt/ros/iron/setup.bash
    echo "[OK] Sourced /opt/ros/iron/setup.bash"
else
    echo "[WARN] Không tìm thấy /opt/ros/humble hoặc iron. Vui lòng kiểm tra ROS 2."
fi

# 2. Đồng bộ ROS_DOMAIN_ID (Phải giống nhau 100% giữa PC và Pi 5)
export ROS_DOMAIN_ID=42
echo "[OK] ROS_DOMAIN_ID=$ROS_DOMAIN_ID"

# 3. Khắc phục việc Điện thoại chặn Multicast bằng CycloneDDS Unicast
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
CYCLONE_CFG="$(dirname "$SCRIPT_DIR")/config/cyclonedds_hotspot.xml"

if [ -f "$CYCLONE_CFG" ]; then
    export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
    export CYCLONEDDS_URI="file://$CYCLONE_CFG"
    echo "[OK] Đã kích hoạt Unicast qua: $CYCLONE_CFG"
    echo "     (Lưu ý sửa IP của Pi 5 và PC trong file XML nếu IP bị đổi)"
else
    echo "[WARN] Chưa tìm thấy $CYCLONE_CFG"
fi

# 4. Kiểm tra IP hiện tại trên mạng Hotspot
HOST_IP=$(hostname -I | awk '{print $1}')
echo "----------------------------------------------------------"
echo "📡 Địa chỉ IP Máy tính trạm: $HOST_IP"
echo "👉 Kiểm tra ping tới Raspberry Pi 5: ping <IP_CỦA_PI_5>"
echo "👉 Lệnh khởi động GCS Operator Terminal:"
echo "   python3 $SCRIPT_DIR/gcs_operator_terminal.py"
echo "=========================================================="
