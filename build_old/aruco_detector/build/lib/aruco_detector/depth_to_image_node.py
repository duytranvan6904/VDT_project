from typing import Optional

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from geometry_msgs.msg import PolygonStamped
from sensor_msgs.msg import Image

DEPTH_MIN_M = 0.2
DEPTH_MAX_M = 8.0
INVALID_DEPTH_FILL_M = 10.0
MASK_MAX_AGE_S = 0.2


def decode_depth_meters(msg: Image) -> Optional[np.ndarray]:
    if msg.encoding in ("32FC1", "R_FLOAT32"):
        return np.frombuffer(msg.data, dtype=np.float32).reshape((msg.height, msg.width))
    if msg.encoding == "16UC1":
        raw = np.frombuffer(msg.data, dtype=np.uint16).reshape((msg.height, msg.width))
        return raw.astype(np.float32) / 1000.0
    return None


def normalize_depth_to_mono8(depth_m: np.ndarray) -> np.ndarray:
    cleaned = np.nan_to_num(depth_m, nan=INVALID_DEPTH_FILL_M, posinf=INVALID_DEPTH_FILL_M, neginf=0.0)
    clipped = np.clip(cleaned, DEPTH_MIN_M, DEPTH_MAX_M)
    scaled = (clipped - DEPTH_MIN_M) / (DEPTH_MAX_M - DEPTH_MIN_M) * 255.0
    return scaled.astype(np.uint8)


def build_mono8_message(mono: np.ndarray, header) -> Image:
    message = Image()
    message.header = header
    message.height, message.width = mono.shape
    message.encoding = "mono8"
    message.is_bigendian = False
    message.step = message.width
    message.data = mono.tobytes()
    return message

def stamp_to_seconds(stamp) -> float:
    return stamp.sec + stamp.nanosec * 1e-9


def apply_polygon_mask(depth_m: np.ndarray, polygon_msg) -> np.ndarray:
    points = np.rint(
        np.array([[p.x, p.y] for p in polygon_msg.polygon.points], dtype=np.float64)
    ).astype(np.int32)
    if len(points) < 3:
        return depth_m
    mask = np.zeros(depth_m.shape, dtype=np.uint8)
    cv2.fillPoly(mask, [points], 1)
    masked = depth_m.astype(np.float32, copy=True)
    masked[mask.astype(bool)] = np.nan
    return masked

class DepthToImageNode(Node):
    def __init__(self) -> None:
        super().__init__("depth_to_image_node")
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )
        self.publisher = self.create_publisher(Image, "/depth_camera/image_mono", 10)
        self.create_subscription(Image, "/depth_camera", self.depth_callback, qos)
        self.latest_polygon = None
        self.create_subscription(PolygonStamped, "/hpad/mask_polygon", self.polygon_callback, 10)

    def polygon_callback(self, msg: PolygonStamped) -> None:
        self.latest_polygon = msg

    def is_polygon_fresh(self, depth_stamp) -> bool:
        if self.latest_polygon is None:
            return False
        age = abs(stamp_to_seconds(depth_stamp) - stamp_to_seconds(self.latest_polygon.header.stamp))
        return age <= MASK_MAX_AGE_S

    def depth_callback(self, msg: Image) -> None:
        depth_m = decode_depth_meters(msg)
        if depth_m is None:
            return
        if self.is_polygon_fresh(msg.header.stamp):
            depth_m = apply_polygon_mask(depth_m, self.latest_polygon)
        self.publisher.publish(build_mono8_message(normalize_depth_to_mono8(depth_m), msg.header))


def main() -> None:
    rclpy.init()
    node = DepthToImageNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
