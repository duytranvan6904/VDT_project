import numpy as np
from geometry_msgs.msg import PointStamped, PoseStamped
from sensor_msgs.msg import Image
from vision_msgs.msg import BoundingBox2D

from .aruco_logic import Detection
from .geometry_utils import compute_corner_bounding_box


def resolve_stamp(header_stamp, fallback_stamp):
    if header_stamp.sec > 0 or header_stamp.nanosec > 0:
        return header_stamp
    return fallback_stamp


def build_pose_message(detection: Detection, stamp, frame_id: str) -> PoseStamped:
    pose = detection["pose"]
    qw, qx, qy, qz = pose["quaternion"]
    message = PoseStamped()
    message.header.stamp = stamp
    message.header.frame_id = frame_id
    message.pose.position.x = pose["x"]
    message.pose.position.y = pose["y"]
    message.pose.position.z = pose["z"]
    message.pose.orientation.w = qw
    message.pose.orientation.x = qx
    message.pose.orientation.y = qy
    message.pose.orientation.z = qz
    return message


def build_point_message(detection: Detection, stamp, frame_id: str) -> PointStamped:
    pose = detection["pose"]
    message = PointStamped()
    message.header.stamp = stamp
    message.header.frame_id = frame_id
    message.point.x = pose["x"]
    message.point.y = pose["y"]
    message.point.z = pose["z"]
    return message


def build_bbox_message(detection: Detection) -> BoundingBox2D:
    center_x, center_y, size_x, size_y = compute_corner_bounding_box(detection["corners"])
    message = BoundingBox2D()
    message.center.position.x = center_x
    message.center.position.y = center_y
    message.center.theta = 0.0
    message.size_x = size_x
    message.size_y = size_y
    return message


def build_bgr_image_message(image: np.ndarray, header) -> Image:
    message = Image()
    message.header = header
    message.height, message.width = image.shape[:2]
    message.encoding = "bgr8"
    message.is_bigendian = False
    message.step = image.shape[1] * 3
    message.data = image.tobytes()
    return message
