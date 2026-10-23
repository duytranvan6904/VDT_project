import time

import rclpy
from geometry_msgs.msg import PointStamped, PoseStamped
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Bool
from vision_msgs.msg import BoundingBox2D

from .annotation import draw_detections
from .aruco_logic import ArUcoDetector
from .camera_model import camera_info_to_intrinsics, fallback_camera_matrix, zero_distortion
from .detection_filter import filter_detections
from .image_conversion import ros_image_to_array
from .message_builders import (
    build_bbox_message,
    build_bgr_image_message,
    build_point_message,
    build_pose_message,
    resolve_stamp,
)

DEFAULT_IMAGE_SIZE = (640, 480)
STATISTICS_PERIOD_S = 2.0


def build_sensor_qos() -> QoSProfile:
    return QoSProfile(
        reliability=ReliabilityPolicy.BEST_EFFORT,
        history=HistoryPolicy.KEEP_LAST,
        depth=3,
    )


class ArucoNode(Node):
    def __init__(self) -> None:
        super().__init__("aruco_node")
        self.declare_node_parameters()
        self.frame_count = 0
        self.detect_count = 0
        self.last_statistics_time = time.monotonic()
        self.camera_info_received = False
        self.image_size = DEFAULT_IMAGE_SIZE
        self.detector = self.create_detector()
        self.create_publishers()
        self.create_camera_subscriptions()

    def warn_missing_camera_info(self) -> None:
        self.get_logger().warning(
            "camera_info not received, pose suppressed", throttle_duration_sec=2.0
        )

    def declare_node_parameters(self) -> None:
        self.declare_parameter("marker_id", 42)
        self.declare_parameter("marker_size_m", 0.15)
        self.declare_parameter("dictionary", "DICT_6X6_50")
        self.declare_parameter("min_detection_distance_m", 0.0)
        self.declare_parameter("min_z_m", 0.0)
        self.declare_parameter("image_topic", "/camera")
        self.declare_parameter("camera_info_topic", "/camera_info")
        self.declare_parameter("camera_frame_id", "camera_optical_frame")
        self.declare_parameter("fallback_horizontal_fov_rad", 1.52)
        self.declare_parameter("require_camera_info", True)
        self.declare_parameter("mask_margin_percent", 0.15)

    def read_parameter(self, name: str):
        return self.get_parameter(name).value

    def create_fallback_matrix(self, width: int, height: int):
        return fallback_camera_matrix(width, height, float(self.read_parameter("fallback_horizontal_fov_rad")))

    def create_detector(self) -> ArUcoDetector:
        width, height = self.image_size
        return ArUcoDetector(
            dictionary_name=str(self.read_parameter("dictionary")),
            marker_size_m=float(self.read_parameter("marker_size_m")),
            camera_matrix=self.create_fallback_matrix(width, height),
            dist_coeffs=zero_distortion(),
            target_marker_id=int(self.read_parameter("marker_id")),
        )

    def create_publishers(self) -> None:
        self.pose_publisher = self.create_publisher(PoseStamped, "/hpad/pose", 10)
        self.position_publisher = self.create_publisher(PointStamped, "/hpad/position_camera", 10)
        self.bbox_publisher = self.create_publisher(BoundingBox2D, "/hpad/bbox", 10)
        self.detected_publisher = self.create_publisher(Bool, "/hpad/detected", 10)
        self.annotated_publisher = self.create_publisher(Image, "/hpad/annotated", 10)

    def create_camera_subscriptions(self) -> None:
        qos = build_sensor_qos()
        self.create_subscription(
            CameraInfo, str(self.read_parameter("camera_info_topic")), self.camera_info_callback, qos
        )
        self.create_subscription(
            Image, str(self.read_parameter("image_topic")), self.image_callback, qos
        )

    def camera_info_callback(self, msg: CameraInfo) -> None:
        intrinsics = camera_info_to_intrinsics(msg)
        if intrinsics is None:
            return
        self.detector.set_camera_parameters(*intrinsics)
        self.camera_info_received = True

    def refresh_fallback_intrinsics(self, width: int, height: int) -> None:
        if self.camera_info_received or (width, height) == self.image_size:
            return
        self.image_size = (width, height)
        self.detector.set_camera_parameters(self.create_fallback_matrix(width, height), zero_distortion())

    def detect_target(self, image):
        detections = self.detector.process_frame(image)
        return filter_detections(
            detections,
            float(self.read_parameter("min_detection_distance_m")),
            float(self.read_parameter("min_z_m")),
        )

    def publish_detected_flag(self, found: bool) -> None:
        self.detected_publisher.publish(Bool(data=found))

    def publish_target(self, detection, header) -> None:
        stamp = resolve_stamp(header.stamp, self.get_clock().now().to_msg())
        frame_id = str(self.read_parameter("camera_frame_id"))
        self.pose_publisher.publish(build_pose_message(detection, stamp, frame_id))
        self.position_publisher.publish(build_point_message(detection, stamp, frame_id))
        self.bbox_publisher.publish(build_bbox_message(detection))

    def publish_annotated(self, image, detections, header) -> None:
        annotated = draw_detections(
            image,
            detections,
            self.detector.camera_matrix,
            self.detector.dist_coeffs,
            self.detector.marker_size_m,
        )
        self.annotated_publisher.publish(build_bgr_image_message(annotated, header))

    def log_statistics_if_due(self) -> None:
        now = time.monotonic()
        if now - self.last_statistics_time < STATISTICS_PERIOD_S:
            return
        self.last_statistics_time = now
        rate = 100.0 * self.detect_count / max(self.frame_count, 1)
        self.get_logger().info(
            f"frames={self.frame_count} detected={self.detect_count} ({rate:.1f}%) "
            f"camera_info={self.camera_info_received}"
        )

    def image_callback(self, msg: Image) -> None:
        image = ros_image_to_array(msg)
        if image is None:
            self.get_logger().warning(f"unsupported image encoding: {msg.encoding}")
            return
        self.refresh_fallback_intrinsics(msg.width, msg.height)
        self.frame_count += 1
        detections = self.detect_target(image)
        if detections and not self.camera_info_received and bool(self.read_parameter("require_camera_info")):
            self.warn_missing_camera_info()
            detections = []
        self.publish_detected_flag(bool(detections))
        if detections:
            self.detect_count += 1
            self.publish_target(detections[0], msg.header)
        self.publish_annotated(image, detections, msg.header)
        self.log_statistics_if_due()


def main() -> None:
    rclpy.init()
    node = ArucoNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
