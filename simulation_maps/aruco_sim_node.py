#!/usr/bin/env python3
"""ArUco detection node for the Gazebo RGB-D camera.

Inputs:
  /camera       sensor_msgs/Image
  /camera_info  sensor_msgs/CameraInfo (optional; an SDF fallback is used)
  /depth_camera sensor_msgs/Image (optional)

Outputs:
  /hpad/pose             geometry_msgs/PoseStamped
  /hpad/position_camera  geometry_msgs/PointStamped (x, y, z in optical frame)
  /hpad/bbox             vision_msgs/BoundingBox2D
  /hpad/detected         std_msgs/Bool
  /hpad/annotated        sensor_msgs/Image

The PnP pose is expressed in the ROS camera optical frame.  The separate
position topic makes x/y/z easy to inspect with ``ros2 topic echo``.
"""

import os
import sys
import time

import cv2
import numpy as np
import rclpy
from geometry_msgs.msg import PointStamped, PoseStamped
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Bool
from vision_msgs.msg import BoundingBox2D

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if REPO_ROOT not in sys.path:
    sys.path.insert(0, REPO_ROOT)

from vision.aruco_detector import ArUcoDetector


class ArucoSimulationNode(Node):
    def __init__(self):
        super().__init__('aruco_sim_node')

        self.declare_parameter('marker_id', 42)
        # marker42.png contains a 30 px title strip above the 371 px code.
        # The 0.90 m H-pad therefore gives an ArUco code side of about
        # 0.90 * 371 / 411 = 0.812 m for PnP scale estimation.
        self.declare_parameter('marker_size_m', 0.812)
        self.declare_parameter('dictionary', 'DICT_6X6_50')
        self.marker_id = int(self.get_parameter('marker_id').value)
        self.marker_size = float(self.get_parameter('marker_size_m').value)
        dictionary = str(self.get_parameter('dictionary').value)

        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=3,
        )

        self.camera_info = None
        self.latest_depth = None
        self.latest_depth_header = None
        self.last_log = 0.0
        self.last_detection_log = 0.0
        self.frame_count = 0
        self.detect_count = 0

        self.pose_pub = self.create_publisher(PoseStamped, '/hpad/pose', 10)
        self.position_pub = self.create_publisher(
            PointStamped, '/hpad/position_camera', 10
        )
        self.bbox_pub = self.create_publisher(BoundingBox2D, '/hpad/bbox', 10)
        self.detected_pub = self.create_publisher(Bool, '/hpad/detected', 10)
        self.annotated_pub = self.create_publisher(Image, '/hpad/annotated', 10)

        self.create_subscription(CameraInfo, '/camera_info', self.camera_info_cb, 10)
        self.create_subscription(Image, '/depth_camera', self.depth_cb, sensor_qos)
        self.create_subscription(Image, '/camera', self.image_cb, sensor_qos)

        # Oak-D-Lite SDF fallback: 1920x1080, horizontal FOV 1.204 rad.
        self.image_width = 1920
        self.image_height = 1080
        self.camera_matrix = self.fallback_camera_matrix(self.image_width, self.image_height)
        self.dist_coeffs = np.zeros((5, 1), dtype=np.float64)
        self.detector = ArUcoDetector(
            dictionary_name=dictionary,
            marker_size_meters=self.marker_size,
            camera_matrix=self.camera_matrix,
            dist_coeffs=self.dist_coeffs,
            target_marker_ids=self.marker_id,
        )

        self.get_logger().info(
            f'ArUco simulation node: ID={self.marker_id}, dictionary={dictionary}, '
            f'marker_size={self.marker_size:.3f} m'
        )

    @staticmethod
    def fallback_camera_matrix(width, height):
        fx = width / (2.0 * np.tan(1.204 / 2.0))
        fy = fx
        return np.array(
            [[fx, 0.0, width / 2.0], [0.0, fy, height / 2.0], [0.0, 0.0, 1.0]],
            dtype=np.float64,
        )

    def camera_info_cb(self, msg):
        if msg.k[0] <= 0.0 or msg.k[4] <= 0.0:
            return
        self.camera_info = msg
        self.image_width = msg.width or self.image_width
        self.image_height = msg.height or self.image_height
        self.camera_matrix = np.array(msg.k, dtype=np.float64).reshape((3, 3))
        self.dist_coeffs = np.array(msg.d, dtype=np.float64).reshape((-1, 1))
        self.detector.set_camera_parameters(self.camera_matrix, self.dist_coeffs)

    @staticmethod
    def image_to_numpy(msg):
        encoding = msg.encoding.lower()
        if encoding in ('mono8', '8uc1'):
            channels = 1
        elif encoding in ('rgb8', 'bgr8'):
            channels = 3
        elif encoding in ('rgba8', 'bgra8'):
            channels = 4
        else:
            return None

        row = np.frombuffer(msg.data, dtype=np.uint8)
        if msg.step <= 0 or row.size < msg.step * msg.height:
            return None
        row = row[:msg.step * msg.height].reshape((msg.height, msg.step))
        image = row[:, :msg.width * channels]
        if channels > 1:
            image = image.reshape((msg.height, msg.width, channels))
            if encoding == 'rgb8':
                image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
            elif encoding == 'rgba8':
                image = cv2.cvtColor(image, cv2.COLOR_RGBA2BGR)
            elif encoding == 'bgra8':
                image = cv2.cvtColor(image, cv2.COLOR_BGRA2BGR)
        return image.copy()

    @staticmethod
    def depth_to_numpy(msg):
        encoding = msg.encoding.upper()
        if encoding in ('32FC1', 'R_FLOAT32'):
            dtype = np.float32
            scale = 1.0
        elif encoding in ('16UC1', 'R16_UINT'):
            dtype = np.uint16
            scale = 0.001
        else:
            return None
        data = np.frombuffer(msg.data, dtype=dtype)
        if msg.height <= 0 or msg.width <= 0 or data.size < msg.height * msg.width:
            return None
        return (data[:msg.height * msg.width].reshape((msg.height, msg.width)) * scale).copy()

    def depth_cb(self, msg):
        depth = self.depth_to_numpy(msg)
        if depth is not None:
            self.latest_depth = depth
            self.latest_depth_header = msg.header

    def image_cb(self, msg):
        image = self.image_to_numpy(msg)
        if image is None:
            self.get_logger().warning(f'Unsupported RGB encoding: {msg.encoding}')
            return

        self.frame_count += 1
        results = self.detector.process_frame(image)
        found = bool(results)
        self.detected_pub.publish(Bool(data=found))

        if found:
            self.detect_count += 1
            result = results[0]
            pose_info = result.get('pose')
            if pose_info is not None:
                pose = PoseStamped()
                pose.header = msg.header
                # OpenCV PnP uses x-right, y-down, z-forward. This is the
                # ROS optical-frame convention, not Gazebo camera_link.
                pose.header.frame_id = 'camera_optical_frame'
                pose.pose.position.x = pose_info['x']
                pose.pose.position.y = pose_info['y']
                pose.pose.position.z = pose_info['z']
                # ArUcoDetector returns quaternion in (w, x, y, z) order.
                qw, qx, qy, qz = pose_info['quaternion']
                pose.pose.orientation.x = qx
                pose.pose.orientation.y = qy
                pose.pose.orientation.z = qz
                pose.pose.orientation.w = qw
                self.pose_pub.publish(pose)

                point = PointStamped()
                point.header = pose.header
                point.point.x = pose_info['x']
                point.point.y = pose_info['y']
                point.point.z = pose_info['z']
                self.position_pub.publish(point)

                now_detection = time.monotonic()
                if now_detection - self.last_detection_log > 0.5:
                    self.get_logger().info(
                        f'[DETECTION] id={result["id"]} '
                        f'camera_optical xyz=('
                        f'{pose_info["x"]:.3f}, {pose_info["y"]:.3f}, '
                        f'{pose_info["z"]:.3f}) m '
                        f'distance={pose_info["distance"]:.3f} m'
                    )
                    self.last_detection_log = now_detection

            corners = result['corners']
            bbox = BoundingBox2D()
            bbox.center.position.x = float(np.mean(corners[:, 0]))
            bbox.center.position.y = float(np.mean(corners[:, 1]))
            bbox.center.theta = 0.0
            bbox.size_x = float(np.max(corners[:, 0]) - np.min(corners[:, 0]))
            bbox.size_y = float(np.max(corners[:, 1]) - np.min(corners[:, 1]))
            self.bbox_pub.publish(bbox)

            if self.latest_depth is not None and pose_info is not None:
                cx = bbox.center.position.x * self.latest_depth.shape[1] / msg.width
                cy = bbox.center.position.y * self.latest_depth.shape[0] / msg.height
                x0 = max(0, int(cx) - 3)
                x1 = min(self.latest_depth.shape[1], int(cx) + 4)
                y0 = max(0, int(cy) - 3)
                y1 = min(self.latest_depth.shape[0], int(cy) + 4)
                patch = self.latest_depth[y0:y1, x0:x1]
                valid = patch[np.isfinite(patch) & (patch > 0.1)]
                if valid.size:
                    pose_info['z_depth'] = float(np.median(valid))

        annotated = self.detector.draw_results(
            image, results, draw_axes=True, draw_bbox_mask=True, draw_summary_table=True
        )
        out = Image()
        out.header = msg.header
        out.height, out.width = annotated.shape[:2]
        out.encoding = 'bgr8'
        out.is_bigendian = False
        out.step = annotated.shape[1] * 3
        out.data = annotated.tobytes()
        self.annotated_pub.publish(out)

        now = time.monotonic()
        if now - self.last_log > 2.0:
            rate = self.detect_count / max(self.frame_count, 1) * 100.0
            self.get_logger().info(
                f'frames={self.frame_count}, detected={self.detect_count} '
                f'({rate:.1f}%), camera_info={self.camera_info is not None}'
            )
            self.last_log = now


def main():
    rclpy.init()
    node = ArucoSimulationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
