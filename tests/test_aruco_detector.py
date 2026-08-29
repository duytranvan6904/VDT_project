#!/usr/bin/env python3
"""
Unit Test: ArUco Detector & PnP Pose Estimation Verification with Synthetic Markers
"""

import unittest
import numpy as np
import cv2

from vision import ArUcoDetector, extract_bounding_box, rvec_to_euler, rvec_to_quaternion


class TestArUcoDetector(unittest.TestCase):

    def setUp(self):
        self.marker_size = 0.15  # 15 cm
        self.dict_name = "DICT_4X4_50"
        
        # Synthetic camera parameters
        self.fx = 615.0
        self.fy = 615.0
        self.cx = 320.0
        self.cy = 240.0
        self.camera_matrix = np.array([
            [self.fx, 0, self.cx],
            [0, self.fy, self.cy],
            [0, 0, 1]
        ], dtype=np.float64)
        self.dist_coeffs = np.zeros((5, 1), dtype=np.float64)
        
        self.detector = ArUcoDetector(
            dictionary_name=self.dict_name,
            marker_size_meters=self.marker_size,
            camera_matrix=self.camera_matrix,
            dist_coeffs=self.dist_coeffs
        )

    def test_synthetic_aruco_detection_and_pnp(self):
        """Test detection and PnP pose estimation on a rendered synthetic marker."""
        # Create blank 640x480 white canvas
        canvas = np.ones((480, 640, 3), dtype=np.uint8) * 255
        
        # Generate ArUco marker image (ID 0)
        marker_id = 0
        marker_pixel_size = 200
        
        if hasattr(cv2.aruco, "generateImageMarker"):
            marker_img = cv2.aruco.generateImageMarker(self.detector.aruco_dict, marker_id, marker_pixel_size)
        else:
            marker_img = cv2.aruco.drawMarker(self.detector.aruco_dict, marker_id, marker_pixel_size)
            
        marker_bgr = cv2.cvtColor(marker_img, cv2.COLOR_GRAY2BGR)
        
        # Place marker at center of canvas
        start_x = (640 - marker_pixel_size) // 2
        start_y = (480 - marker_pixel_size) // 2
        canvas[start_y:start_y+marker_pixel_size, start_x:start_x+marker_pixel_size] = marker_bgr
        
        # Run detection & PnP pose estimation
        results = self.detector.process_frame(canvas)
        
        self.assertTrue(len(results) > 0, "Failed to detect synthetic ArUco marker!")
        res = results[0]
        self.assertEqual(res["id"], marker_id, f"Expected marker ID {marker_id}, got {res['id']}")
        
        # Verify Pose estimation results
        self.assertIsNotNone(res["pose"], "PnP pose estimation returned None")
        pose = res["pose"]
        
        print(f"\n[UnitTest] Synthetic Marker ID:{res['id']} Pose: X={pose['x']:.4f}m, Y={pose['y']:.4f}m, Z={pose['z']:.4f}m")
        print(f"[UnitTest] Roll={pose['roll']:.2f}°, Pitch={pose['pitch']:.2f}°, Yaw={pose['yaw']:.2f}°")
        
        # Marker is directly in front of camera (X ~ 0, Y ~ 0, Z > 0)
        self.assertAlmostEqual(pose["x"], 0.0, delta=0.08)
        self.assertAlmostEqual(pose["y"], 0.0, delta=0.08)
        self.assertGreater(pose["z"], 0.1)

    def test_utils_conversions(self):
        """Test rotation vector to Euler/Quaternion utility functions."""
        rvec = np.array([0.0, 0.0, 0.0], dtype=np.float64)
        roll, pitch, yaw = rvec_to_euler(rvec)
        self.assertAlmostEqual(roll, 0.0)
        self.assertAlmostEqual(pitch, 0.0)
        self.assertAlmostEqual(yaw, 0.0)
        
        quat = rvec_to_quaternion(rvec)
        self.assertAlmostEqual(quat[0], 1.0)  # qw = 1.0 for zero rotation


if __name__ == "__main__":
    unittest.main()
