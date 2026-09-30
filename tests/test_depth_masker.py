#!/usr/bin/env python3
"""
Unit Test: Depth Masking Processor Verification (Task N3)
"""

import unittest
import numpy as np
import cv2

from vision import DepthMasker


class MockRealSenseDepthFrame:
    """Mock object simulating pyrealsense2 rs.depth_frame C++ object."""
    def __init__(self, data_array: np.ndarray):
        self._data = data_array

    def get_data(self):
        return self._data


class TestDepthMasker(unittest.TestCase):

    def setUp(self):
        self.masker = DepthMasker(margin_percent=0.15)
        self.img_h, self.img_w = 480, 640
        
        # Fake synthetic depth image (2000 mm everywhere = 2.0 meters)
        self.synthetic_depth = np.ones((self.img_h, self.img_w), dtype=np.uint16) * 2000
        
        # Fake detection result for marker ID 0 placed at center of image
        self.fake_corners = np.array([
            [270, 190],
            [370, 190],
            [370, 290],
            [270, 290]
        ], dtype=np.float32)
        
        self.fake_detection_results = [{
            "id": 0,
            "corners": self.fake_corners,
            "bbox": (255, 175, 130, 130),
            "pose": None
        }]

    def test_depth_mask_creation_and_zero_out(self):
        """Test that depth pixels inside H-Pad polygon are zeroed out and outside are untouched."""
        masked_depth, binary_mask = self.masker.mask_depth_frame(
            self.synthetic_depth,
            self.fake_detection_results,
            fill_value=0,
            use_polygon=True
        )

        self.assertEqual(binary_mask.shape, (self.img_h, self.img_w))
        self.assertTrue(np.any(binary_mask == 255), "Binary mask should have non-zero masked regions")
        
        # Center pixel (320, 240) is inside the H-Pad -> must be 0
        self.assertEqual(masked_depth[240, 320], 0, "H-Pad center depth should be zeroed out!")

        # Top-left corner pixel (10, 10) is outside H-Pad -> must remain 2000 mm
        self.assertEqual(masked_depth[10, 10], 2000, "Background depth pixel outside H-Pad must remain untouched!")

    def test_realsense_depth_frame_object(self):
        """Test that rs.depth_frame object with .get_data() method is correctly converted and masked."""
        rs_frame = MockRealSenseDepthFrame(self.synthetic_depth)
        masked_depth, binary_mask = self.masker.mask_depth_frame(
            rs_frame,
            self.fake_detection_results,
            fill_value=0
        )

        self.assertIsNotNone(masked_depth)
        self.assertEqual(masked_depth[240, 320], 0)
        self.assertEqual(masked_depth[10, 10], 2000)

    def test_visualization_overlay(self):
        """Test depth mask visualization overlay generation."""
        dummy_vis = np.zeros((self.img_h, self.img_w, 3), dtype=np.uint8)
        _, binary_mask = self.masker.mask_depth_frame(self.synthetic_depth, self.fake_detection_results)
        
        vis_result = self.masker.visualize_masked_depth(dummy_vis, binary_mask)
        self.assertEqual(vis_result.shape, (self.img_h, self.img_w, 3))


if __name__ == "__main__":
    unittest.main()
