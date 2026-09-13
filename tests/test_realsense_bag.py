#!/usr/bin/env python3
"""
Unit Test: RealSense D430 .bag Recording & Playback Interface Verification
Tests parameter initialization, file validation, and CLI configuration for D430.
"""

import os
import unittest
import numpy as np

from vision.realsense_stream import RealSenseCamera
from main_aruco_detector import parse_args as parse_detector_args
from vision.record_bag import parse_args as parse_recorder_args


class TestRealSenseBagIntegration(unittest.TestCase):

    def test_d430_camera_init_parameters(self):
        """Verify D430 defaults and .bag configuration parameters."""
        cam = RealSenseCamera(
            width=640,
            height=480,
            fps=30,
            enable_depth=True,
            prefer_infrared=True,
            record_to_file="recordings/test_record.bag",
            playback_bag_file="recordings/test_playback.bag",
            repeat_playback=False
        )

        self.assertEqual(cam.width, 640)
        self.assertEqual(cam.height, 480)
        self.assertEqual(cam.fps, 30)
        self.assertTrue(cam.prefer_infrared, "D430 must prioritize Infrared 1 (IR1) by default.")
        self.assertEqual(cam.record_to_file, "recordings/test_record.bag")
        self.assertEqual(cam.playback_bag_file, "recordings/test_playback.bag")
        self.assertFalse(cam.repeat_playback)
        self.assertFalse(cam.is_rs_active)
        self.assertFalse(cam.is_playback)

    def test_nonexistent_bag_playback_fails_gracefully(self):
        """Verify starting playback with a non-existent file fails gracefully without crashing."""
        cam = RealSenseCamera(
            playback_bag_file="/path/to/nonexistent_file_12345.bag"
        )
        success = cam.start()
        self.assertFalse(success, "Starting non-existent .bag file must return False.")
        self.assertFalse(cam.is_rs_active)

    def test_detector_cli_bag_arguments(self):
        """Verify main_aruco_detector CLI correctly accepts --bag and --record-bag."""
        test_args = [
            "--bag", "recordings/sample_flight.bag",
            "--record-bag", "recordings/out.bag",
            "--no-repeat-bag",
            "--no-display"
        ]
        import sys
        old_argv = sys.argv
        try:
            sys.argv = ["main_aruco_detector.py"] + test_args
            parsed = parse_detector_args()
            self.assertEqual(parsed.bag, "recordings/sample_flight.bag")
            self.assertEqual(parsed.record_bag, "recordings/out.bag")
            self.assertTrue(parsed.no_repeat_bag)
            self.assertTrue(parsed.no_display)
        finally:
            sys.argv = old_argv

    def test_recorder_cli_arguments(self):
        """Verify record_bag.py CLI arguments for D430 capture."""
        test_args = [
            "--output", "recordings/custom_test.bag",
            "--duration", "60",
            "--no-display"
        ]
        import sys
        old_argv = sys.argv
        try:
            sys.argv = ["record_bag.py"] + test_args
            parsed = parse_recorder_args()
            self.assertEqual(parsed.output, "recordings/custom_test.bag")
            self.assertEqual(parsed.duration, 60)
            self.assertTrue(parsed.no_display)
        finally:
            sys.argv = old_argv


if __name__ == "__main__":
    unittest.main()
