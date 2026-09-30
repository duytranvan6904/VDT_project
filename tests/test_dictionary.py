#!/usr/bin/env python3
"""
ArUco Dictionary & Multi-Marker Diagnostic Scanner
Author: Duy (Vision Lead) - VDT Project

Usage:
  python3 test_dictionary.py --image /path/to/image.jpg
"""

import sys
import argparse
import cv2
import numpy as np

DICTS_TO_TEST = [
    "DICT_4X4_50",
    "DICT_4X4_100",
    "DICT_4X4_250",
    "DICT_5X5_50",
    "DICT_5X5_100",
    "DICT_5X5_250",
    "DICT_6X6_50",
    "DICT_6X6_100",
    "DICT_6X6_250",
    "DICT_ARUCO_ORIGINAL"
]


def test_image(img_path: str):
    image = cv2.imread(img_path)
    if image is None:
        print(f"[ERROR] Could not load image from '{img_path}'. Please check file path!")
        return

    print(f"\n========================================================")
    print(f" Diagnostic Image Analysis: {img_path}")
    print(f" Resolution: {image.shape[1]} x {image.shape[0]} pixels")
    print(f"========================================================")

    for dict_name in DICTS_TO_TEST:
        dict_id = getattr(cv2.aruco, dict_name)
        if hasattr(cv2.aruco, "getPredefinedDictionary"):
            aruco_dict = cv2.aruco.getPredefinedDictionary(dict_id)
        else:
            aruco_dict = cv2.aruco.Dictionary_get(dict_id)

        if hasattr(cv2.aruco, "DetectorParameters_create"):
            params = cv2.aruco.DetectorParameters_create()
        elif hasattr(cv2.aruco, "DetectorParameters"):
            params = cv2.aruco.DetectorParameters()
        else:
            params = None

        if params is not None:
            params.adaptiveThreshWinSizeMin = 3
            params.adaptiveThreshWinSizeMax = 53
            params.adaptiveThreshWinSizeStep = 4
            params.minMarkerPerimeterRate = 0.01
            params.maxMarkerPerimeterRate = 4.0
            params.minDistanceToBorder = 0
            if hasattr(cv2.aruco, "CORNER_REFINE_SUBPIX"):
                params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX

        if hasattr(cv2.aruco, "ArucoDetector"):
            detector = cv2.aruco.ArucoDetector(aruco_dict, params)
            corners, ids, rejected = detector.detectMarkers(image)
        else:
            corners, ids, rejected = cv2.aruco.detectMarkers(image, aruco_dict, parameters=params)

        if ids is not None and len(ids) > 0:
            id_list = ids.flatten().tolist()
            print(f"[SUCCESS] Dictionary [{dict_name:19s}] -> DETECTED {len(ids)} marker(s)! IDs: {id_list}")
            for idx, mid in enumerate(id_list):
                c = corners[idx].reshape((4, 2))
                center_x = float(np.mean(c[:, 0]))
                center_y = float(np.mean(c[:, 1]))
                print(f"    - Marker ID {mid:2d} at pixel position: Center=({center_x:.1f}, {center_y:.1f})")
        else:
            rej_count = len(rejected) if rejected is not None else 0
            print(f"[ FAILED] Dictionary [{dict_name:19s}] -> 0 markers detected. ({rej_count} candidate shapes rejected)")

    print("========================================================\n")


def main():
    parser = argparse.ArgumentParser(description="Test ArUco Dictionary against image")
    parser.add_argument("--image", type=str, required=True, help="Path to input image file")
    args = parser.parse_args()

    test_image(args.image)


if __name__ == "__main__":
    main()
