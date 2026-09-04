#!/usr/bin/env python3
"""
Main Executable Script: ArUco Detection, PnP Pose Estimation & Depth Masking with Intel RealSense D435
Author: Duy (Vision/Estimation Lead) - VDT Quadrotor H-Pad Project
"""

import sys
import time
import argparse
import numpy as np
import cv2

from vision import RealSenseCamera, ArUcoDetector, DepthMasker, rvec_to_euler


def parse_marker_sizes_arg(arg_str: str, default_size: float):
    """Parse marker sizes argument."""
    if not arg_str:
        return default_size
    if ":" not in arg_str:
        try:
            return float(arg_str)
        except ValueError:
            return default_size
    
    size_dict = {-1: default_size}
    pairs = arg_str.split(",")
    for p in pairs:
        if ":" in p:
            parts = p.split(":")
            try:
                mid = int(parts[0].strip())
                msize = float(parts[1].strip())
                size_dict[mid] = msize
            except ValueError:
                pass
    return size_dict


def parse_args():
    parser = argparse.ArgumentParser(
        description="RealSense D435 ArUco Detection, PnP Pose Estimation & Depth Masking"
    )
    parser.add_argument(
        "--marker-size",
        type=float,
        default=0.15,
        help="Default side length of ArUco markers in meters (default: 0.15 = 15cm)"
    )
    parser.add_argument(
        "--marker-sizes",
        type=str,
        default="",
        help="Per-marker size mapping 'ID:size_m,ID:size_m' (e.g. '0:0.15,1:0.05,2:0.05')"
    )
    parser.add_argument(
        "--dict",
        type=str,
        default="DICT_4X4_50",
        help="ArUco dictionary name (default: DICT_4X4_50)"
    )
    parser.add_argument(
        "--margin-percent",
        type=float,
        default=0.15,
        help="Depth Mask expansion margin (default: 0.15 = 15%)"
    )
    parser.add_argument(
        "--width",
        type=int,
        default=640,
        help="Camera resolution width (default: 640)"
    )
    parser.add_argument(
        "--height",
        type=int,
        default=480,
        help="Camera resolution height (default: 480)"
    )
    parser.add_argument(
        "--fps",
        type=int,
        default=30,
        help="Camera FPS (default: 30)"
    )
    parser.add_argument(
        "--no-display",
        action="store_true",
        help="Run in headless mode without GUI window"
    )
    return parser.parse_args()


def main():
    args = parse_args()

    marker_sizes = parse_marker_sizes_arg(args.marker_sizes, args.marker_size)

    print("==========================================================")
    print(" Quadrotor H-Pad Vision Pipeline - Task Vision Lead (Duy)")
    print(" ArUco Detection, PnP Estimation & Depth Masking (N3)")
    print("==========================================================")
    print(f" Default Size  : {args.marker_size * 100:.1f} cm ({args.marker_size} m)")
    print(f" Custom Sizes  : {marker_sizes}")
    print(f" Dictionary    : {args.dict}")
    print(f" Mask Margin   : {args.margin_percent * 100:.0f}% expansion")
    print(f" Stream Spec   : {args.width}x{args.height} @ {args.fps} FPS")
    print("==========================================================")

    # 1. Initialize RealSense Camera
    camera = RealSenseCamera(
        width=args.width,
        height=args.height,
        fps=args.fps,
        enable_depth=True
    )

    if not camera.start():
        print("[ERROR] Failed to start RealSense camera pipeline.")
        sys.exit(1)

    # 2. Initialize ArUco Detector & Depth Masker
    detector = ArUcoDetector(
        dictionary_name=args.dict,
        marker_size_meters=marker_sizes,
        camera_matrix=camera.camera_matrix,
        dist_coeffs=camera.dist_coeffs
    )

    depth_masker = DepthMasker(margin_percent=args.margin_percent)

    prev_time = time.time()
    frame_count = 0
    fps = 0.0

    print("\n[INFO] Stream active. Key Controls:")
    print("  - Press 'q' or ESC: Quit program")
    print("  - Press 'd': Toggle Depth Mask Bounding Box")
    print("  - Press 'a': Toggle 3D Axis drawing\n")

    draw_bbox_mask = True
    draw_axes = True

    try:
        while True:
            ret, color_img, depth_frame, depth_vis = camera.get_frame()

            if not ret or color_img is None or color_img.size == 0:
                time.sleep(0.01)
                continue

            detector.set_camera_parameters(camera.camera_matrix, camera.dist_coeffs)

            # Process frame: Detect ArUco markers & compute PnP Pose Estimation
            results = detector.process_frame(color_img)

            # Perform Depth Masking (Task N3): Exclude H-Pad regions from Depth Map
            masked_depth_frame, binary_mask = depth_masker.mask_depth_frame(
                depth_frame,
                results,
                fill_value=0,
                use_polygon=True
            )

            # Measure RealSense Depth at marker center for double-validation
            for res in results:
                if res["pose"] is not None:
                    pts = res["corners"]
                    cx, cy = int(np.mean(pts[:, 0])), int(np.mean(pts[:, 1]))
                    z_sensor = camera.get_depth_at_pixel(depth_frame, cx, cy)
                    res["pose"]["z_sensor"] = z_sensor

            # Calculate FPS
            curr_time = time.time()
            frame_count += 1
            if curr_time - prev_time >= 1.0:
                fps = frame_count / (curr_time - prev_time)
                frame_count = 0
                prev_time = curr_time

            # Render clean annotations on color frame
            annotated_frame = detector.draw_results(
                color_img,
                results,
                draw_axes=draw_axes,
                draw_bbox_mask=draw_bbox_mask,
                draw_summary_table=True
            )

            # Overlay HUD Header
            status_text = f"FPS: {fps:.1f} | Detected: {len(results)} markers | Dict: {args.dict}"
            cv2.rectangle(annotated_frame, (10, 10), (450, 40), (0, 0, 0), -1)
            cv2.putText(annotated_frame, status_text, (15, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 255), 2)

            # Visualize Depth Stream with Masked Region Overlay
            if depth_vis is not None:
                depth_masked_vis = depth_masker.visualize_masked_depth(depth_vis, binary_mask)
            else:
                depth_masked_vis = None

            # Display windows (unless running headless)
            if not args.no_display:
                cv2.imshow("RealSense D435 - ArUco PnP Pose Estimation", annotated_frame)

                if depth_masked_vis is not None:
                    cv2.imshow("RealSense D435 - Masked Depth Stream (Task N3)", depth_masked_vis)

                key = cv2.waitKey(1) & 0xFF
                if key == ord('q') or key == 27:
                    print("[INFO] User requested stop.")
                    break
                elif key == ord('d'):
                    draw_bbox_mask = not draw_bbox_mask
                    print(f"[INFO] Bounding Box Mask rendering: {draw_bbox_mask}")
                elif key == ord('a'):
                    draw_axes = not draw_axes
                    print(f"[INFO] 3D Axis rendering: {draw_axes}")
            else:
                time.sleep(0.03)
                if frame_count >= 30 and args.no_display:
                    print("\n[INFO] Headless test run completed (30 frames executed). Exiting.")
                    break

    except KeyboardInterrupt:
        print("\n[INFO] Interrupted by keyboard.")

    finally:
        camera.stop()
        if not args.no_display:
            cv2.destroyAllWindows()
        print("[INFO] ArUco Detection, PnP Estimation & Depth Masking task finished successfully.")


if __name__ == "__main__":
    main()
