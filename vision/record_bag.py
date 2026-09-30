#!/usr/bin/env python3
"""
RealSense D430 Raw .bag Recorder Utility
Records uncompressed Infrared (IR1) and Depth streams to a .bag file for offline testing.

Usage examples:
    # 1. Record for 60 seconds (default) to recordings/d430_<timestamp>.bag:
    python3 vision/record_bag.py

    # 2. Record for 30 seconds to a custom filename:
    python3 vision/record_bag.py --output recordings/test_hpad_run1.bag --duration 30

    # 3. Headless recording without GUI window:
    python3 vision/record_bag.py --no-display --duration 60
"""

import os
import sys
import time
import argparse
import signal
import subprocess
from datetime import datetime

try:
    import pyrealsense2 as rs
    PYREALSENSE2_AVAILABLE = True
except ImportError:
    PYREALSENSE2_AVAILABLE = False

try:
    import cv2
    import numpy as np
    OPENCV_AVAILABLE = True
except ImportError:
    OPENCV_AVAILABLE = False


def parse_args():
    parser = argparse.ArgumentParser(
        description="Record raw uncompressed .bag files from RealSense D430 (IR1 + Depth)"
    )
    parser.add_argument(
        "--output", "-o",
        type=str,
        default="",
        help="Path to output .bag file (default: recordings/d430_capture_YYYYMMDD_HHMMSS.bag)"
    )
    parser.add_argument(
        "--duration", "-d",
        type=int,
        default=60,
        help="Recording duration in seconds (default: 60, use 0 for unlimited until stopped)"
    )
    parser.add_argument(
        "--width",
        type=int,
        default=640,
        help="Frame width (default: 640)"
    )
    parser.add_argument(
        "--height",
        type=int,
        default=480,
        help="Frame height (default: 480)"
    )
    parser.add_argument(
        "--fps",
        type=int,
        default=30,
        help="Frame rate (default: 30)"
    )
    parser.add_argument(
        "--no-display",
        action="store_true",
        help="Disable GUI preview window (useful for headless / SSH runs)"
    )
    parser.add_argument(
        "--force-cli",
        action="store_true",
        help="Force using /usr/local/bin/rs-record binary directly instead of Python SDK"
    )
    return parser.parse_args()


def record_with_cli_tool(output_path: str, duration: int) -> bool:
    """Fallback recording using the compiled /usr/local/bin/rs-record binary."""
    rs_record_bin = "/usr/local/bin/rs-record"
    if not os.path.exists(rs_record_bin):
        print(f"[ERROR] Neither pyrealsense2 nor {rs_record_bin} is available.")
        return False

    cmd = [rs_record_bin, "-f", output_path]
    if duration > 0:
        cmd.extend(["-t", str(duration)])

    print(f"[INFO] Launching native Librealsense recorder: {' '.join(cmd)}")
    print(f"[INFO] Target: RealSense D430 -> {output_path}")
    if duration > 0:
        print(f"[INFO] Recording for {duration} seconds...")
    else:
        print("[INFO] Recording until Ctrl+C is pressed...")

    try:
        proc = subprocess.run(cmd)
        if proc.returncode == 0:
            file_size_mb = os.path.getsize(output_path) / (1024 * 1024) if os.path.exists(output_path) else 0
            print(f"\n[SUCCESS] Recording completed cleanly!")
            print(f"  - Saved to: {output_path}")
            print(f"  - File size: {file_size_mb:.2f} MB")
            return True
        else:
            print(f"[ERROR] rs-record returned non-zero exit code: {proc.returncode}")
            return False
    except KeyboardInterrupt:
        print("\n[INFO] Recording stopped by user.")
        return True


def record_with_pyrealsense2(args, output_path: str) -> bool:
    """Record raw uncompressed .bag using pyrealsense2 Python SDK."""
    pipeline = rs.pipeline()
    config = rs.config()

    # Configure uncompressed raw recording
    config.enable_record_to_file(output_path)

    # RealSense D430: Left Infrared 1 (Y8) + Depth (Z16)
    config.enable_stream(rs.stream.infrared, 1, args.width, args.height, rs.format.y8, args.fps)
    config.enable_stream(rs.stream.depth, args.width, args.height, rs.format.z16, args.fps)

    print(f"[INFO] Initializing RealSense D430 hardware pipeline...")
    try:
        profile = pipeline.start(config)
        dev = profile.get_device()
        dev_name = dev.get_info(rs.camera_info.name)
        dev_sn = dev.get_info(rs.camera_info.serial_number)
        print(f"[INFO] Connected to: {dev_name} (S/N: {dev_sn})")
        print(f"[INFO] Stream config: IR1 (Y8) + Depth (Z16) @ {args.width}x{args.height} {args.fps} FPS")
        print(f"[INFO] Output file: {output_path}")
        print(f"[INFO] Press 'q' in preview window or Ctrl+C in terminal to stop.\n")
    except Exception as e:
        print(f"[ERROR] Failed to start pyrealsense2 pipeline: {e}")
        return False

    start_time = time.time()
    last_print_time = start_time
    frame_count = 0
    running = True

    def handle_sigint(sig, frame):
        nonlocal running
        print("\n[INFO] Stop requested (Ctrl+C). Finalizing bag file...")
        running = False

    signal.signal(signal.SIGINT, handle_sigint)

    try:
        while running:
            elapsed = time.time() - start_time
            if args.duration > 0 and elapsed >= args.duration:
                print(f"\n[INFO] Duration of {args.duration}s reached.")
                break

            try:
                frames = pipeline.wait_for_frames(timeout_ms=5000)
            except Exception as e:
                print(f"\n[WARN] Timeout waiting for frames: {e}")
                continue

            frame_count += 1
            ir_frame = frames.get_infrared_frame(1)
            depth_frame = frames.get_depth_frame()

            # Progress update in terminal every 1.0s
            curr_time = time.time()
            if curr_time - last_print_time >= 1.0:
                fps = frame_count / (curr_time - start_time)
                rem_str = f" | Remaining: {args.duration - int(elapsed)}s" if args.duration > 0 else ""
                sys.stdout.write(
                    f"\r[RECORDING] Elapsed: {int(elapsed)}s{rem_str} | Frames: {frame_count} | FPS: {fps:.1f}"
                )
                sys.stdout.flush()
                last_print_time = curr_time

            # Live preview window (if OpenCV available and display enabled)
            if not args.no_display and OPENCV_AVAILABLE and ir_frame and depth_frame:
                ir_img = np.asanyarray(ir_frame.get_data())
                depth_img = np.asanyarray(depth_frame.get_data())
                depth_vis = cv2.applyColorMap(cv2.convertScaleAbs(depth_img, alpha=0.03), cv2.COLORMAP_JET)
                ir_vis = cv2.cvtColor(ir_img, cv2.COLOR_GRAY2BGR)

                preview = np.hstack((ir_vis, depth_vis))
                cv2.putText(preview, f"REC: {int(elapsed)}s | {output_path}", (15, 30),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
                cv2.imshow("RealSense D430 Recorder Preview [IR1 | Depth]", preview)
                key = cv2.waitKey(1) & 0xFF
                if key == ord('q') or key == 27:
                    print("\n[INFO] User pressed 'q'. Finalizing bag file...")
                    break

    finally:
        pipeline.stop()
        if not args.no_display and OPENCV_AVAILABLE:
            cv2.destroyAllWindows()

    file_size_mb = os.path.getsize(output_path) / (1024 * 1024) if os.path.exists(output_path) else 0
    print(f"\n[SUCCESS] RealSense D430 .bag file saved cleanly!")
    print(f"  - Path     : {output_path}")
    print(f"  - Duration : {time.time() - start_time:.1f} s")
    print(f"  - Frames   : {frame_count}")
    print(f"  - Size     : {file_size_mb:.2f} MB")
    return True


def main():
    args = parse_args()

    # Determine output file path
    record_dir = "recordings"
    os.makedirs(record_dir, exist_ok=True)

    if not args.output:
        ts = datetime.now().strftime("%Y%m%d_%H%M%S")
        args.output = os.path.join(record_dir, f"d430_capture_{ts}.bag")
    else:
        out_dir = os.path.dirname(args.output)
        if out_dir:
            os.makedirs(out_dir, exist_ok=True)

    print("=" * 60)
    print(" RealSense D430 Raw .bag Recorder Utility")
    print(" Targets: Left Infrared (IR1) + Depth (Z16)")
    print(f" Output  : {args.output}")
    print(f" Duration: {args.duration}s (60s default)" if args.duration > 0 else " Duration: Unlimited")
    print(f" Spec    : {args.width}x{args.height} @ {args.fps} FPS")
    print("=" * 60)

    # If force-cli requested or pyrealsense2 not available, use compiled rs-record tool
    if args.force_cli or not PYREALSENSE2_AVAILABLE:
        if not PYREALSENSE2_AVAILABLE:
            print("[INFO] Python package pyrealsense2 not detected.")
            print("[INFO] Falling back to high-performance native binary /usr/local/bin/rs-record.")
        record_with_cli_tool(args.output, args.duration)
    else:
        record_with_pyrealsense2(args, args.output)


if __name__ == "__main__":
    main()
