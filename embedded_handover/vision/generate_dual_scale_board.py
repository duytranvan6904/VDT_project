#!/usr/bin/env python3
"""
Generate Dual-Scale ArUco Board for Precision Landing.
Design:
  - Big Coarse Marker: ID 42 (DICT_6X6_50), Size: 52cm x 52cm (0.52m)
  - Small Inner Marker: ID 43 (DICT_6X6_50), Size: 10cm x 10cm (0.10m)
  - Markers are separate; the small marker stays at the landing origin.
  - Total Pad Canvas: 118.9cm x 84.1cm (A0 landscape)

Outputs:
  - vision/dual_scale_aruco_board_A0_52_5cm.png (Texture & Printing)
  - vision/dual_scale_aruco_board_A0_52_5cm.pdf (Printable 1:1 PDF)
  - vision/dual_scale_board_config.yaml (OpenCV Board Definition)
"""

import os
import argparse
import numpy as np
import cv2
from PIL import Image
import yaml


def create_dual_scale_board(
    big_id: int = 42,
    big_size_m: float = 0.52,
    small_id: int = 43,
    small_size_m: float = 0.10,
    small_patch_m: float = 0.11,
    margin_m: float = 0.012,
    dict_name: str = "DICT_6X6_50",
    pixels_per_cm: int = 25,
    paper_width_m: float = 1.189,
    paper_height_m: float = 0.841,
):
    """
    Construct image and 3D corner coordinates for the dual-scale board.
    
    Args:
        big_id: ArUco ID of the outer marker
        big_size_m: Physical width/height of big marker in meters
        small_id: ArUco ID of the inner marker
        small_size_m: Physical width/height of small marker in meters
        small_patch_m: White background patch width/height for small marker in meters
        margin_m: Outer white quiet zone margin width in meters
        dict_name: OpenCV ArUco dictionary name
        pixels_per_cm: Resolution scale (25 px/cm gives 1300px for 52cm marker)
        
    Returns:
        canvas: 2D uint8 numpy array (grayscale image)
        board_config: dict containing 3D object points and metadata
    """
    if not hasattr(cv2.aruco, dict_name):
        raise ValueError(f"Unknown dictionary name: {dict_name}")
    
    dict_id = getattr(cv2.aruco, dict_name)
    dictionary = cv2.aruco.getPredefinedDictionary(dict_id)
    
    # Calculate pixel dimensions
    big_px = int(round(big_size_m * 100 * pixels_per_cm))
    small_px = int(round(small_size_m * 100 * pixels_per_cm))
    small_patch_px = int(round(small_patch_m * 100 * pixels_per_cm))
    margin_px = int(round(margin_m * 100 * pixels_per_cm))
    
    total_w_px = int(round(paper_width_m * 100 * pixels_per_cm))
    total_h_px = int(round(paper_height_m * 100 * pixels_per_cm))
    if big_px + 2 * margin_px > min(total_w_px, total_h_px):
        raise ValueError("The large marker and margin do not fit on the A0 canvas")
    big_x = margin_px
    big_y = (total_h_px - big_px) // 2
    small_x = (total_w_px - small_patch_px) // 2
    small_y = (total_h_px - small_patch_px) // 2
    if big_x + big_px >= small_x:
        raise ValueError("The large marker must not overlap the centered landing marker")
    
    # Cross-version ArUco marker drawing
    def _draw_marker(dict_obj, marker_id, size):
        if hasattr(cv2.aruco, "generateImageMarker"):
            return cv2.aruco.generateImageMarker(dict_obj, marker_id, size)
        elif hasattr(cv2.aruco, "drawMarker"):
            return cv2.aruco.drawMarker(dict_obj, marker_id, size)
        raise AttributeError("cv2.aruco has neither generateImageMarker nor drawMarker")

    # 1. Draw outer big marker
    big_marker = _draw_marker(dictionary, big_id, big_px)
    
    # 2. Draw inner small marker
    small_marker = _draw_marker(dictionary, small_id, small_px)
    
    # 3. Create white quiet zone patch for inner marker
    patch = np.ones((small_patch_px, small_patch_px), dtype=np.uint8) * 255
    offset = (small_patch_px - small_px) // 2
    patch[offset:offset + small_px, offset:offset + small_px] = small_marker
    
    # 4. Place both tags on an A0 landscape sheet without overlapping either code.
    # The coarse tag sits at left; the fine tag stays centered on the landing origin.
    canvas = np.ones((total_h_px, total_w_px), dtype=np.uint8) * 255
    canvas[big_y:big_y + big_px, big_x:big_x + big_px] = big_marker
    canvas[small_y:small_y + small_patch_px, small_x:small_x + small_patch_px] = patch
    
    # 6. Calculate 3D object points in Board Frame
    # Origin (0,0,0) is defined at the exact geometric center of the pad.
    # Standard OpenCV corner order: Top-Left, Top-Right, Bottom-Right, Bottom-Left
    # X points Right, Y points Up (or standard image plane: X right, Y down/up).
    # In OpenCV ArUco Board standard:
    # Top-Left:  [-size/2,  size/2, 0]
    # Top-Right: [ size/2,  size/2, 0]
    # Bottom-Right: [ size/2, -size/2, 0]
    # Bottom-Left:  [-size/2, -size/2, 0]
    
    hb = big_size_m / 2.0
    big_center_x = -paper_width_m / 2.0 + margin_m + hb
    big_corners_3d = [
        [big_center_x - hb,  hb, 0.0],
        [big_center_x + hb,  hb, 0.0],
        [big_center_x + hb, -hb, 0.0],
        [big_center_x - hb, -hb, 0.0]
    ]
    
    hs = small_size_m / 2.0
    small_corners_3d = [
        [-hs,  hs, 0.0],
        [ hs,  hs, 0.0],
        [ hs, -hs, 0.0],
        [-hs, -hs, 0.0]
    ]
    
    board_config = {
        "dictionary": dict_name,
        "board_type": "a0_landscape_dual_scale",
        "total_pad_size_m": float(paper_height_m),
        "total_pad_width_m": float(paper_width_m),
        "total_pad_height_m": float(paper_height_m),
        "markers": [
            {
                "id": int(big_id),
                "role": "outer_coarse",
                "size_m": float(big_size_m),
                "effective_range_m": [0.8, 5.0],
                "corners_3d": big_corners_3d
            },
            {
                "id": int(small_id),
                "role": "inner_fine",
                "size_m": float(small_size_m),
                "effective_range_m": [0.05, 1.2],
                "corners_3d": small_corners_3d
            }
        ]
    }
    
    return canvas, board_config


def main():
    parser = argparse.ArgumentParser(description="Generate Dual-Scale ArUco Board for Precision Landing")
    parser.add_argument("--big-id", type=int, default=42, help="Outer marker ID (default: 42)")
    parser.add_argument("--big-size", type=float, default=0.52, help="Outer marker size in meters (default: 0.52 = 52cm)")
    parser.add_argument("--small-id", type=int, default=43, help="Inner marker ID (default: 43)")
    parser.add_argument("--small-size", type=float, default=0.10, help="Inner marker size in meters (default: 0.10 = 10cm)")
    parser.add_argument("--small-patch", type=float, default=0.11, help="Inner marker white patch size in meters (default: 0.11 = 11cm)")
    parser.add_argument("--margin", type=float, default=0.012, help="Large marker edge margin in meters")
    parser.add_argument("--paper-width", type=float, default=1.189, help="A0 landscape width in meters")
    parser.add_argument("--paper-height", type=float, default=0.841, help="A0 landscape height in meters")
    parser.add_argument("--pixels-per-cm", type=int, default=25, help="Texture resolution in pixels per cm")
    parser.add_argument("--dict", type=str, default="DICT_6X6_50", help="ArUco dictionary (default: DICT_6X6_50)")
    parser.add_argument("--output-dir", type=str, default="vision", help="Output directory")
    args = parser.parse_args()
    
    os.makedirs(args.output_dir, exist_ok=True)
    
    print(f"Generating Dual-Scale ArUco Board:")
    print(f"  • Dictionary: {args.dict}")
    print(f"  • Outer Marker: ID {args.big_id}, Size {args.big_size*100:.1f}cm")
    print(f"  • Inner Marker: ID {args.small_id}, Size {args.small_size*100:.1f}cm (patch {args.small_patch*100:.1f}cm)")
    print(f"  • Landing Sheet: {args.paper_width*100:.1f}cm x {args.paper_height*100:.1f}cm (A0 landscape)")
    
    canvas, board_config = create_dual_scale_board(
        big_id=args.big_id,
        big_size_m=args.big_size,
        small_id=args.small_id,
        small_size_m=args.small_size,
        small_patch_m=args.small_patch,
        margin_m=args.margin,
        dict_name=args.dict,
        pixels_per_cm=args.pixels_per_cm,
        paper_width_m=args.paper_width,
        paper_height_m=args.paper_height,
    )
    
    # 1. Save PNG
    png_path = os.path.join(args.output_dir, "dual_scale_aruco_board_A0_52_5cm.png")
    cv2.imwrite(png_path, canvas)
    print(f"  -> Saved PNG: {png_path} ({canvas.shape[1]}x{canvas.shape[0]} px)")
    
    # 2. Save Printable PDF (with 1:1 scale DPI)
    dpi = args.pixels_per_cm / 0.393701
    pdf_path = os.path.join(args.output_dir, "dual_scale_aruco_board_A0_52_5cm.pdf")
    pil_img = Image.fromarray(canvas)
    pil_img.save(pdf_path, "PDF", resolution=dpi)
    print(f"  -> Saved 1:1 Printable PDF: {pdf_path}")
    
    # 3. Save YAML configuration
    yaml_path = os.path.join(args.output_dir, "dual_scale_board_config.yaml")
    with open(yaml_path, "w") as f:
        yaml.dump(board_config, f, default_flow_style=None, sort_keys=False)
    print(f"  -> Saved Board Config: {yaml_path}")
    
    # 4. Verify detection on generated board
    dictionary = getattr(cv2.aruco, args.dict)
    d = cv2.aruco.getPredefinedDictionary(dictionary)
    corners, ids, _ = cv2.aruco.detectMarkers(canvas, d)
    detected_ids = ids.flatten().tolist() if ids is not None else []
    print(f"\nVerification on generated image: detected IDs = {detected_ids}")
    assert set([args.big_id, args.small_id]).issubset(set(detected_ids)), (
        f"Validation failed: expected {[args.big_id, args.small_id]}, got {detected_ids}"
    )
    print("[SUCCESS] Dual-Scale ArUco Board created and verified successfully!")


if __name__ == "__main__":
    main()
