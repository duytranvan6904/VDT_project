#!/usr/bin/env python3
"""
Generate Dual-Scale ArUco Board for Precision Landing.
Design:
  - Big Outer Marker: ID 42 (DICT_6X6_50), Size: 40cm x 40cm (0.40m)
  - Small Inner Marker: ID 43 (DICT_6X6_50), Size: 5cm x 5cm (0.05m)
  - Small Marker White Quiet Zone: 6cm x 6cm (0.06m) centered
  - Total Pad Canvas (with 5cm outer margin): 50cm x 50cm (0.50m)

Outputs:
  - vision/dual_scale_aruco_board_40_5cm.png (Texture & Printing)
  - vision/dual_scale_aruco_board_40_5cm.pdf (Printable 1:1 PDF)
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
    big_size_m: float = 0.40,
    small_id: int = 43,
    small_size_m: float = 0.05,
    small_patch_m: float = 0.06,
    margin_m: float = 0.05,
    dict_name: str = "DICT_6X6_50",
    pixels_per_cm: int = 50,
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
        pixels_per_cm: Resolution scale (50 px/cm gives 2000px for 40cm marker)
        
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
    
    total_px = big_px + 2 * margin_px
    
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
    
    # 4. Embed inner patch exactly at the center of the outer marker
    board_img = big_marker.copy()
    center = big_px // 2
    patch_half = small_patch_px // 2
    board_img[
        center - patch_half:center - patch_half + small_patch_px,
        center - patch_half:center - patch_half + small_patch_px
    ] = patch
    
    # 5. Place board onto full canvas with outer white margin
    canvas = np.ones((total_px, total_px), dtype=np.uint8) * 255
    canvas[margin_px:margin_px + big_px, margin_px:margin_px + big_px] = board_img
    
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
    big_corners_3d = [
        [-hb,  hb, 0.0],
        [ hb,  hb, 0.0],
        [ hb, -hb, 0.0],
        [-hb, -hb, 0.0]
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
        "board_type": "nested_dual_scale",
        "total_pad_size_m": float(big_size_m + 2 * margin_m),
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
    parser.add_argument("--big-size", type=float, default=0.40, help="Outer marker size in meters (default: 0.40 = 40cm)")
    parser.add_argument("--small-id", type=int, default=43, help="Inner marker ID (default: 43)")
    parser.add_argument("--small-size", type=float, default=0.05, help="Inner marker size in meters (default: 0.05 = 5cm)")
    parser.add_argument("--small-patch", type=float, default=0.06, help="Inner marker white patch size in meters (default: 0.06 = 6cm)")
    parser.add_argument("--margin", type=float, default=0.05, help="Canvas outer white margin in meters (default: 0.05 = 5cm)")
    parser.add_argument("--dict", type=str, default="DICT_6X6_50", help="ArUco dictionary (default: DICT_6X6_50)")
    parser.add_argument("--output-dir", type=str, default="vision", help="Output directory")
    args = parser.parse_args()
    
    os.makedirs(args.output_dir, exist_ok=True)
    
    print(f"Generating Dual-Scale ArUco Board:")
    print(f"  • Dictionary: {args.dict}")
    print(f"  • Outer Marker: ID {args.big_id}, Size {args.big_size*100:.1f}cm")
    print(f"  • Inner Marker: ID {args.small_id}, Size {args.small_size*100:.1f}cm (patch {args.small_patch*100:.1f}cm)")
    print(f"  • Total Landing Pad Size: {(args.big_size + 2*args.margin)*100:.1f}cm x {(args.big_size + 2*args.margin)*100:.1f}cm")
    
    canvas, board_config = create_dual_scale_board(
        big_id=args.big_id,
        big_size_m=args.big_size,
        small_id=args.small_id,
        small_size_m=args.small_size,
        small_patch_m=args.small_patch,
        margin_m=args.margin,
        dict_name=args.dict,
        pixels_per_cm=50  # 2500 x 2500 px
    )
    
    # 1. Save PNG
    png_path = os.path.join(args.output_dir, "dual_scale_aruco_board_40_5cm.png")
    cv2.imwrite(png_path, canvas)
    print(f"  -> Saved PNG: {png_path} ({canvas.shape[1]}x{canvas.shape[0]} px)")
    
    # 2. Save Printable PDF (with 1:1 scale DPI)
    # 50 px/cm = 127 DPI
    dpi = int(50 / 0.393701) # 127 DPI
    pdf_path = os.path.join(args.output_dir, "dual_scale_aruco_board_40_5cm.pdf")
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
