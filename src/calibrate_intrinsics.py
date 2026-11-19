#!/usr/bin/env python3
"""Step 3: solve each camera's intrinsics from its captured ChArUco frames.

Writes `camera_matrix` and `distortion_coeff` to `<images>/<camera>/camera.json`.
"""
from pathlib import Path
from typing import Optional

import cv2
import numpy as np

from calib_config import CalibCfg, camera_dir, camera_json_path, camera_names
from charuco import build_charuco_detector, detect_board
from json_io import to_json


def calibrate_single_camera(cam_name: str, image_dir: Path, output_json: Path) -> Optional[dict]:
    """Calibrate one camera from `image_dir`; returns the camera.json dict, or None on failure."""
    images = sorted(Path(image_dir).glob(f"*.{CalibCfg.IMAGE_EXT}"))
    if not images:
        print(f"[{cam_name}] No images found in {image_dir}. Skipping.")
        return None

    detector, board = build_charuco_detector()
    all_corners, all_ids = [], []
    image_size = None

    print(f"\n[{cam_name}] Processing {len(images)} images from '{image_dir}'...")
    for path in images:
        img = cv2.imread(str(path))
        if img is None:
            print(f"  [SKIPPED]  {path.name}: unreadable")
            continue

        size = (img.shape[1], img.shape[0])
        if image_size is None:
            image_size = size
        elif size != image_size:
            print(f"  [SKIPPED]  {path.name}: size {size} != {image_size}")
            continue

        detection = detect_board(detector, img)
        if detection.is_valid and len(detection.corners) == len(detection.ids):
            all_corners.append(np.ascontiguousarray(detection.corners, np.float32).reshape(-1, 1, 2))
            all_ids.append(np.ascontiguousarray(detection.ids, np.int32).reshape(-1, 1))
            print(f"  [ACCEPTED] {path.name}: {detection.count} corners")
        else:
            print(f"  [REJECTED] {path.name}: {detection.count} corners (< {CalibCfg.MIN_CORNERS})")

    n_valid = len(all_corners)
    if n_valid < CalibCfg.MIN_FRAMES:
        print(f"[ERROR] [{cam_name}] Only {n_valid} valid frames; need >= {CalibCfg.MIN_FRAMES}.")
        return None

    rational = n_valid >= CalibCfg.RATIONAL_MODEL_MIN_FRAMES
    rms, camera_matrix, dist_coeffs, _, _ = cv2.aruco.calibrateCameraCharuco(
        charucoCorners=all_corners,
        charucoIds=all_ids,
        board=board,
        imageSize=image_size,
        cameraMatrix=None,
        distCoeffs=None,
        flags=cv2.CALIB_RATIONAL_MODEL if rational else 0,
    )

    # OpenCV pads the rational model to 14 values; the last 6 (thin-prism, tilt) are fixed
    # at zero. Keep only the 8 that were actually fitted.
    dist_coeffs = dist_coeffs.flatten()[:8 if rational else 5]

    print("-" * 55)
    print(f"[{cam_name}] Calibration results")
    print(f"  Image size:               {image_size[0]} x {image_size[1]}")
    print(f"  Valid frames used:        {n_valid} / {len(images)}")
    print(f"  RMS reprojection error:   {rms:.4f} px")
    print(f"  Focal length (fx, fy):    ({camera_matrix[0, 0]:.2f}, {camera_matrix[1, 1]:.2f}) px")
    print(f"  Principal point (cx, cy): ({camera_matrix[0, 2]:.2f}, {camera_matrix[1, 2]:.2f}) px")
    print(f"  Distortion model:         {'rational (8 coeffs)' if rational else 'plumb_bob (5 coeffs)'}")
    print("-" * 55)

    # Same schema as mvs_extrinsic_calib's camera.json; calibrate_extrinsics.py adds the pose.
    calib = {
        "camera_matrix": camera_matrix.tolist(),
        "distortion_coeff": dist_coeffs.tolist(),
        "camera": cam_name,
    }
    Path(output_json).parent.mkdir(parents=True, exist_ok=True)
    to_json(output_json, calib)
    print(f"[{cam_name}] Saved: {output_json}")
    return calib


def main() -> None:
    for cam_name in camera_names():
        calibrate_single_camera(cam_name, camera_dir(cam_name), camera_json_path(cam_name))
    print("\n[DONE] camera.json files are under:", CalibCfg.IMAGES_DIR.resolve())


if __name__ == "__main__":
    main()
