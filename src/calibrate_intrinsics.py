#!/usr/bin/env python3
import os
import glob
import cv2
import numpy as np
from calib_config import CalibCfg
from camera_io import build_charuco_detector
from json_io import to_json


def calibrate_single_camera(cam_name: str, image_dir: str, output_json: str):
    image_pattern = os.path.join(image_dir, f"*.{CalibCfg.IMAGE_EXT}")
    images = sorted(glob.glob(image_pattern))

    if not images:
        print(f"[{cam_name}] No images found in {image_dir}. Skipping.")
        return

    detector, board = build_charuco_detector()

    all_corners = []
    all_ids = []
    image_size = None
    valid_count = 0

    print(f"\n[{cam_name}] Processing {len(images)} images from '{image_dir}'...")

    for fname in images:
        img = cv2.imread(fname)
        if img is None:
            continue

        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        if image_size is None:
            image_size = (gray.shape[1], gray.shape[0])

        corners, ids, _, _ = detector.detectBoard(gray)

        if (
            corners is not None
            and ids is not None
            and len(corners) >= CalibCfg.MIN_CORNERS
            and len(corners) == len(ids)
        ):
            corners = np.ascontiguousarray(corners, dtype=np.float32).reshape(-1, 1, 2)
            ids = np.ascontiguousarray(ids, dtype=np.int32).reshape(-1, 1)
            all_corners.append(corners)
            all_ids.append(ids)
            valid_count += 1
            print(f"  [ACCEPTED] {os.path.basename(fname)}: {len(corners)} corners")
        else:
            num = 0 if corners is None else len(corners)
            print(f"  [REJECTED] {os.path.basename(fname)}: {num} corners (< {CalibCfg.MIN_CORNERS})")

    if valid_count < 8:
        print(f"[ERROR] [{cam_name}] Only {valid_count} valid frames found. Need >= 8 for reliable calibration.")
        return

    calib_flags = cv2.CALIB_RATIONAL_MODEL if valid_count >= 15 else 0

    rep_error, camera_matrix, dist_coeffs, _, _ = cv2.aruco.calibrateCameraCharuco(
        charucoCorners=all_corners,
        charucoIds=all_ids,
        board=board,
        imageSize=image_size,
        cameraMatrix=None,
        distCoeffs=None,
        flags=calib_flags
    )

    fx, fy = float(camera_matrix[0, 0]), float(camera_matrix[1, 1])
    cx, cy = float(camera_matrix[0, 2]), float(camera_matrix[1, 2])

    print("-" * 55)
    print(f"[{cam_name}] Calibration Results:")
    print(f"  Image Size:             {image_size[0]} x {image_size[1]}")
    print(f"  Valid Frames Used:      {valid_count} / {len(images)}")
    print(f"  RMS Reprojection Error: {rep_error:.4f} px")
    print(f"  Focal Length (fx, fy):  ({fx:.2f}, {fy:.2f}) px")
    print(f"  Principal Point (cx, cy): ({cx:.2f}, {cy:.2f}) px")
    print(f"  Distortion Model:       {'rational_polynomial' if calib_flags == cv2.CALIB_RATIONAL_MODEL else 'plumb_bob'}")
    print("-" * 55)

    # Matches the camera.json schema written by mvs_extrinsic_calib (camera_matrix,
    # distortion_coeff, camera). The pose fields (t_param, r_param, rotation_matrix,
    # projection_matrix) get added to this same file by calibrate_extrinsics.py.
    calib_dict = {
        "camera_matrix": camera_matrix.tolist(),
        "distortion_coeff": dist_coeffs.flatten().tolist(),
        "camera": cam_name,
    }

    os.makedirs(os.path.dirname(output_json), exist_ok=True)
    to_json(output_json, calib_dict)

    print(f"[{cam_name}] Saved: {output_json}")


def main():
    for _, cam_name in CalibCfg.CAMERAS.items():
        img_dir = os.path.join(CalibCfg.CALIB_DIR, "images", cam_name)
        out_json = os.path.join(img_dir, "camera.json")
        calibrate_single_camera(cam_name, img_dir, out_json)

    if CalibCfg.USE_BASLER:
        for _, cam_name in CalibCfg.BASLER_CAMERAS.items():
            img_dir = os.path.join(CalibCfg.CALIB_DIR, "images", cam_name)
            out_json = os.path.join(img_dir, "camera.json")
            calibrate_single_camera(cam_name, img_dir, out_json)

    print("\n[SUCCESS] All cameras calibrated. camera.json files are available under:",
          os.path.abspath(os.path.join(CalibCfg.CALIB_DIR, "images")))


if __name__ == "__main__":
    main()