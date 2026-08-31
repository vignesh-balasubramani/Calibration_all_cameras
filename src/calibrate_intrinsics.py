#!/usr/bin/env python3
import os
import glob
import cv2
import numpy as np
import yaml
from calib_config import CalibCfg


def calibrate_single_camera(cam_name: str, image_dir: str, output_yaml: str):
    image_pattern = os.path.join(image_dir, f"*.{CalibCfg.IMAGE_EXT}")
    images = sorted(glob.glob(image_pattern))

    if not images:
        print(f"[{cam_name}] No images found in {image_dir}. Skipping.")
        return

    dictionary = cv2.aruco.getPredefinedDictionary(CalibCfg.DICT_TYPE)
    board = cv2.aruco.CharucoBoard(
        (CalibCfg.SQUARES_X, CalibCfg.SQUARES_Y),
        CalibCfg.SQUARE_LENGTH_M,
        CalibCfg.MARKER_LENGTH_M,
        dictionary
    )
    detector = cv2.aruco.CharucoDetector(board, cv2.aruco.CharucoParameters(), cv2.aruco.DetectorParameters())

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

    # Export to YAML in standard format
    calib_dict = {
        "camera_name": cam_name,
        "image_width": int(image_size[0]),
        "image_height": int(image_size[1]),
        "reprojection_error_rms": float(rep_error),
        "camera_matrix": {
            "rows": 3,
            "cols": 3,
            "data": [float(x) for x in camera_matrix.flatten()]
        },
        "distortion_model": "rational_polynomial" if calib_flags == cv2.CALIB_RATIONAL_MODEL else "plumb_bob",
        "distortion_coefficients": {
            "rows": 1,
            "cols": int(dist_coeffs.size),
            "data": [float(x) for x in dist_coeffs.flatten()]
        }
    }

    os.makedirs(os.path.dirname(output_yaml), exist_ok=True)
    with open(output_yaml, "w") as f:
        yaml.dump(calib_dict, f, default_flow_style=False)

    print(f"[{cam_name}] Saved YAML: {output_yaml}")


def main():
    yaml_dir = os.path.join(CalibCfg.CALIB_DIR, "yaml")
    os.makedirs(yaml_dir, exist_ok=True)

    # 1. Calibrate RealSense cameras
    for _, cam_name in CalibCfg.CAMERAS.items():
        img_dir = os.path.join(CalibCfg.CALIB_DIR, "images", cam_name)
        out_yaml = os.path.join(yaml_dir, f"{cam_name}_intrinsics.yaml")
        calibrate_single_camera(cam_name, img_dir, out_yaml)

    # 2. Calibrate Basler camera
    if CalibCfg.USE_BASLER:
        for _, cam_name in CalibCfg.BASLER_CAMERAS.items():
            img_dir = os.path.join(CalibCfg.CALIB_DIR, "images", cam_name)
            out_yaml = os.path.join(yaml_dir, f"{cam_name}_intrinsics.yaml")
            calibrate_single_camera(cam_name, img_dir, out_yaml)

    print("\n[SUCCESS] All cameras calibrated. YAML files are available in:", os.path.abspath(yaml_dir))


if __name__ == "__main__":
    main()