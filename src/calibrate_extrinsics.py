#!/usr/bin/env python3
import os
import json
import cv2
import numpy as np
from datetime import datetime
from calib_config import CalibCfg
from camera_io import build_charuco_detector
from json_io import to_json, from_json, NumpyEncoder

IMAGES_DIR = os.path.join(CalibCfg.CALIB_DIR, "images")
ORIGIN_DIR = os.path.join(CalibCfg.CALIB_DIR, "world_origin_raw")
COMBINED_DIR = os.path.join(CalibCfg.CALIB_DIR, "combined_calibration_data")


def load_intrinsics(cam_name):
    json_path = os.path.join(IMAGES_DIR, cam_name, "camera.json")
    if not os.path.exists(json_path):
        raise FileNotFoundError(f"Missing intrinsics file: {json_path}. Run calibrate_intrinsics.py first.")
    data = from_json(json_path)
    K = data["camera_matrix"].astype(np.float64).reshape((3, 3))
    D = data["distortion_coeff"].astype(np.float64).reshape((-1, 1))
    return K, D


def main():
    detector, board = build_charuco_detector()

    all_cams = list(CalibCfg.CAMERAS.values())
    if CalibCfg.USE_BASLER:
        all_cams.extend(list(CalibCfg.BASLER_CAMERAS.values()))

    camera_list = []

    print("=" * 75)
    print("SOLVING EXTRINSIC POSES RELATIVE TO WORKSTATION WORLD ORIGIN")
    print("=" * 75)

    for cam in all_cams:
        img_path = os.path.join(ORIGIN_DIR, f"{cam}_origin.png")
        if not os.path.exists(img_path):
            print(f"[WARN] No origin capture file found for '{cam}' at {img_path}")
            continue

        K, D = load_intrinsics(cam)
        img = cv2.imread(img_path)
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        corners, ids, _, _ = detector.detectBoard(gray)

        if corners is None or ids is None or len(corners) < CalibCfg.MIN_CORNERS:
            num = 0 if corners is None else len(corners)
            print(f"[ERROR] {cam} detected only {num} corners (< {CalibCfg.MIN_CORNERS})")
            continue

        obj_pts, img_pts = board.matchImagePoints(corners, ids)
        if obj_pts is None or len(obj_pts) < CalibCfg.MIN_CORNERS:
            print(f"[ERROR] Insufficient 3D-2D matched points for {cam}")
            continue

        success, rvec, tvec = cv2.solvePnP(obj_pts, img_pts, K, D, flags=cv2.SOLVEPNP_ITERATIVE)
        if not success:
            print(f"[ERROR] solvePnP failed for {cam}")
            continue

        rotation_matrix, _ = cv2.Rodrigues(rvec)
        projection_matrix = K @ np.hstack((rotation_matrix, tvec.reshape(3, 1)))

        # world -> camera position, for a sanity-check print only
        T_world_to_cam = np.eye(4, dtype=np.float64)
        T_world_to_cam[:3, :3] = rotation_matrix
        T_world_to_cam[:3, 3] = tvec.flatten()
        cam_pos_world = np.linalg.inv(T_world_to_cam)[:3, 3]

        print(f"[{cam}]:")
        print(f"  3D Position on Table (x, y, z) : [{cam_pos_world[0]:.3f}, {cam_pos_world[1]:.3f}, {cam_pos_world[2]:.3f}] meters")
        print(f"  Height above tabletop (Z_w)    : {cam_pos_world[2]:.3f} meters")

        # camera.json now matches mvs_extrinsic_calib's schema exactly: camera_matrix,
        # distortion_coeff, t_param, r_param, rotation_matrix, projection_matrix, camera
        camera_json_path = os.path.join(IMAGES_DIR, cam, "camera.json")
        camera_data = {
            "camera_matrix": K.tolist(),
            "distortion_coeff": D.flatten().tolist(),
            "t_param": tvec.reshape(3, 1).tolist(),
            "r_param": rvec.reshape(3, 1).tolist(),
            "rotation_matrix": rotation_matrix.tolist(),
            "projection_matrix": projection_matrix.tolist(),
            "camera": cam,
        }
        to_json(camera_json_path, camera_data)
        camera_list.append(camera_data)

    os.makedirs(COMBINED_DIR, exist_ok=True)
    combined = {
        "header": {
            "created": datetime.now().strftime("%H_%M_%S"),
            "world_origin_description": "Workstation table corner (ChArUco top-left corner)",
            "board": {
                "squares_x": CalibCfg.SQUARES_X,
                "squares_y": CalibCfg.SQUARES_Y,
                "square_length_m": CalibCfg.SQUARE_LENGTH_M,
                "marker_length_m": CalibCfg.MARKER_LENGTH_M,
            },
        },
        "cameras": camera_list,
    }
    combined_path = os.path.join(COMBINED_DIR, f"camera_parameter_{datetime.now().strftime('%Y%m%d_%H%M')}.json")
    with open(combined_path, "w") as f:
        json.dump(combined, f, cls=NumpyEncoder, indent=4)

    print("-" * 75)
    print(f"[SUCCESS] Per-camera camera.json files updated under: {os.path.abspath(IMAGES_DIR)}")
    print(f"[SUCCESS] Combined calibration written to: {combined_path}")


if __name__ == "__main__":
    main()