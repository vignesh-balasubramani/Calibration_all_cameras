#!/usr/bin/env python3
import os
import cv2
import numpy as np
import yaml
from calib_config import CalibCfg

INTRINSICS_DIR = os.path.join(CalibCfg.CALIB_DIR, "yaml")
ORIGIN_DIR = os.path.join(CalibCfg.CALIB_DIR, "world_origin_raw")
OUTPUT_YAML = os.path.join(CalibCfg.CALIB_DIR, "yaml", "workstation_extrinsics.yaml")


def load_intrinsics(cam_name):
    yaml_path = os.path.join(INTRINSICS_DIR, f"{cam_name}_intrinsics.yaml")
    if not os.path.exists(yaml_path):
        raise FileNotFoundError(f"Missing intrinsics file: {yaml_path}. Run calibrate_intrinsics.py first.")
    with open(yaml_path, "r") as f:
        data = yaml.safe_load(f)
    K = np.array(data["camera_matrix"]["data"], dtype=np.float64).reshape((3, 3))
    D = np.array(data["distortion_coefficients"]["data"], dtype=np.float64).reshape((-1, 1))
    return K, D


def main():
    dictionary = cv2.aruco.getPredefinedDictionary(CalibCfg.DICT_TYPE)
    board = cv2.aruco.CharucoBoard(
        (CalibCfg.SQUARES_X, CalibCfg.SQUARES_Y),
        CalibCfg.SQUARE_LENGTH_M,
        CalibCfg.MARKER_LENGTH_M,
        dictionary
    )
    detector = cv2.aruco.CharucoDetector(board, cv2.aruco.CharucoParameters(), cv2.aruco.DetectorParameters())

    all_cams = list(CalibCfg.CAMERAS.values())
    if CalibCfg.USE_BASLER:
        all_cams.extend(list(CalibCfg.BASLER_CAMERAS.values()))

    extrinsics_output = {
        "world_origin_description": "Workstation table corner (ChArUco Top-Left corner)",
        "coordinate_convention": "X_w: along long edge (m), Y_w: along short edge (m), Z_w: height above table (m)",
        "units": "meters",
        "cameras": {}
    }

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

        if corners is not None and ids is not None and len(corners) >= CalibCfg.MIN_CORNERS:
            obj_pts, img_pts = board.matchImagePoints(corners, ids)
            if obj_pts is not None and len(obj_pts) >= CalibCfg.MIN_CORNERS:
                success, rvec, tvec = cv2.solvePnP(
                    obj_pts, img_pts, K, D, flags=cv2.SOLVEPNP_ITERATIVE
                )
                if success:
                    R_mat, _ = cv2.Rodrigues(rvec)

                    # T_W_to_Cam: transforms World point -> Camera coordinate
                    T_world_to_cam = np.eye(4, dtype=np.float64)
                    T_world_to_cam[:3, :3] = R_mat
                    T_world_to_cam[:3, 3] = tvec.flatten()

                    # T_Cam_to_World: transforms Camera coordinate -> Table World point
                    T_cam_to_world = np.linalg.inv(T_world_to_cam)
                    cam_pos_world = T_cam_to_world[:3, 3]

                    print(f"[{cam}]:")
                    print(f"  3D Position on Table (x, y, z) : [{cam_pos_world[0]:.3f}, {cam_pos_world[1]:.3f}, {cam_pos_world[2]:.3f}] meters")
                    print(f"  Height above tabletop (Z_w)    : {cam_pos_world[2]:.3f} meters")

                    extrinsics_output["cameras"][cam] = {
                        "T_world_to_camera": {
                            "rows": 4,
                            "cols": 4,
                            "data": [float(x) for x in T_world_to_cam.flatten()]
                        },
                        "T_camera_to_world": {
                            "rows": 4,
                            "cols": 4,
                            "data": [float(x) for x in T_cam_to_world.flatten()]
                        },
                        "camera_position_in_world": [float(x) for x in cam_pos_world],
                        "rotation_vector": [float(x) for x in rvec.flatten()],
                        "translation_vector": [float(x) for x in tvec.flatten()],
                        "matched_corners_used": int(len(obj_pts))
                    }
                else:
                    print(f"[ERROR] solvePnP failed for {cam}")
            else:
                print(f"[ERROR] Insufficient 3D-2D matched points for {cam}")
        else:
            num = 0 if corners is None else len(corners)
            print(f"[ERROR] {cam} detected only {num} corners (< {CalibCfg.MIN_CORNERS})")

    os.makedirs(os.path.dirname(OUTPUT_YAML), exist_ok=True)
    with open(OUTPUT_YAML, "w") as f:
        yaml.dump(extrinsics_output, f, default_flow_style=False)

    print("-" * 75)
    print(f"[SUCCESS] Workstation origin extrinsics saved to: {OUTPUT_YAML}")


if __name__ == "__main__":
    main()