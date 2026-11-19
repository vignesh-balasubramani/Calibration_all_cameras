#!/usr/bin/env python3
"""Step 5: solve every camera's pose relative to the table origin.

For each camera, detects the ChArUco board in its world-origin frame and runs
`solvePnP`. The pose is added to the camera's `camera.json`, and all cameras are also
written to one combined `camera_parameter_<timestamp>.json`.
"""
from datetime import datetime
from typing import Tuple

import cv2
import numpy as np

from calib_config import CalibCfg, camera_json_path, camera_names
from charuco import build_charuco_detector, detect_board
from geometry import camera_position_in_world
from json_io import load_camera, to_json


class PoseEstimationError(RuntimeError):
    """The board pose could not be recovered from an image."""


def estimate_board_pose(image: np.ndarray, K: np.ndarray, D: np.ndarray,
                        detector, board) -> Tuple[np.ndarray, np.ndarray, float]:
    """World -> camera pose of the board in `image`.

    Returns (rvec, tvec, rms_px) where rms_px is the RMS reprojection error of the
    board corners under the recovered pose.
    """
    detection = detect_board(detector, image)
    if not detection.is_valid:
        raise PoseEstimationError(
            f"only {detection.count} corners detected (need >= {CalibCfg.MIN_CORNERS})")

    obj_pts, img_pts = board.matchImagePoints(detection.corners, detection.ids)
    if obj_pts is None or len(obj_pts) < CalibCfg.MIN_CORNERS:
        raise PoseEstimationError("too few matched 3D-2D point pairs")

    ok, rvec, tvec = cv2.solvePnP(obj_pts, img_pts, K, D, flags=cv2.SOLVEPNP_ITERATIVE)
    if not ok:
        raise PoseEstimationError("solvePnP failed")

    projected, _ = cv2.projectPoints(obj_pts, rvec, tvec, K, D)
    rms = float(np.sqrt(np.mean(np.sum((projected - img_pts) ** 2, axis=-1))))
    return rvec, tvec, rms


def main() -> None:
    detector, board = build_charuco_detector()
    camera_list = []

    print("=" * 75)
    print("SOLVING EXTRINSIC POSES RELATIVE TO THE WORKSTATION WORLD ORIGIN")
    print("=" * 75)

    for cam in camera_names():
        origin_path = CalibCfg.ORIGIN_DIR / f"{cam}_origin.png"
        if not origin_path.exists():
            print(f"[WARN] {cam}: no origin capture at {origin_path}")
            continue

        intrinsics = load_camera(cam, "Run calibrate_intrinsics.py first.")
        K = intrinsics["camera_matrix"].astype(np.float64).reshape(3, 3)
        D = intrinsics["distortion_coeff"].astype(np.float64).reshape(-1, 1)

        try:
            rvec, tvec, rms = estimate_board_pose(cv2.imread(str(origin_path)), K, D, detector, board)
        except PoseEstimationError as exc:
            print(f"[ERROR] {cam}: {exc}")
            continue

        R, _ = cv2.Rodrigues(rvec)
        position = camera_position_in_world(R, tvec)
        print(f"[{cam}]")
        print(f"  Camera position in world (X, Y, Z) : "
              f"[{position[0]:+.3f}, {position[1]:+.3f}, {position[2]:+.3f}] m")
        print(f"  Height above tabletop (-Z)         : {-position[2]:.3f} m")
        print(f"  PnP reprojection RMS               : {rms:.3f} px")

        camera_data = {
            "camera_matrix": K.tolist(),
            "distortion_coeff": D.flatten().tolist(),
            "t_param": tvec.reshape(3, 1).tolist(),
            "r_param": rvec.reshape(3, 1).tolist(),
            "rotation_matrix": R.tolist(),
            "projection_matrix": (K @ np.hstack((R, tvec.reshape(3, 1)))).tolist(),
            "camera": cam,
        }
        to_json(camera_json_path(cam), camera_data)
        camera_list.append(camera_data)

    if not camera_list:
        print("[ERROR] No camera poses were solved; nothing written.")
        return

    now = datetime.now()
    combined = {
        "header": {
            "created": now.isoformat(timespec="seconds"),
            "world_origin_description": "Top-left corner of the ChArUco board on the workstation table",
            "world_frame": "X along board top edge, Y down board left edge, Z into the table",
            "board": {
                "squares_x": CalibCfg.SQUARES_X,
                "squares_y": CalibCfg.SQUARES_Y,
                "square_length_m": CalibCfg.SQUARE_LENGTH_M,
                "marker_length_m": CalibCfg.MARKER_LENGTH_M,
            },
        },
        "cameras": camera_list,
    }
    CalibCfg.COMBINED_DIR.mkdir(parents=True, exist_ok=True)
    combined_path = CalibCfg.COMBINED_DIR / f"camera_parameter_{now:%Y%m%d_%H%M}.json"
    to_json(combined_path, combined)

    print("-" * 75)
    print(f"[SUCCESS] {len(camera_list)} camera.json files updated under {CalibCfg.IMAGES_DIR.resolve()}")
    print(f"[SUCCESS] Combined calibration: {combined_path}")


if __name__ == "__main__":
    main()
