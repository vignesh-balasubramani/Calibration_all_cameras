#!/usr/bin/env python3
import os
import cv2
import numpy as np
import yaml
from calibration_test.src.calib_config import CalibCfg

EXTRINSICS_YAML = os.path.join(CalibCfg.CALIB_DIR, "yaml", "workstation_extrinsics.yaml")
INTRINSICS_DIR = os.path.join(CalibCfg.CALIB_DIR, "yaml")


def load_intrinsics(cam_name):
    yaml_path = os.path.join(INTRINSICS_DIR, f"{cam_name}_intrinsics.yaml")
    with open(yaml_path, "r") as f:
        data = yaml.safe_load(f)
    K = np.array(data["camera_matrix"]["data"], dtype=np.float64).reshape((3, 3))
    return K


def load_extrinsics():
    with open(EXTRINSICS_YAML, "r") as f:
        data = yaml.safe_load(f)
    return data["cameras"]


def pixel_to_world(u: int, v: int, depth_m: float, K: np.ndarray, T_cam_to_world: np.ndarray):
    """
    Transforms a 2D pixel (u, v) and depth value Z_c into 3D Workstation World coordinates.
    """
    fx, fy = K[0, 0], K[1, 1]
    cx, cy = K[0, 2], K[1, 2]

    # Deproject to 3D Camera Frame
    X_c = (u - cx) * depth_m / fx
    Y_c = (v - cy) * depth_m / fy
    Z_c = depth_m

    P_camera = np.array([X_c, Y_c, Z_c, 1.0], dtype=np.float64)

    # Transform to Workstation World Frame
    P_world = T_cam_to_world @ P_camera
    return P_world[:3]  # (X_w, Y_w, Z_w)


def main():
    if not os.path.exists(EXTRINSICS_YAML):
        print(f"[ERROR] Extrinsics YAML not found: {EXTRINSICS_YAML}")
        return

    extrinsics = load_extrinsics()
    print("=" * 65)
    print("MULTI-CAMERA WORLD COORDINATE CONVERTER")
    print("=" * 65)

    # Example: Deproject optical center pixel of camera_0 at 1.0 meter depth
    target_cam = "camera_0"
    if target_cam in extrinsics:
        K = load_intrinsics(target_cam)
        T_cam_to_world = np.array(extrinsics[target_cam]["T_camera_to_world"]["data"]).reshape((4, 4))

        u_center, v_center = int(K[0, 2]), int(K[1, 2])
        test_depth_m = 1.000  # 1 meter forward along optical axis

        P_w = pixel_to_world(u_center, v_center, test_depth_m, K, T_cam_to_world)
        print(f"Sample Projection for [{target_cam}]:")
        print(f"  Pixel: ({u_center}, {v_center}) with Depth: {test_depth_m:.3f} m")
        print(f"  -> World Coordinate (Table X, Table Y, Height Z): [{P_w[0]:.3f}, {P_w[1]:.3f}, {P_w[2]:.3f}] meters\n")


if __name__ == "__main__":
    main()