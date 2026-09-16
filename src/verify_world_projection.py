#!/usr/bin/env python3
import os
import numpy as np
from calib_config import CalibCfg
from json_io import from_json

IMAGES_DIR = os.path.join(CalibCfg.CALIB_DIR, "images")


def load_camera(cam_name):
    json_path = os.path.join(IMAGES_DIR, cam_name, "camera.json")
    if not os.path.exists(json_path):
        raise FileNotFoundError(f"Missing camera.json for '{cam_name}': {json_path}. Run calibrate_extrinsics.py first.")
    return from_json(json_path)


def cam_to_world_transform(camera_data):
    """Builds T_cam_to_world from the stored rotation_matrix/t_param (world -> camera)."""
    R = camera_data["rotation_matrix"].astype(np.float64).reshape(3, 3)
    t = camera_data["t_param"].astype(np.float64).reshape(3)

    T_world_to_cam = np.eye(4, dtype=np.float64)
    T_world_to_cam[:3, :3] = R
    T_world_to_cam[:3, 3] = t

    return np.linalg.inv(T_world_to_cam)


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
    target_cam = "camera_0"
    print("=" * 65)
    print("MULTI-CAMERA WORLD COORDINATE CONVERTER")
    print("=" * 65)

    camera_data = load_camera(target_cam)
    K = camera_data["camera_matrix"].astype(np.float64).reshape(3, 3)
    T_cam_to_world = cam_to_world_transform(camera_data)

    u_center, v_center = int(K[0, 2]), int(K[1, 2])
    test_depth_m = 1.000  # 1 meter forward along optical axis

    P_w = pixel_to_world(u_center, v_center, test_depth_m, K, T_cam_to_world)
    print(f"Sample Projection for [{target_cam}]:")
    print(f"  Pixel: ({u_center}, {v_center}) with Depth: {test_depth_m:.3f} m")
    print(f"  -> World Coordinate (Table X, Table Y, Height Z): [{P_w[0]:.3f}, {P_w[1]:.3f}, {P_w[2]:.3f}] meters\n")


if __name__ == "__main__":
    main()