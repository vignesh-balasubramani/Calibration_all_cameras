#!/usr/bin/env python3
"""Rigid-transform helpers for the world <-> camera conventions used in camera.json.

camera.json stores the pose exactly as OpenCV's solvePnP returns it, i.e. the
*world -> camera* transform:  X_cam = R @ X_world + t.

The world frame is the ChArUco board frame: origin at the board's top-left corner,
+X along the board's top edge, +Y down the board's left edge, and +Z = X x Y pointing
*into* the board (into the table). A camera above the table therefore has a
**negative** Z_w; its height above the table is -Z_w.
"""
import numpy as np


def world_to_camera_matrix(R: np.ndarray, t: np.ndarray) -> np.ndarray:
    """4x4 homogeneous world -> camera transform."""
    T = np.eye(4, dtype=np.float64)
    T[:3, :3] = np.asarray(R, dtype=np.float64).reshape(3, 3)
    T[:3, 3] = np.asarray(t, dtype=np.float64).reshape(3)
    return T


def camera_to_world_matrix(R: np.ndarray, t: np.ndarray) -> np.ndarray:
    """4x4 homogeneous camera -> world transform (inverse of the stored pose)."""
    R = np.asarray(R, dtype=np.float64).reshape(3, 3)
    t = np.asarray(t, dtype=np.float64).reshape(3)
    T = np.eye(4, dtype=np.float64)
    T[:3, :3] = R.T
    T[:3, 3] = -R.T @ t
    return T


def camera_position_in_world(R: np.ndarray, t: np.ndarray) -> np.ndarray:
    """Camera optical centre expressed in world coordinates (metres)."""
    return camera_to_world_matrix(R, t)[:3, 3]


def deproject_pixel(u: float, v: float, depth_m: float, K: np.ndarray) -> np.ndarray:
    """Pinhole back-projection of pixel (u, v) at `depth_m` along the optical axis.

    Assumes (u, v) is already undistorted; lens distortion is not applied here.
    """
    fx, fy, cx, cy = K[0, 0], K[1, 1], K[0, 2], K[1, 2]
    return np.array([(u - cx) * depth_m / fx, (v - cy) * depth_m / fy, depth_m])


def pixel_to_world(u: float, v: float, depth_m: float, K: np.ndarray,
                   T_cam_to_world: np.ndarray) -> np.ndarray:
    """Pixel + depth -> 3D point in the world frame (X_w, Y_w, Z_w) in metres."""
    point_cam = np.append(deproject_pixel(u, v, depth_m, K), 1.0)
    return (T_cam_to_world @ point_cam)[:3]
