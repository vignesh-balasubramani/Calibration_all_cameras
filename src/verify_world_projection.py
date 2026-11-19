#!/usr/bin/env python3
"""Step 6: sanity-check a camera's extrinsics by deprojecting a pixel into world coordinates.

By default this takes the principal point at 1 m depth, which lies on the optical axis,
so the printed world point should be exactly `--depth` metres in front of the camera.
For a camera looking down at the table, expect its Z_w to be negative (world +Z points
into the table) and the deprojected point's Z_w to be closer to 0.

Example:
    python verify_world_projection.py --camera camera_0 --pixel 640 360 --depth 0.8
"""
import argparse

import numpy as np

from calib_config import camera_names
from geometry import camera_to_world_matrix, pixel_to_world
from json_io import load_camera


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    parser.add_argument("--camera", default=camera_names()[0],
                        help="camera name as it appears in calib_config.py (default: first configured)")
    parser.add_argument("--pixel", nargs=2, type=float, metavar=("U", "V"),
                        help="pixel to deproject (default: the principal point)")
    parser.add_argument("--depth", type=float, default=1.0,
                        help="depth along the optical axis, in metres (default: 1.0)")
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    camera = load_camera(args.camera, "Run calibrate_extrinsics.py first.")
    K = camera["camera_matrix"].astype(np.float64).reshape(3, 3)
    R = camera["rotation_matrix"].astype(np.float64).reshape(3, 3)
    t = camera["t_param"].astype(np.float64).reshape(3)

    u, v = args.pixel if args.pixel else (K[0, 2], K[1, 2])
    T_cam_to_world = camera_to_world_matrix(R, t)
    point = pixel_to_world(u, v, args.depth, K, T_cam_to_world)

    print("=" * 65)
    print(f"[{args.camera}] pixel ({u:.1f}, {v:.1f}) at depth {args.depth:.3f} m")
    print(f"  Camera position in world: {np.round(T_cam_to_world[:3, 3], 3).tolist()} m")
    print(f"  World point (X, Y, Z):    {np.round(point, 3).tolist()} m")
    print(f"  Height above tabletop:    {-point[2]:.3f} m  (= -Z; world +Z points into the table)")
    print("=" * 65)


if __name__ == "__main__":
    main()
