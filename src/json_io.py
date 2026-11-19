#!/usr/bin/env python3
"""JSON read/write helpers for the camera.json schema used by mvs_extrinsic_calib."""
import json

import numpy as np

from calib_config import camera_json_path


class NumpyEncoder(json.JSONEncoder):
    """Serialise numpy arrays and scalars as plain JSON lists/numbers."""

    def default(self, obj):
        if isinstance(obj, np.ndarray):
            return obj.tolist()
        if isinstance(obj, np.generic):
            return obj.item()
        return super().default(obj)


def to_json(file_path, data) -> None:
    with open(file_path, "w") as f:
        json.dump(data, f, cls=NumpyEncoder, indent=4)


def from_json(file_path) -> dict:
    """Load a JSON file, converting every value to a numpy array.

    Keys containing "__" are left untouched. This mirrors `from_json` in
    mvs_extrinsic_calib so both pipelines can be read by the same downstream code.
    Note that the string-valued "camera" entry therefore comes back as a 0-d array.
    """
    with open(file_path) as f:
        data = json.load(f)
    return {k: v if "__" in k else np.array(v) for k, v in data.items()}


def load_camera(cam_name: str, missing_hint: str = "") -> dict:
    """Load `<CALIB_DIR>/images/<cam_name>/camera.json` with a helpful error if absent."""
    path = camera_json_path(cam_name)
    if not path.exists():
        raise FileNotFoundError(f"Missing camera.json for '{cam_name}': {path}. {missing_hint}".strip())
    return from_json(path)
