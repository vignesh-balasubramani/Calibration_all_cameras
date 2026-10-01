#!/usr/bin/env python3
"""JSON read/write helpers matching the camera.json schema used by mvs_extrinsic_calib."""
import json
import numpy as np


class NumpyEncoder(json.JSONEncoder):
    def default(self, obj):
        if isinstance(obj, np.ndarray):
            return obj.tolist()
        return json.JSONEncoder.default(self, obj)


def to_json(file_path, data):
    with open(file_path, "w") as f:
        json.dump(data, f, cls=NumpyEncoder, indent=4)


def from_json(file_path):
    with open(file_path) as f:
        data = json.load(f)
    for k, v in data.items():
        if "__" not in k:
            data[k] = np.array(v)
    return data
