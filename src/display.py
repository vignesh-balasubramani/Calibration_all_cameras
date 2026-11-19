#!/usr/bin/env python3
"""OpenCV preview-window helpers shared by the interactive capture scripts."""
from typing import Sequence, Tuple

import cv2
import numpy as np

from calib_config import CalibCfg
from charuco import Detection

GREEN = (0, 255, 0)
RED = (0, 0, 255)

KEYS_SAVE = frozenset({ord("c"), ord(" ")})
KEYS_NEXT = frozenset({ord("n")})
KEYS_QUIT = frozenset({ord("q"), 27})  # 27 = Esc


def read_key() -> int:
    """Poll the OpenCV window for one key press (non-blocking, ~1 ms)."""
    return cv2.waitKey(1) & 0xFF


def draw_corners(frame: np.ndarray, detection: Detection) -> np.ndarray:
    """Return a copy of `frame` with the detected ChArUco corners drawn on it."""
    canvas = frame.copy()
    if detection.corners is not None and detection.ids is not None:
        cv2.aruco.drawDetectedCornersCharuco(canvas, detection.corners, detection.ids, GREEN)
    return canvas


def resize_to_width(image: np.ndarray, width: int) -> np.ndarray:
    """Downscale `image` to `width` pixels, preserving its aspect ratio."""
    height = round(image.shape[0] * width / image.shape[1])
    return cv2.resize(image, (width, height))


def status_text(name: str, detection: Detection) -> str:
    verdict = "OK" if detection.is_valid else f"NEED >= {CalibCfg.MIN_CORNERS}"
    return f"{name}: {detection.count} corners ({verdict})"


def draw_status(image: np.ndarray, text: str, ok: bool) -> None:
    """Draw a one-line status banner (green when `ok`, red otherwise), in place."""
    scale, thickness, pad = 0.7, 2, 10
    (w, h), baseline = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, scale, thickness)
    cv2.rectangle(image, (pad, pad), (pad + w + 2 * pad, pad + h + baseline + 2 * pad),
                  (20, 20, 20), -1)
    cv2.putText(image, text, (2 * pad, 2 * pad + h), cv2.FONT_HERSHEY_SIMPLEX, scale,
                GREEN if ok else RED, thickness)


def make_grid(tiles: Sequence[np.ndarray], tile_size: Tuple[int, int] = (380, 215),
              cols: int = 3) -> np.ndarray:
    """Tile images into a `cols`-wide grid; unused cells are left black."""
    width, height = tile_size
    resized = [cv2.resize(t, tile_size) for t in tiles]
    rows = []
    for start in range(0, len(resized), cols):
        row = resized[start:start + cols]
        row += [np.zeros((height, width, 3), np.uint8)] * (cols - len(row))
        rows.append(np.hstack(row))
    return np.vstack(rows)
