"""Render photorealistic-enough ChArUco views from known camera poses (no hardware needed)."""
import cv2
import numpy as np

from charuco import build_charuco_detector

IMAGE_SIZE = (1280, 720)
K_TRUE = np.array([[900.0, 0, 640.0], [0, 900.0, 360.0], [0, 0, 1]])

_PX_PER_M = 3000          # resolution of the flat board raster
_MARGIN_PX = 90           # white border around the board, in raster pixels


def _board_raster():
    _, board = build_charuco_detector()
    squares_x, squares_y = board.getChessboardSize()
    sq = board.getSquareLength()
    inner = (round(squares_x * sq * _PX_PER_M), round(squares_y * sq * _PX_PER_M))
    size = (inner[0] + 2 * _MARGIN_PX, inner[1] + 2 * _MARGIN_PX)
    return board.generateImage(size, marginSize=_MARGIN_PX, borderBits=1), board


def board_center_m():
    _, board = build_charuco_detector()
    squares_x, squares_y = board.getChessboardSize()
    return np.array([squares_x, squares_y, 0.0]) * board.getSquareLength() / 2


def render_view(rvec, tvec, K=K_TRUE, size=IMAGE_SIZE):
    """Grayscale-as-BGR image of the board as seen by a camera with world->camera pose (rvec, tvec)."""
    raster, _ = _board_raster()
    h, w = raster.shape
    corners_px = np.float32([[0, 0], [w, 0], [w, h], [0, h]])
    world = np.hstack([(corners_px - _MARGIN_PX) / _PX_PER_M, np.zeros((4, 1), np.float32)])
    dst, _ = cv2.projectPoints(world, rvec, tvec, K, None)
    H = cv2.getPerspectiveTransform(corners_px, dst.reshape(4, 2).astype(np.float32))
    img = cv2.warpPerspective(raster, H, size, flags=cv2.INTER_AREA, borderValue=110)
    return cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)


def random_pose(rng):
    """A pose that keeps the whole board in view, tilted up to ~30 degrees."""
    rot = cv2.Rodrigues(np.deg2rad(rng.uniform(-30, 30, 3)))[0]
    # Real boards are viewed with the camera in front of them (z<0 in board frame).
    base = cv2.Rodrigues(np.array([0.0, 0.0, 0.0]))[0]
    R = rot @ base
    distance = rng.uniform(0.5, 0.9)
    t = -R @ board_center_m() + np.array([rng.uniform(-0.03, 0.03), rng.uniform(-0.03, 0.03), distance])
    return cv2.Rodrigues(R)[0], t.reshape(3, 1)
