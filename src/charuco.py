#!/usr/bin/env python3
"""ChArUco board construction and detection. Pure OpenCV, no camera hardware needed."""
from typing import NamedTuple, Optional, Tuple

import cv2
import numpy as np

from calib_config import CalibCfg


class Detection(NamedTuple):
    """Result of one ChArUco detection pass. Both arrays are None if nothing was found."""
    corners: Optional[np.ndarray]
    ids: Optional[np.ndarray]

    @property
    def count(self) -> int:
        return 0 if self.corners is None else len(self.corners)

    @property
    def is_valid(self) -> bool:
        """True if enough corners were found to use this frame."""
        return self.ids is not None and self.count >= CalibCfg.MIN_CORNERS


def build_charuco_detector() -> Tuple[cv2.aruco.CharucoDetector, cv2.aruco.CharucoBoard]:
    """Build the board described by CalibCfg together with a detector for it."""
    dictionary = cv2.aruco.getPredefinedDictionary(CalibCfg.DICT_TYPE)
    board = cv2.aruco.CharucoBoard(
        (CalibCfg.SQUARES_X, CalibCfg.SQUARES_Y),
        CalibCfg.SQUARE_LENGTH_M,
        CalibCfg.MARKER_LENGTH_M,
        dictionary,
    )
    detector = cv2.aruco.CharucoDetector(
        board, cv2.aruco.CharucoParameters(), cv2.aruco.DetectorParameters()
    )
    return detector, board


def detect_board(detector: cv2.aruco.CharucoDetector, image: np.ndarray) -> Detection:
    """Detect ChArUco corners in a BGR or grayscale image."""
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY) if image.ndim == 3 else image
    corners, ids, _, _ = detector.detectBoard(gray)
    return Detection(corners, ids)
