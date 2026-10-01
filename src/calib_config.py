#!/usr/bin/env python3
import cv2

VGA = (640, 480)
HD = (1280, 720)
FHD = (1920, 1080)
UHD_4K = (4096, 2160)


class CalibCfg:
    CALIB_DIR = "./calibration_data"
    IMAGE_EXT = "png"

    # physical board dimensions, update to match the printed target
    SQUARES_X = 7
    SQUARES_Y = 5
    SQUARE_LENGTH_M = 0.038        # 38.0 mm
    MARKER_LENGTH_M = 0.0285       # 28.5 mm
    DICT_TYPE = cv2.aruco.DICT_APRILTAG_36h11

    # minimum detected corners for a frame to be accepted
    MIN_CORNERS = 6

    realsense_resolution = HD      # (1280, 720) or VGA (640, 480)
    realsense_fps = 30

    basler_resolution = UHD_4K     # (4096, 2160) or HD (1280, 720)
    basler_fps = 30

    # 5 Intel RealSense D435 cameras: serial number -> folder name
    CAMERAS = {
        "948122070553": "camera_6",
        "147122075374": "camera_4",
        "146322075747": "camera_5",
        "147122072939": "camera_2",
        "239722071052": "camera_0",
    }

    # Basler ace 2 GigE camera: IP or serial -> folder name
    USE_BASLER = True
    BASLER_CAMERAS = {
        "192.168.1.1": "camera_basler"
    } if USE_BASLER else {}