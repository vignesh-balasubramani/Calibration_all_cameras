#!/usr/bin/env python3
"""Rig and ChArUco-board configuration shared by every calibration script.

Edit this file to match your hardware *before* running anything. Everything that
depends on the physical setup (camera serials, board dimensions, resolutions) lives
here so the scripts themselves never need to change.
"""
import os
from dataclasses import dataclass
from pathlib import Path

import cv2

REPO_ROOT = Path(__file__).resolve().parent.parent

# Common capture resolutions (width, height)
VGA = (640, 480)
HD = (1280, 720)
FHD = (1920, 1080)
UHD_4K = (4096, 2160)


class CalibCfg:
    # --- Output locations -------------------------------------------------------
    # Anchored to the repository root so scripts behave the same from any working
    # directory. Override with the CALIB_DIR environment variable.
    CALIB_DIR = Path(os.environ.get("CALIB_DIR", REPO_ROOT / "calibration_data"))
    IMAGES_DIR = CALIB_DIR / "images"                    # <camera>/calib_*.png + camera.json
    ORIGIN_DIR = CALIB_DIR / "world_origin_raw"          # <camera>_origin.png
    COMBINED_DIR = CALIB_DIR / "combined_calibration_data"
    IMAGE_EXT = "png"

    # --- ChArUco board ----------------------------------------------------------
    # Must match the *printed* target. Measure the real square/marker size with
    # calipers; printers rescale, and a 1 % error here is a 1 % error in every
    # translation the pipeline reports.
    SQUARES_X = 7
    SQUARES_Y = 5
    SQUARE_LENGTH_M = 0.038        # 38.0 mm
    MARKER_LENGTH_M = 0.0285       # 28.5 mm
    DICT_TYPE = cv2.aruco.DICT_APRILTAG_36h11

    # --- Acceptance thresholds --------------------------------------------------
    MIN_CORNERS = 6                # ChArUco corners needed for a frame to count
    MIN_FRAMES = 8                 # fewer accepted frames -> intrinsics are refused
    RATIONAL_MODEL_MIN_FRAMES = 15  # at/above this, fit 8 distortion coeffs instead of 5

    # --- Capture settings -------------------------------------------------------
    REALSENSE_RESOLUTION = HD      # HD (1280, 720) or VGA (640, 480)
    REALSENSE_FPS = 30

    BASLER_RESOLUTION = UHD_4K     # centered-ROI crop of the sensor; capped at sensor size

    # --- Cameras ----------------------------------------------------------------
    # Folder/camera name is what appears in every output file and in camera.json.

    # Intel RealSense D435: serial number -> camera name
    REALSENSE_CAMERAS = {
        "948122070553": "camera_6",
        "147122075374": "camera_4",
        "146322075747": "camera_5",
        "147122072939": "camera_2",
        "239722071052": "camera_0",
    }

    # Basler ace 2 (GigE): IP address or serial number -> camera name
    USE_BASLER = True
    BASLER_CAMERAS = {
        "192.168.1.1": "camera_basler",
    }


@dataclass(frozen=True)
class CameraSpec:
    """One physical camera: what it is, how to find it, and what to call it."""
    name: str
    kind: str          # "realsense" or "basler"
    device_id: str     # RealSense serial | Basler IP address or serial


def camera_specs() -> list:
    """All configured cameras, RealSense first, in the order they were declared."""
    specs = [CameraSpec(name, "realsense", serial)
             for serial, name in CalibCfg.REALSENSE_CAMERAS.items()]
    if CalibCfg.USE_BASLER:
        specs += [CameraSpec(name, "basler", target)
                  for target, name in CalibCfg.BASLER_CAMERAS.items()]
    return specs


def camera_names() -> list:
    return [spec.name for spec in camera_specs()]


def camera_dir(cam_name: str) -> Path:
    return CalibCfg.IMAGES_DIR / cam_name


def camera_json_path(cam_name: str) -> Path:
    return camera_dir(cam_name) / "camera.json"
