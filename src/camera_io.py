#!/usr/bin/env python3
"""Shared ChArUco detector and camera setup helpers for RealSense + Basler rigs."""
import cv2
import pyrealsense2 as rs
from pypylon import pylon
from calib_config import CalibCfg


def build_charuco_detector():
    dictionary = cv2.aruco.getPredefinedDictionary(CalibCfg.DICT_TYPE)
    board = cv2.aruco.CharucoBoard(
        (CalibCfg.SQUARES_X, CalibCfg.SQUARES_Y),
        CalibCfg.SQUARE_LENGTH_M,
        CalibCfg.MARKER_LENGTH_M,
        dictionary
    )
    detector = cv2.aruco.CharucoDetector(board, cv2.aruco.CharucoParameters(), cv2.aruco.DetectorParameters())
    return detector, board


def open_realsense_pipeline(serial: str):
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_device(serial)
    config.enable_stream(
        rs.stream.color,
        CalibCfg.realsense_resolution[0],
        CalibCfg.realsense_resolution[1],
        rs.format.bgr8,
        CalibCfg.realsense_fps
    )
    pipeline.start(config)
    return pipeline


def configure_basler_centered_roi(camera):
    """Applies centered ROI cropping based on CalibCfg.basler_resolution."""
    if hasattr(camera, "OffsetX"):
        camera.OffsetX.SetValue(0)
    if hasattr(camera, "OffsetY"):
        camera.OffsetY.SetValue(0)

    max_w = camera.Width.GetMax()
    max_h = camera.Height.GetMax()

    target_w = min(int(CalibCfg.basler_resolution[0]), max_w)
    target_h = min(int(CalibCfg.basler_resolution[1]), max_h)

    inc_w = camera.Width.GetInc()
    inc_h = camera.Height.GetInc()
    target_w = (target_w // inc_w) * inc_w
    target_h = (target_h // inc_h) * inc_h

    camera.Width.SetValue(target_w)
    camera.Height.SetValue(target_h)

    off_x = (max_w - target_w) // 2
    off_y = (max_h - target_h) // 2

    if hasattr(camera, "OffsetX"):
        inc_x = camera.OffsetX.GetInc()
        camera.OffsetX.SetValue((off_x // inc_x) * inc_x)

    if hasattr(camera, "OffsetY"):
        inc_y = camera.OffsetY.GetInc()
        camera.OffsetY.SetValue((off_y // inc_y) * inc_y)


def open_basler_camera(target_id: str):
    """Opens, configures (centered ROI) and starts grabbing on a Basler camera.

    Returns (camera, converter) with the converter set up to output BGR8 frames.
    """
    tl_factory = pylon.TlFactory.GetInstance()
    info = pylon.DeviceInfo()
    if "." in target_id:
        info.SetPropertyValue("IpAddress", target_id)
    else:
        info.SetSerialNumber(target_id)

    camera = pylon.InstantCamera(tl_factory.CreateDevice(info))
    camera.Open()
    if camera.IsGrabbing():
        camera.StopGrabbing()

    configure_basler_centered_roi(camera)
    camera.StartGrabbing(pylon.GrabStrategy_LatestImageOnly)

    converter = pylon.ImageFormatConverter()
    converter.OutputPixelFormat = pylon.PixelType_BGR8packed

    return camera, converter
