#!/usr/bin/env python3
import os
import cv2
import numpy as np
import pyrealsense2 as rs
from pypylon import pylon
from calib_config import CalibCfg

SAVE_DIR = os.path.join(CalibCfg.CALIB_DIR, "world_origin_raw")
os.makedirs(SAVE_DIR, exist_ok=True)


def get_detector():
    dictionary = cv2.aruco.getPredefinedDictionary(CalibCfg.DICT_TYPE)
    board = cv2.aruco.CharucoBoard(
        (CalibCfg.SQUARES_X, CalibCfg.SQUARES_Y),
        CalibCfg.SQUARE_LENGTH_M,
        CalibCfg.MARKER_LENGTH_M,
        dictionary
    )
    detector = cv2.aruco.CharucoDetector(board, cv2.aruco.CharucoParameters(), cv2.aruco.DetectorParameters())
    return detector, board


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


def main():
    detector, board = get_detector()

    # 1. Initialize RealSense Camera Pipelines
    rs_pipelines = {}
    for serial, name in CalibCfg.CAMERAS.items():
        p = rs.pipeline()
        c = rs.config()
        c.enable_device(serial)
        c.enable_stream(
            rs.stream.color,
            CalibCfg.realsense_resolution[0],
            CalibCfg.realsense_resolution[1],
            rs.format.bgr8,
            CalibCfg.realsense_fps
        )
        try:
            p.start(c)
            rs_pipelines[name] = p
        except Exception as e:
            print(f"[WARN] Failed to connect RealSense {name} ({serial}): {e}")

    # 2. Initialize Basler Camera with Centered ROI
    basler_cam = None
    converter = None
    if CalibCfg.USE_BASLER:
        tl_factory = pylon.TlFactory.GetInstance()
        for target_id, name in CalibCfg.BASLER_CAMERAS.items():
            try:
                info = pylon.DeviceInfo()
                if "." in target_id:
                    info.SetPropertyValue("IpAddress", target_id)
                else:
                    info.SetSerialNumber(target_id)
                basler_cam = pylon.InstantCamera(tl_factory.CreateDevice(info))
                basler_cam.Open()
                configure_basler_centered_roi(basler_cam)
                basler_cam.StartGrabbing(pylon.GrabStrategy_LatestImageOnly)

                converter = pylon.ImageFormatConverter()
                converter.OutputPixelFormat = pylon.PixelType_BGR8packed
            except Exception as e:
                print(f"[WARN] Failed to connect Basler {name} ({target_id}): {e}")

    print("=" * 65)
    print("WORKSTATION WORLD ORIGIN FRAME CAPTURE")
    print("Align top-left corner of ChArUco board with the chosen table corner.")
    print("Controls: [Space] or [C] Capture Origin Frames | [Q] Exit")
    print("=" * 65)

    try:
        while True:
            current_frames = {}
            display_frames = {}

            # Grab RealSense feeds
            for name, p in rs_pipelines.items():
                frames = p.wait_for_frames(timeout_ms=5000)
                color = frames.get_color_frame()
                if color:
                    frame = np.asanyarray(color.get_data())
                    current_frames[name] = frame
                    disp = frame.copy()
                    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
                    corners, ids, _, _ = detector.detectBoard(gray)
                    if corners is not None and ids is not None and len(corners) >= CalibCfg.MIN_CORNERS:
                        cv2.aruco.drawDetectedCornersCharuco(disp, corners, ids, (0, 255, 0))
                        cv2.putText(disp, f"{name}: {len(corners)} corners (OK)", (15, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                    else:
                        num = 0 if corners is None else len(corners)
                        cv2.putText(disp, f"{name}: {num} corners (NEED >= 6)", (15, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
                    display_frames[name] = disp

            # Grab Basler feed
            if basler_cam and basler_cam.IsGrabbing():
                res = basler_cam.RetrieveResult(5000, pylon.TimeoutHandling_ThrowException)
                if res.GrabSucceeded():
                    frame = converter.Convert(res).GetArray()
                    current_frames["camera_basler"] = frame
                    disp = frame.copy()
                    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
                    corners, ids, _, _ = detector.detectBoard(gray)
                    if corners is not None and ids is not None and len(corners) >= CalibCfg.MIN_CORNERS:
                        cv2.aruco.drawDetectedCornersCharuco(disp, corners, ids, (0, 255, 0))
                        cv2.putText(disp, f"camera_basler: {len(corners)} corners (OK)", (15, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                    else:
                        num = 0 if corners is None else len(corners)
                        cv2.putText(disp, f"camera_basler: {num} corners (NEED >= 6)", (15, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
                    display_frames["camera_basler"] = disp
                res.Release()

            # Render 2x3 multi-camera grid preview
            previews = [cv2.resize(img, (380, 215)) for img in display_frames.values()]
            if len(previews) == 6:
                row1 = np.hstack(previews[0:3])
                row2 = np.hstack(previews[3:6])
                grid = np.vstack((row1, row2))
            elif len(previews) > 0:
                grid = np.hstack(previews)
            else:
                continue

            cv2.imshow("Workstation Origin Alignment Preview", grid)

            key = cv2.waitKey(1) & 0xFF
            if key in [ord('c'), 32]:
                for cam_name, img in current_frames.items():
                    out_path = os.path.join(SAVE_DIR, f"{cam_name}_origin.png")
                    cv2.imwrite(out_path, img)
                    print(f"[SAVED] {cam_name} -> {out_path}")
                print(f"\n[SUCCESS] World origin frames captured across all cameras.")
                break
            elif key in [ord('q'), 27]:
                break

    finally:
        for p in rs_pipelines.values():
            p.stop()
        if basler_cam:
            basler_cam.StopGrabbing()
            basler_cam.Close()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()