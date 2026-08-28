#!/usr/bin/env python3
import os
import sys
import cv2
import numpy as np
import pyrealsense2 as rs
from pypylon import pylon
from calib_config import CalibCfg


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


def capture_realsense(serial: str, cam_name: str, detector):
    save_dir = os.path.join(CalibCfg.CALIB_DIR, "images", cam_name)
    os.makedirs(save_dir, exist_ok=True)

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

    try:
        pipeline.start(config)
    except Exception as e:
        print(f"[ERROR] Failed to start RealSense {cam_name} ({serial}): {e}")
        return

    print("=" * 60)
    print(f"Active Camera: {cam_name} (RealSense: {serial})")
    print("Controls:")
    print("  [C] or [Space] : Save calibration frame (when >= 6 corners detected)")
    print("  [N]            : Finish this camera and move to next")
    print("  [Q]            : Exit capture session")
    print("=" * 60)

    count = len([f for f in os.listdir(save_dir) if f.endswith(CalibCfg.IMAGE_EXT)])
    
    while True:
        frames = pipeline.wait_for_frames(timeout_ms=5000)
        color_frame = frames.get_color_frame()
        if not color_frame:
            continue

        frame = np.asanyarray(color_frame.get_data())
        display = frame.copy()

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, _, _ = detector.detectBoard(gray)

        num_corners = len(corners) if corners is not None else 0
        is_valid = num_corners >= CalibCfg.MIN_CORNERS

        if corners is not None and ids is not None:
            cv2.aruco.drawDetectedCornersCharuco(display, corners, ids, (0, 255, 0))

        preview = cv2.resize(display, (960, 540))
        status_color = (0, 255, 0) if is_valid else (0, 0, 255)
        status_msg = f"[{cam_name}] Saved: {count} | Corners: {num_corners} ({'VALID' if is_valid else 'NEED >= 6'})"
        
        cv2.rectangle(preview, (10, 10), (700, 50), (20, 20, 20), -1)
        cv2.putText(preview, status_msg, (20, 38), cv2.FONT_HERSHEY_SIMPLEX, 0.7, status_color, 2)
        cv2.imshow("Calibration Acquisition", preview)

        key = cv2.waitKey(1) & 0xFF
        if key in [ord('c'), 32]:  # 'c' or Space
            if is_valid:
                img_path = os.path.join(save_dir, f"calib_{count:03d}.{CalibCfg.IMAGE_EXT}")
                cv2.imwrite(img_path, frame)
                print(f"[{cam_name}] Saved frame {count:03d} -> {img_path} ({num_corners} corners)")
                count += 1
            else:
                print(f"[{cam_name}] Rejected: Insufficient corners ({num_corners} < {CalibCfg.MIN_CORNERS})")
        elif key == ord('n'):
            break
        elif key in [ord('q'), 27]:
            pipeline.stop()
            cv2.destroyAllWindows()
            sys.exit(0)

    pipeline.stop()
    cv2.destroyAllWindows()


def capture_basler(target_id: str, cam_name: str, detector):
    save_dir = os.path.join(CalibCfg.CALIB_DIR, "images", cam_name)
    os.makedirs(save_dir, exist_ok=True)

    tl_factory = pylon.TlFactory.GetInstance()
    if "." in target_id:
        info = pylon.DeviceInfo()
        info.SetPropertyValue("IpAddress", target_id)
        camera = pylon.InstantCamera(tl_factory.CreateDevice(info))
    else:
        info = pylon.DeviceInfo()
        info.SetSerialNumber(target_id)
        camera = pylon.InstantCamera(tl_factory.CreateDevice(info))

    try:
        camera.Open()
        if camera.IsGrabbing():
                    camera.StopGrabbing()
        
            # ----------------- 1. RESOLUTION / ROI -----------------
        if hasattr(camera, "OffsetX"):
            camera.OffsetX.SetValue(0)
        if hasattr(camera, "OffsetY"):
            camera.OffsetY.SetValue(0)

        target_w, target_h = CalibCfg.basler_resolution
        max_w = camera.Width.GetMax()
        max_h = camera.Height.GetMax()

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
        camera.Height.SetValue(target_h)

        off_x = (max_w - target_w) // 2
        off_y = (max_h - target_h) // 2
        if hasattr(camera, "OffsetX"):
            inc_x = camera.OffsetX.GetInc()
            camera.OffsetX.SetValue((off_x // inc_x) * inc_x)
        if hasattr(camera, "OffsetY"):
            inc_y = camera.OffsetY.GetInc()
            camera.OffsetY.SetValue((off_y // inc_y) * inc_y)
        camera.StartGrabbing(pylon.GrabStrategy_LatestImageOnly)
    except Exception as e:
        print(f"[ERROR] Failed to connect to Basler {cam_name} ({target_id}): {e}")
        return

    converter = pylon.ImageFormatConverter()
    converter.OutputPixelFormat = pylon.PixelType_BGR8packed

    print("=" * 60)
    print(f"Active Camera: {cam_name} (Basler: {target_id})")
    print("Controls:")
    print("  [C] or [Space] : Save calibration frame (when >= 6 corners detected)")
    print("  [N]            : Finish this camera and move to next")
    print("  [Q]            : Exit capture session")
    print("=" * 60)

    count = len([f for f in os.listdir(save_dir) if f.endswith(CalibCfg.IMAGE_EXT)])

    while camera.IsGrabbing():
        grab_result = camera.RetrieveResult(5000, pylon.TimeoutHandling_ThrowException)
        if grab_result.GrabSucceeded():
            frame = converter.Convert(grab_result).GetArray()
            display = frame.copy()

            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            corners, ids, _, _ = detector.detectBoard(gray)

            num_corners = len(corners) if corners is not None else 0
            is_valid = num_corners >= CalibCfg.MIN_CORNERS

            if corners is not None and ids is not None:
                cv2.aruco.drawDetectedCornersCharuco(display, corners, ids, (0, 255, 0))

            preview = cv2.resize(display, (960, 540))
            status_color = (0, 255, 0) if is_valid else (0, 0, 255)
            status_msg = f"[{cam_name}] Saved: {count} | Corners: {num_corners} ({'VALID' if is_valid else 'NEED >= 6'})"

            cv2.rectangle(preview, (10, 10), (700, 50), (20, 20, 20), -1)
            cv2.putText(preview, status_msg, (20, 38), cv2.FONT_HERSHEY_SIMPLEX, 0.7, status_color, 2)
            cv2.imshow("Calibration Acquisition", preview)

            key = cv2.waitKey(1) & 0xFF
            if key in [ord('c'), 32]:
                if is_valid:
                    img_path = os.path.join(save_dir, f"calib_{count:03d}.{CalibCfg.IMAGE_EXT}")
                    cv2.imwrite(img_path, frame)
                    print(f"[{cam_name}] Saved frame {count:03d} -> {img_path} ({num_corners} corners)")
                    count += 1
                else:
                    print(f"[{cam_name}] Rejected: Insufficient corners ({num_corners} < {CalibCfg.MIN_CORNERS})")
            elif key == ord('n'):
                grab_result.Release()
                break
            elif key in [ord('q'), 27]:
                grab_result.Release()
                camera.StopGrabbing()
                camera.Close()
                cv2.destroyAllWindows()
                sys.exit(0)

        grab_result.Release()

    camera.StopGrabbing()
    camera.Close()
    cv2.destroyAllWindows()


def main():
    detector, _ = get_detector()

    # 1. Capture RealSense cameras sequentially
    for serial, cam_name in CalibCfg.CAMERAS.items():
        capture_realsense(serial, cam_name, detector)

    # 2. Capture Basler camera
    if CalibCfg.USE_BASLER:
        for target_id, cam_name in CalibCfg.BASLER_CAMERAS.items():
            capture_basler(target_id, cam_name, detector)

    print("\n[SUCCESS] Image acquisition completed for all cameras.")


if __name__ == "__main__":
    main()