#!/usr/bin/env python3
import os
import cv2
import numpy as np
from pypylon import pylon
from calib_config import CalibCfg
from camera_io import build_charuco_detector, open_realsense_pipeline, open_basler_camera

SAVE_DIR = os.path.join(CalibCfg.CALIB_DIR, "world_origin_raw")
os.makedirs(SAVE_DIR, exist_ok=True)


def main():
    detector, board = build_charuco_detector()

    rs_pipelines = {}
    for serial, name in CalibCfg.CAMERAS.items():
        try:
            rs_pipelines[name] = open_realsense_pipeline(serial)
        except Exception as e:
            print(f"[WARN] Failed to connect RealSense {name} ({serial}): {e}")

    basler_cam = None
    converter = None
    if CalibCfg.USE_BASLER:
        for target_id, name in CalibCfg.BASLER_CAMERAS.items():
            try:
                basler_cam, converter = open_basler_camera(target_id)
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