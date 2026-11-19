#!/usr/bin/env python3
"""Step 2: interactively capture ChArUco calibration frames, one camera at a time.

Controls:  [C]/[Space] save frame  |  [N] next camera  |  [Q]/[Esc] quit
"""
import cv2

from calib_config import CalibCfg, CameraSpec, camera_dir, camera_specs
from cameras import open_source
from charuco import build_charuco_detector, detect_board
from display import (KEYS_NEXT, KEYS_QUIT, KEYS_SAVE, draw_corners, draw_status,
                     read_key, resize_to_width, status_text)

WINDOW = "Calibration Acquisition"
PREVIEW_WIDTH = 960


def _next_free_path(save_dir, index):
    """First calib_NNN.<ext> at or after `index` that does not exist yet (never overwrites)."""
    while True:
        path = save_dir / f"calib_{index:03d}.{CalibCfg.IMAGE_EXT}"
        if not path.exists():
            return path
        index += 1


def capture_camera(spec: CameraSpec, detector) -> bool:
    """Capture frames from one camera. Returns False if the user asked to quit."""
    save_dir = camera_dir(spec.name)
    save_dir.mkdir(parents=True, exist_ok=True)

    try:
        source = open_source(spec)
    except Exception as exc:
        print(f"[ERROR] Failed to open {spec.kind} {spec.name} ({spec.device_id}): {exc}")
        return True  # skip this camera, carry on with the rest

    print("=" * 60)
    print(f"Active camera: {spec.name} ({spec.kind}: {spec.device_id})")
    print(f"  [C] / [Space] : save frame (needs >= {CalibCfg.MIN_CORNERS} corners)")
    print("  [N]           : done with this camera, go to the next")
    print("  [Q] / [Esc]   : quit")
    print("=" * 60)

    saved = len(list(save_dir.glob(f"*.{CalibCfg.IMAGE_EXT}")))
    keep_going = True
    with source:
        while True:
            frame = source.read()
            if frame is None:
                continue

            detection = detect_board(detector, frame)
            preview = resize_to_width(draw_corners(frame, detection), PREVIEW_WIDTH)
            draw_status(preview, f"{status_text(spec.name, detection)} | saved: {saved}",
                        detection.is_valid)
            cv2.imshow(WINDOW, preview)

            key = read_key()
            if key in KEYS_SAVE:
                if detection.is_valid:
                    path = _next_free_path(save_dir, saved)
                    cv2.imwrite(str(path), frame)
                    saved += 1
                    print(f"[{spec.name}] saved {path.name} ({detection.count} corners)")
                else:
                    print(f"[{spec.name}] rejected: {detection.count} < {CalibCfg.MIN_CORNERS} corners")
            elif key in KEYS_NEXT:
                break
            elif key in KEYS_QUIT:
                keep_going = False
                break

    cv2.destroyAllWindows()
    return keep_going


def main() -> None:
    detector, _ = build_charuco_detector()
    for spec in camera_specs():
        if not capture_camera(spec, detector):
            print("\nQuit requested; stopping early.")
            return
    print("\n[SUCCESS] Image acquisition completed for all cameras.")


if __name__ == "__main__":
    main()
