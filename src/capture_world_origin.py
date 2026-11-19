#!/usr/bin/env python3
"""Step 4: capture one simultaneous "world origin" frame from every camera.

Place the ChArUco board flat on the table with its top-left corner on the point you
want as the world origin, make sure every camera sees it, then press C / Space.

Controls:  [C]/[Space] capture all cameras  |  [Q]/[Esc] quit without saving
"""
import contextlib

import cv2

from calib_config import CalibCfg, camera_specs
from cameras import open_source
from charuco import build_charuco_detector, detect_board
from display import (KEYS_QUIT, KEYS_SAVE, draw_corners, draw_status, make_grid,
                     read_key, status_text)

WINDOW = "Workstation Origin Alignment Preview"


def main() -> None:
    detector, _ = build_charuco_detector()
    CalibCfg.ORIGIN_DIR.mkdir(parents=True, exist_ok=True)

    with contextlib.ExitStack() as stack:
        sources = {}
        for spec in camera_specs():
            try:
                sources[spec.name] = stack.enter_context(open_source(spec))
            except Exception as exc:
                print(f"[WARN] Failed to open {spec.kind} {spec.name} ({spec.device_id}): {exc}")
        if not sources:
            print("[ERROR] No cameras could be opened.")
            return

        print("=" * 65)
        print("WORKSTATION WORLD ORIGIN FRAME CAPTURE")
        print("Put the ChArUco board's top-left corner on the chosen table origin.")
        print("Controls: [Space]/[C] capture origin frames | [Q]/[Esc] quit")
        print("=" * 65)

        latest = {}   # newest frame per camera
        tiles = {}    # newest annotated preview per camera (keeps grid layout stable)
        try:
            while True:
                for name, source in sources.items():
                    frame = source.read()
                    if frame is None:
                        continue
                    detection = detect_board(detector, frame)
                    tile = draw_corners(frame, detection)
                    tile = cv2.resize(tile, (380, 215))
                    draw_status(tile, status_text(name, detection), detection.is_valid)
                    latest[name], tiles[name] = frame, tile

                if not tiles:
                    continue
                cv2.imshow(WINDOW, make_grid([tiles[n] for n in sources if n in tiles]))

                key = read_key()
                if key in KEYS_SAVE:
                    for name, frame in latest.items():
                        out_path = CalibCfg.ORIGIN_DIR / f"{name}_origin.png"
                        cv2.imwrite(str(out_path), frame)
                        print(f"[SAVED] {name} -> {out_path}")
                    missing = set(sources) - set(latest)
                    if missing:
                        print(f"[WARN] No frame received from: {', '.join(sorted(missing))}")
                    print("\n[SUCCESS] World origin frames captured.")
                    break
                if key in KEYS_QUIT:
                    break
        finally:
            cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
