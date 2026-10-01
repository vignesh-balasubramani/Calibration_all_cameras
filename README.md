# calibration_test

Intrinsic and extrinsic calibration pipeline for a fixed multi-camera rig: 5 Intel
RealSense D435 cameras and 1 Basler ace 2 GigE camera, all calibrated against a shared
ChArUco (chessboard + AprilTag) target and a common world origin on the workstation table.

## Prerequisites

**Hardware**
- 5x Intel RealSense D435 (USB), 1x Basler ace 2 (GigE)
- A printed ChArUco board — generate one with `utils/generate_charuco_board.py`, mounted
  on a flat, rigid backing (foam board or similar). Measure the actual printed square and
  marker size with calipers and update `SQUARE_LENGTH_M` / `MARKER_LENGTH_M` in
  `src/calib_config.py` if they differ from the print job.

**Software**
- Python 3 with the packages in `src/requirements.txt` (OpenCV with `aruco` contrib
  modules, `pyrealsense2`, `pypylon`, `pyyaml`).
- Basler Pylon SDK installed on the host so `pypylon` can talk to the GigE camera.
- Optionally, run everything inside the provided Docker environment instead of installing
  dependencies on the host — see [Docker environment](#docker-environment) below.

**Configuration**
- Edit `src/calib_config.py` before running anything:
  - `CAMERAS`: RealSense serial number -> folder name for each of the 5 cameras.
  - `BASLER_CAMERAS`: Basler IP (or serial) -> folder name.
  - `SQUARES_X` / `SQUARES_Y` / `SQUARE_LENGTH_M` / `MARKER_LENGTH_M`: must match the
    printed board.
  - `realsense_resolution` / `basler_resolution`: capture resolution per camera type.

## Pipeline

The scripts are meant to be run in this order from inside `src/`:

1. **`generate_charuco_board.py`** (in `utils/`) — renders the ChArUco target as a
   print-ready PDF at the configured paper size and DPI, sized in millimeters from
   `utils/charuco_config.yaml`. Print it, mount it flat, and measure the actual square
   size before calibrating.

2. **`capture_calibration_images.py`** — walks through each RealSense camera, then the
   Basler camera, showing a live corner-detection overlay. Press `C` / `Space` to save a
   frame once enough corners are detected, `N` to move to the next camera, `Q` to quit.
   Move the board around each camera's field of view (different distances, angles, and
   corners of the frame) to get good coverage — 15-20 accepted frames per camera is a
   reasonable target. Images land in `calibration_data/images/<camera_name>/`.

3. **`calibrate_intrinsics.py`** — runs ChArUco camera calibration (`cv2.aruco.calibrateCameraCharuco`)
   over the captured images for every camera and writes `camera_matrix` and
   `distortion_coeff` into `calibration_data/images/<camera_name>/camera.json`.

4. **`capture_world_origin.py`** — streams all cameras at once in a grid preview. Place
   the board's top-left corner at the chosen table origin, align all cameras on it, then
   press `C` / `Space` to save one origin frame per camera to
   `calibration_data/world_origin_raw/<camera_name>_origin.png`.

5. **`calibrate_extrinsics.py`** — loads each camera's `camera.json` plus its origin
   frame, detects the board, and solves `solvePnP` to get the camera's pose relative to
   the table origin. Adds `t_param`, `r_param`, `rotation_matrix`, and `projection_matrix`
   into that same `camera.json`, then writes a combined
   `calibration_data/combined_calibration_data/camera_parameter_<timestamp>.json` with a
   `header` (board geometry, timestamp) and a `cameras` list of every camera's full
   calibration.

6. **`verify_world_projection.py`** — sanity check. Deprojects a sample pixel (the
   principal point of `camera_0` at 1 m depth) through its `camera.json` to confirm the
   resulting table coordinates look reasonable.

Re-run steps 2-5 whenever a camera is physically moved, and step 2-3 alone if only the
intrinsics need to be refreshed (e.g. after changing lens focus).

### Output layout and format

```
calibration_data/
├── images/<camera_name>/
│   ├── *.png              # step 2: raw calibration frames
│   └── camera.json        # step 3 writes intrinsics, step 5 adds pose
├── world_origin_raw/<camera_name>_origin.png   # step 4: origin frames
└── combined_calibration_data/
    └── camera_parameter_<timestamp>.json       # step 5: all cameras combined
```

`camera.json` uses the same schema as `mvs_extrinsic_calib`'s per-camera output, so
downstream code written against that pipeline can read this one without changes:

```json
{
    "camera_matrix": [[fx, 0, cx], [0, fy, cy], [0, 0, 1]],
    "distortion_coeff": [k1, k2, p1, p2, k3],
    "t_param": [[tx], [ty], [tz]],
    "r_param": [[rx], [ry], [rz]],
    "rotation_matrix": [[...]],
    "projection_matrix": [[...]],
    "camera": "camera_0"
}
```

`rotation_matrix` and `t_param` express the world -> camera transform (`X_cam = R @
X_world + t`), matching OpenCV's `solvePnP` convention. `src/json_io.py` provides the same
`to_json` / `from_json` helpers `mvs_extrinsic_calib` uses.

**One deliberate deviation:** `distortion_coeff` is not fixed-length here. `calibrate_intrinsics.py`
uses `cv2.CALIB_RATIONAL_MODEL` once a camera has 15+ valid frames, which produces 8
coefficients instead of 5. `mvs_extrinsic_calib` always pads its (MATLAB-sourced) distortion
vector to exactly 5 elements. Any downstream code that assumes a fixed 5-element
`distortion_coeff` needs to handle the 8-element case, or the affected camera needs to be
kept under 15 accepted frames to stay on the 5-coefficient model.

## Docker environment

`src/Dockerfile` and `src/docker-compose.yaml` set up an Ubuntu 22.04 container with the
Python dependencies pre-installed, X11 forwarding for the live preview windows, and host
networking/USB/device passthrough for the RealSense and Basler cameras.

```shell
cd src
./run_docker.sh
```

This grants the container access to the host X display, opens a `sysctl` socket buffer
tweak for reliable Basler GigE streaming, and drops you into an interactive shell inside
the container with the project bind-mounted at `/workspace`. Run the pipeline scripts from
there as described above.

## Shared code

- `src/camera_io.py` holds the RealSense/Basler camera setup and ChArUco
  detector/board construction used by all of the scripts above, so there's one place to
  change if the board definition or camera setup needs to change.
- `src/json_io.py` holds the `camera.json` read/write helpers, matching
  `mvs_extrinsic_calib`'s `util/json_formatter.py` (`to_json`/`from_json`, `NumpyEncoder`).
