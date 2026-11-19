"""End-to-end checks of the offline pipeline against synthetic ground truth."""
import importlib

import cv2
import numpy as np
import pytest

import synthetic
from calib_config import CalibCfg
from calibrate_extrinsics import PoseEstimationError, estimate_board_pose
from calibrate_intrinsics import calibrate_single_camera
from charuco import build_charuco_detector, detect_board
from geometry import (camera_position_in_world, camera_to_world_matrix, pixel_to_world,
                      world_to_camera_matrix)
from json_io import from_json


@pytest.fixture(scope="module")
def detector_board():
    return build_charuco_detector()


def _write_views(directory, n, seed=0):
    rng = np.random.default_rng(seed)
    directory.mkdir(parents=True, exist_ok=True)
    for i in range(n):
        rvec, tvec = synthetic.random_pose(rng)
        cv2.imwrite(str(directory / f"calib_{i:03d}.png"), synthetic.render_view(rvec, tvec))


@pytest.mark.parametrize("n_frames, rational", [(10, False), (18, True)])
def test_intrinsics_recovered(tmp_path, n_frames, rational):
    _write_views(tmp_path / "imgs", n_frames)
    out = tmp_path / "camera.json"

    result = calibrate_single_camera("cam", tmp_path / "imgs", out)

    assert result is not None
    K = np.array(result["camera_matrix"])
    assert K[0, 0] == pytest.approx(synthetic.K_TRUE[0, 0], rel=0.03)
    assert K[0, 2] == pytest.approx(synthetic.K_TRUE[0, 2], abs=8)
    assert K[1, 2] == pytest.approx(synthetic.K_TRUE[1, 2], abs=8)
    assert len(result["distortion_coeff"]) == (8 if rational else 5)
    assert set(from_json(out)) == {"camera_matrix", "distortion_coeff", "camera"}


def test_intrinsics_refused_with_too_few_frames(tmp_path):
    _write_views(tmp_path / "imgs", CalibCfg.MIN_FRAMES - 1)
    assert calibrate_single_camera("cam", tmp_path / "imgs", tmp_path / "camera.json") is None
    assert not (tmp_path / "camera.json").exists()


def test_extrinsics_recovered(detector_board):
    detector, board = detector_board
    rvec_true = cv2.Rodrigues(cv2.Rodrigues(np.deg2rad([20.0, -15.0, 10.0]))[0])[0]
    tvec_true = np.array([[-0.10], [-0.05], [0.70]])
    image = synthetic.render_view(rvec_true, tvec_true)

    rvec, tvec, rms = estimate_board_pose(image, synthetic.K_TRUE, np.zeros((5, 1)), detector, board)

    assert np.allclose(tvec, tvec_true, atol=0.005)
    assert np.degrees(np.linalg.norm(rvec - rvec_true)) < 0.5
    assert rms < 1.0


def test_camera_above_table_has_negative_world_z(detector_board):
    """World +Z points into the table, so a camera looking down at it sits at Z_w < 0."""
    detector, board = detector_board
    tvec_true = np.array([[-0.133], [-0.095], [0.60]])      # head-on view, 0.6 m away
    image = synthetic.render_view(np.zeros((3, 1)), tvec_true)

    rvec, tvec, _ = estimate_board_pose(image, synthetic.K_TRUE, np.zeros((5, 1)), detector, board)
    position = camera_position_in_world(cv2.Rodrigues(rvec)[0], tvec)

    assert position[2] == pytest.approx(-0.60, abs=0.01)


def test_pose_estimation_fails_on_blank_image(detector_board):
    detector, board = detector_board
    with pytest.raises(PoseEstimationError):
        estimate_board_pose(np.full((720, 1280, 3), 127, np.uint8), synthetic.K_TRUE,
                            np.zeros((5, 1)), detector, board)


def test_pixel_to_world_round_trip():
    rng = np.random.default_rng(1)
    rvec, tvec = synthetic.random_pose(rng)
    R = cv2.Rodrigues(rvec)[0]
    world_point = np.array([0.12, 0.08, 0.0])

    pixel, _ = cv2.projectPoints(world_point.reshape(1, 3), rvec, tvec, synthetic.K_TRUE, None)
    depth = (R @ world_point + tvec.ravel())[2]
    recovered = pixel_to_world(*pixel.ravel(), depth, synthetic.K_TRUE,
                               camera_to_world_matrix(R, tvec))

    assert np.allclose(recovered, world_point, atol=1e-6)


def test_pose_matrices_are_inverses():
    rvec, tvec = synthetic.random_pose(np.random.default_rng(2))
    R = cv2.Rodrigues(rvec)[0]
    product = world_to_camera_matrix(R, tvec) @ camera_to_world_matrix(R, tvec)
    assert np.allclose(product, np.eye(4), atol=1e-12)


def test_detection_validity_follows_min_corners(detector_board):
    detector, _ = detector_board
    good = detect_board(detector, synthetic.render_view(np.zeros((3, 1)), np.array([[-0.133], [-0.095], [0.6]])))
    blank = detect_board(detector, np.full((720, 1280), 127, np.uint8))
    assert good.is_valid and good.count >= CalibCfg.MIN_CORNERS
    assert not blank.is_valid and blank.count == 0


@pytest.mark.parametrize("module", [
    "calib_config", "charuco", "display", "geometry", "json_io", "cameras",
    "calibrate_intrinsics", "calibrate_extrinsics", "verify_world_projection",
    "capture_calibration_images", "capture_world_origin",
])
def test_modules_import_without_camera_sdks(module):
    """Importing must not require pyrealsense2/pypylon; only opening a camera does."""
    importlib.import_module(module)


def test_generate_board_pdf(tmp_path):
    import generate_charuco_board
    from pathlib import Path

    out = generate_charuco_board.generate(
        Path(generate_charuco_board.HERE / "charuco_config.yaml"), tmp_path / "board.pdf")
    assert out.read_bytes().startswith(b"%PDF")
