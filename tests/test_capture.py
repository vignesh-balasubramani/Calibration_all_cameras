"""Headless smoke tests for the interactive capture scripts, using fake cameras and key presses."""
import cv2
import numpy as np
import pytest

import capture_calibration_images
import capture_world_origin
import synthetic
from calib_config import CalibCfg
from cameras import FrameSource, open_source


class FakeSource(FrameSource):
    """Yields a good board view, with every 3rd read simulating a dropped frame."""

    def __init__(self):
        self.reads, self.closed = 0, False
        self.frame = synthetic.render_view(np.zeros((3, 1)), np.array([[-0.133], [-0.095], [0.6]]))

    def read(self):
        self.reads += 1
        return None if self.reads % 3 == 0 else self.frame.copy()

    def close(self):
        self.closed = True


@pytest.fixture
def headless(monkeypatch, tmp_path):
    """Redirect output to tmp_path, fake the cameras and the GUI, and script the key presses."""
    sources = []

    def fake_open(spec):
        sources.append(FakeSource())
        return sources[-1]

    monkeypatch.setattr(CalibCfg, "IMAGES_DIR", tmp_path / "images")
    monkeypatch.setattr(CalibCfg, "ORIGIN_DIR", tmp_path / "origin")
    monkeypatch.setattr(CalibCfg, "REALSENSE_CAMERAS", {"111": "cam_a", "222": "cam_b"})
    monkeypatch.setattr(CalibCfg, "USE_BASLER", False)
    monkeypatch.setattr(cv2, "imshow", lambda *a, **k: None)
    monkeypatch.setattr(cv2, "destroyAllWindows", lambda: None)
    for module in (capture_calibration_images, capture_world_origin):
        monkeypatch.setattr(module, "open_source", fake_open)

    def script_keys(*keys):
        it = iter(keys)
        for module in (capture_calibration_images, capture_world_origin):
            monkeypatch.setattr(module, "read_key", lambda: next(it, ord("q")))

    return tmp_path, sources, script_keys


def test_capture_saves_frames_then_advances_cameras(headless):
    tmp, sources, script_keys = headless
    # cam_a: save, save, next | cam_b: save, quit
    script_keys(ord("c"), ord(" "), ord("n"), ord("c"), ord("q"))

    capture_calibration_images.main()

    assert sorted(p.name for p in (tmp / "images/cam_a").iterdir()) == ["calib_000.png", "calib_001.png"]
    assert [p.name for p in (tmp / "images/cam_b").iterdir()] == ["calib_000.png"]
    assert all(s.closed for s in sources)


def test_capture_never_overwrites_existing_frames(headless):
    tmp, _, script_keys = headless
    existing = tmp / "images/cam_a"
    existing.mkdir(parents=True)
    cv2.imwrite(str(existing / "calib_000.png"), np.zeros((4, 4), np.uint8))
    script_keys(ord("c"), ord("q"))

    capture_calibration_images.main()

    assert sorted(p.name for p in existing.iterdir()) == ["calib_000.png", "calib_001.png"]
    assert cv2.imread(str(existing / "calib_000.png")).shape[:2] == (4, 4)


def test_world_origin_saves_one_frame_per_camera(headless):
    tmp, sources, script_keys = headless
    script_keys(ord("c"))

    capture_world_origin.main()

    assert sorted(p.name for p in (tmp / "origin").iterdir()) == ["cam_a_origin.png", "cam_b_origin.png"]
    assert all(s.closed for s in sources)


def test_world_origin_quit_saves_nothing(headless):
    tmp, sources, script_keys = headless
    script_keys(ord("q"))

    capture_world_origin.main()

    assert list((tmp / "origin").iterdir()) == []
    assert all(s.closed for s in sources)


def test_open_source_rejects_unknown_camera_kind():
    from calib_config import CameraSpec
    with pytest.raises(ValueError):
        open_source(CameraSpec("x", "webcam", "0"))
