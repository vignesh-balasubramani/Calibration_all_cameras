#!/usr/bin/env python3
"""Uniform frame-grabbing interface over Intel RealSense and Basler GigE cameras.

The vendor SDKs (`pyrealsense2`, `pypylon`) are imported lazily, so the offline
scripts (intrinsics/extrinsics solving, verification) work on a machine that has
neither SDK installed.
"""
from abc import ABC, abstractmethod
from typing import Optional

import numpy as np

from calib_config import CalibCfg, CameraSpec

GRAB_TIMEOUT_MS = 5000


class FrameSource(ABC):
    """A camera that yields BGR frames. Usable as a context manager."""

    @abstractmethod
    def read(self) -> Optional[np.ndarray]:
        """Block for the next BGR frame, or return None if this grab produced none."""

    @abstractmethod
    def close(self) -> None:
        """Stop streaming and release the device."""

    def __enter__(self) -> "FrameSource":
        return self

    def __exit__(self, *exc_info) -> None:
        self.close()


class RealSenseSource(FrameSource):
    def __init__(self, serial: str):
        import pyrealsense2 as rs

        config = rs.config()
        config.enable_device(serial)
        config.enable_stream(
            rs.stream.color,
            CalibCfg.REALSENSE_RESOLUTION[0],
            CalibCfg.REALSENSE_RESOLUTION[1],
            rs.format.bgr8,
            CalibCfg.REALSENSE_FPS,
        )
        self._pipeline = rs.pipeline()
        self._pipeline.start(config)

    def read(self) -> Optional[np.ndarray]:
        frames = self._pipeline.wait_for_frames(timeout_ms=GRAB_TIMEOUT_MS)
        color = frames.get_color_frame()
        # Copy: the SDK recycles the underlying buffer once the frame is released.
        return np.asanyarray(color.get_data()).copy() if color else None

    def close(self) -> None:
        self._pipeline.stop()


class BaslerSource(FrameSource):
    def __init__(self, target_id: str):
        """`target_id` is an IPv4 address (contains a '.') or a serial number."""
        from pypylon import pylon

        self._pylon = pylon
        info = pylon.DeviceInfo()
        if "." in target_id:
            info.SetPropertyValue("IpAddress", target_id)
        else:
            info.SetSerialNumber(target_id)

        self._camera = pylon.InstantCamera(pylon.TlFactory.GetInstance().CreateDevice(info))
        self._camera.Open()
        if self._camera.IsGrabbing():
            self._camera.StopGrabbing()
        _configure_centered_roi(self._camera)
        self._camera.StartGrabbing(pylon.GrabStrategy_LatestImageOnly)

        self._converter = pylon.ImageFormatConverter()
        self._converter.OutputPixelFormat = pylon.PixelType_BGR8packed

    def read(self) -> Optional[np.ndarray]:
        result = self._camera.RetrieveResult(
            GRAB_TIMEOUT_MS, self._pylon.TimeoutHandling_ThrowException
        )
        try:
            if not result.GrabSucceeded():
                return None
            return self._converter.Convert(result).GetArray().copy()
        finally:
            result.Release()

    def close(self) -> None:
        if self._camera.IsGrabbing():
            self._camera.StopGrabbing()
        self._camera.Close()


def _configure_centered_roi(camera) -> None:
    """Crop the sensor to CalibCfg.BASLER_RESOLUTION, centered, honouring step sizes."""
    has_offsets = hasattr(camera, "OffsetX") and hasattr(camera, "OffsetY")
    if has_offsets:  # offsets must be zero before the size can be changed freely
        camera.OffsetX.SetValue(0)
        camera.OffsetY.SetValue(0)

    max_w, max_h = camera.Width.GetMax(), camera.Height.GetMax()
    width = min(int(CalibCfg.BASLER_RESOLUTION[0]), max_w)
    height = min(int(CalibCfg.BASLER_RESOLUTION[1]), max_h)
    width -= width % camera.Width.GetInc()
    height -= height % camera.Height.GetInc()
    camera.Width.SetValue(width)
    camera.Height.SetValue(height)

    if has_offsets:
        off_x, off_y = (max_w - width) // 2, (max_h - height) // 2
        camera.OffsetX.SetValue(off_x - off_x % camera.OffsetX.GetInc())
        camera.OffsetY.SetValue(off_y - off_y % camera.OffsetY.GetInc())


def open_source(spec: CameraSpec) -> FrameSource:
    """Open and start streaming the camera described by `spec`."""
    if spec.kind == "realsense":
        return RealSenseSource(spec.device_id)
    if spec.kind == "basler":
        return BaslerSource(spec.device_id)
    raise ValueError(f"Unknown camera kind {spec.kind!r} for {spec.name}")
