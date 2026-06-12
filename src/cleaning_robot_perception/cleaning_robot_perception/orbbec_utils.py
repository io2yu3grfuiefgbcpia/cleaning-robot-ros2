"""Frame conversion helpers for Orbbec pyorbbecsdk frames."""

from __future__ import annotations

from typing import Any

import cv2
import numpy as np

try:
    from pyorbbecsdk import OBFormat, VideoFrame
except ImportError:  # pragma: no cover - optional hardware SDK
    OBFormat = Any  # type: ignore
    VideoFrame = Any  # type: ignore


def _nv12_to_bgr(data: np.ndarray, width: int, height: int) -> np.ndarray:
    y_channel = data[0:height, :]
    uv_channel = data[height: height + height // 2].reshape(height // 2, width)
    yuv = cv2.merge([y_channel, uv_channel])
    return cv2.cvtColor(yuv, cv2.COLOR_YUV2BGR_NV12)


def frame_to_bgr_image(frame: VideoFrame) -> np.ndarray | None:
    """Convert a pyorbbecsdk VideoFrame to an OpenCV BGR image."""
    width = frame.get_width()
    height = frame.get_height()
    frame_format = frame.get_format()
    data = np.asanyarray(frame.get_data())

    if frame_format == OBFormat.RGB:
        image = data.reshape((height, width, 3))
        return cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
    if frame_format == OBFormat.BGR:
        return data.reshape((height, width, 3)).copy()
    if frame_format == OBFormat.YUYV:
        image = data.reshape((height, width, 2))
        return cv2.cvtColor(image, cv2.COLOR_YUV2BGR_YUYV)
    if frame_format == OBFormat.MJPG:
        return cv2.imdecode(data, cv2.IMREAD_COLOR)
    if frame_format == OBFormat.NV12:
        return _nv12_to_bgr(data, width, height)
    if frame_format == OBFormat.UYVY:
        image = data.reshape((height, width, 2))
        return cv2.cvtColor(image, cv2.COLOR_YUV2BGR_UYVY)
    return None
