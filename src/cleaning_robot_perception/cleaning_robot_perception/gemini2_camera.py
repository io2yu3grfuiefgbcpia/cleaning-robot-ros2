"""Direct Orbbec Gemini 2 capture helper with pyorbbec/OpenCV fallbacks."""

from __future__ import annotations

import logging
import time
from typing import Any

import cv2
import numpy as np

from cleaning_robot_perception.orbbec_utils import frame_to_bgr_image

LOGGER = logging.getLogger(__name__)

CAP_OBSENSOR = getattr(cv2, "CAP_OBSENSOR", 1400)
CAP_OBSENSOR_BGR = getattr(cv2, "CAP_OBSENSOR_BGR_IMAGE", 0)
CAP_OBSENSOR_DEPTH = getattr(cv2, "CAP_OBSENSOR_DEPTH_MAP", 1)


def _opencv_major() -> int:
    try:
        return int(str(getattr(cv2, "__version__", "4")).split(".")[0])
    except (ValueError, IndexError):
        return 4


def _depth_to_colormap(depth: np.ndarray) -> np.ndarray:
    dmin = 300
    dmax = 5000
    alpha = 255.0 / float(dmax - dmin)
    beta = -dmin * alpha
    normalized = cv2.convertScaleAbs(depth, alpha=alpha, beta=beta)
    return cv2.applyColorMap(normalized, cv2.COLORMAP_JET)


def _overlay_depth_on_bgr(
    bgr: np.ndarray,
    depth: np.ndarray,
    alpha: float = 0.4,
) -> np.ndarray:
    depth_color = _depth_to_colormap(depth)
    if depth_color.shape[:2] != bgr.shape[:2]:
        depth_color = cv2.resize(depth_color, (bgr.shape[1], bgr.shape[0]))
    return cv2.addWeighted(bgr, 1.0 - alpha, depth_color, alpha, 0)


class Gemini2Camera:
    """Gemini 2 camera wrapper with a simple OpenCV BGR frame interface."""

    def __init__(self, camera_config: dict[str, Any]) -> None:
        path = camera_config.get("device_path")
        device_id = camera_config.get("device_id", 0)
        if path:
            self._video_source: Any = str(path)
        elif isinstance(device_id, str):
            text = device_id.strip()
            if text.startswith("/dev/"):
                self._video_source = text
            elif text.isdigit():
                self._video_source = int(text)
            else:
                self._video_source = text
        else:
            self._video_source = int(device_id)

        self.backend = str(camera_config.get("backend", "auto"))
        self.stream = str(camera_config.get("stream", "color"))
        self.color_width = int(camera_config.get("color_width", 640))
        self.color_height = int(camera_config.get("color_height", 480))
        self.color_fps = int(camera_config.get("color_fps", 30))
        self.jpeg_quality = int(camera_config.get("jpeg_quality", 70))

        self._impl: str | None = None
        self._cap: cv2.VideoCapture | None = None
        self._pipeline: Any = None

    @property
    def implementation(self) -> str | None:
        """Return the currently active backend implementation name."""
        return self._impl

    def open(self) -> bool:
        """Open the camera by trying the configured backend order."""
        for name in self._backend_order():
            if name == "pyorbbec" and self._open_pyorbbec():
                self._impl = "pyorbbec"
                LOGGER.info("Gemini2 using pyorbbecsdk, stream=%s", self.stream)
                return True
            if name == "obsensor" and self._open_obsensor():
                self._impl = "obsensor"
                LOGGER.info("Gemini2 using OpenCV CAP_OBSENSOR")
                return True
            if name == "uvc" and self._open_uvc():
                self._impl = "uvc"
                return True

        LOGGER.error(
            "Gemini2 open failed. Install pyorbbecsdk or use an OpenCV/V4L2 "
            "device that exposes the Orbbec color stream."
        )
        return False

    def _backend_order(self) -> list[str]:
        if self.backend == "auto":
            return ["pyorbbec", "obsensor", "uvc"]
        return [self.backend]

    def _open_pyorbbec(self) -> bool:
        try:
            from pyorbbecsdk import (  # type: ignore
                Config,
                OBError,
                OBFormat,
                OBSensorType,
                Pipeline,
                VideoStreamProfile,
            )
        except ImportError:
            return False

        pipeline = Pipeline()
        config = Config()
        try:
            if self.stream in ("color", "depth_color"):
                profiles = pipeline.get_stream_profile_list(
                    OBSensorType.COLOR_SENSOR
                )
                try:
                    profile: VideoStreamProfile = profiles.get_video_stream_profile(
                        self.color_width,
                        self.color_height,
                        OBFormat.RGB,
                        self.color_fps,
                    )
                except OBError:
                    profile = profiles.get_default_video_stream_profile()
                config.enable_stream(profile)

            if self.stream in ("depth", "depth_color"):
                profiles = pipeline.get_stream_profile_list(
                    OBSensorType.DEPTH_SENSOR
                )
                config.enable_stream(profiles.get_default_video_stream_profile())

            if self.stream in ("ir", "ir_stereo"):
                profiles = pipeline.get_stream_profile_list(OBSensorType.IR_SENSOR)
                config.enable_stream(profiles.get_default_video_stream_profile())
                if self.stream == "ir_stereo":
                    try:
                        right_profiles = pipeline.get_stream_profile_list(
                            OBSensorType.IR_RIGHT_SENSOR
                        )
                        config.enable_stream(
                            right_profiles.get_default_video_stream_profile()
                        )
                    except Exception:
                        pass
        except Exception as exc:
            LOGGER.debug("pyorbbec stream config failed: %s", exc)
            return False

        try:
            pipeline.start(config)
        except Exception as exc:
            LOGGER.debug("pyorbbec pipeline start failed: %s", exc)
            return False

        self._pipeline = pipeline
        return True

    def _open_obsensor(self) -> bool:
        if self.stream in ("ir", "ir_stereo"):
            return False
        try:
            cap = cv2.VideoCapture(self._video_source, CAP_OBSENSOR)
        except TypeError:
            LOGGER.debug("OpenCV VideoCapture has no two-argument backend API")
            return False

        if not cap.isOpened():
            cap.release()
            return False
        self._cap = cap
        return True

    def _open_uvc(self) -> bool:
        if self.stream not in ("color", "depth_color"):
            LOGGER.warning("UVC supports color only; forcing stream=color")
            self.stream = "color"

        candidates = self._uvc_candidates()
        for label, factory in candidates:
            cap = self._try_open_uvc(label, factory)
            if cap is not None:
                self._cap = cap
                LOGGER.info("Gemini2 using UVC source %s (%s)",
                            self._video_source, label)
                return True
            time.sleep(0.25)
        return False

    def _uvc_candidates(self) -> list[tuple[str, Any]]:
        cap_v4l2 = getattr(cv2, "CAP_V4L2", getattr(cv2, "CAP_V4L", None))
        source = self._video_source
        by_index: list[tuple[str, Any]] = []
        by_path: list[tuple[str, Any]] = []

        if isinstance(source, str) and source.startswith("/dev/video"):
            tail = source[10:]
            if tail.isdigit():
                index = int(tail)
                by_index.append(
                    (f"VideoCapture({index})",
                     lambda idx=index: cv2.VideoCapture(idx))
                )
                if cap_v4l2 is not None:
                    by_index.append(
                        (
                            f"VideoCapture({index},CAP_V4L2)",
                            lambda idx=index: cv2.VideoCapture(idx, cap_v4l2),
                        )
                    )
            by_path.append(("VideoCapture(path)",
                            lambda: cv2.VideoCapture(source)))
            if cap_v4l2 is not None:
                by_path.append(
                    (
                        "VideoCapture(path,CAP_V4L2)",
                        lambda: cv2.VideoCapture(source, cap_v4l2),
                    )
                )
        elif isinstance(source, int):
            index = source
            by_index.append(
                (f"VideoCapture({index})",
                 lambda idx=index: cv2.VideoCapture(idx))
            )
            if cap_v4l2 is not None:
                by_index.append(
                    (
                        f"VideoCapture({index},CAP_V4L2)",
                        lambda idx=index: cv2.VideoCapture(idx, cap_v4l2),
                    )
                )
            if _opencv_major() >= 4:
                device_path = f"/dev/video{index}"
                by_path.append(
                    (f"VideoCapture({device_path})",
                     lambda path=device_path: cv2.VideoCapture(path))
                )
                if cap_v4l2 is not None:
                    by_path.append(
                        (
                            f"VideoCapture({device_path},CAP_V4L2)",
                            lambda path=device_path: cv2.VideoCapture(
                                path,
                                cap_v4l2,
                            ),
                        )
                    )
        else:
            by_path.append(("VideoCapture(source)",
                            lambda: cv2.VideoCapture(source)))

        if _opencv_major() < 4:
            return by_index + by_path
        return by_path + by_index

    def _try_open_uvc(self, label: str, factory: Any) -> cv2.VideoCapture | None:
        try:
            cap = factory()
        except TypeError:
            return None
        except Exception as exc:
            LOGGER.debug("UVC %s open exception: %s", label, exc)
            return None

        if cap is None or not cap.isOpened():
            if cap is not None:
                cap.release()
            return None

        cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.color_width)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.color_height)
        cap.set(cv2.CAP_PROP_FPS, self.color_fps)
        buffer_size = getattr(cv2, "CAP_PROP_BUFFERSIZE", None)
        if buffer_size is not None:
            try:
                cap.set(buffer_size, 1)
            except Exception:
                pass

        ok, _frame = cap.read()
        if not ok:
            for _ in range(8):
                time.sleep(0.15)
                ok, _frame = cap.read()
                if ok:
                    break

        if not ok and _opencv_major() < 4 and cap.isOpened():
            LOGGER.debug("UVC %s accepted despite first-frame failure", label)
            return cap
        if not ok:
            cap.release()
            return None
        return cap

    def read_frame(self) -> np.ndarray | None:
        """Read one frame as an OpenCV BGR image."""
        if self._impl == "pyorbbec":
            return self._read_pyorbbec()
        if self._impl == "obsensor":
            return self._read_obsensor()
        if self._impl == "uvc":
            return self._read_uvc()
        return None

    def _read_pyorbbec(self) -> np.ndarray | None:
        frames = self._pipeline.wait_for_frames(200)
        if frames is None:
            return None

        if self.stream == "color":
            color_frame = frames.get_color_frame()
            return None if color_frame is None else frame_to_bgr_image(color_frame)

        if self.stream == "depth":
            depth_frame = frames.get_depth_frame()
            if depth_frame is None:
                return None
            width = depth_frame.get_width()
            height = depth_frame.get_height()
            raw = np.frombuffer(
                depth_frame.get_data(),
                dtype=np.uint16,
            ).reshape((height, width))
            return _depth_to_colormap(raw)

        if self.stream == "depth_color":
            color_frame = frames.get_color_frame()
            depth_frame = frames.get_depth_frame()
            if color_frame is None:
                return None
            bgr = frame_to_bgr_image(color_frame)
            if bgr is None or depth_frame is None:
                return bgr
            width = depth_frame.get_width()
            height = depth_frame.get_height()
            raw = np.frombuffer(
                depth_frame.get_data(),
                dtype=np.uint16,
            ).reshape((height, width))
            return _overlay_depth_on_bgr(bgr, raw)

        if self.stream == "ir":
            ir_frame = frames.get_ir_frame()
            if ir_frame is None:
                return None
            width = ir_frame.get_width()
            height = ir_frame.get_height()
            gray = np.frombuffer(
                ir_frame.get_data(),
                dtype=np.uint8,
            ).reshape((height, width))
            return cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)

        if self.stream == "ir_stereo":
            left_frame = frames.get_ir_frame()
            right_frame = None
            try:
                right_frame = frames.get_ir_right_frame()
            except Exception:
                pass
            if left_frame is None:
                return None
            left = self._ir_frame_to_bgr(left_frame)
            if right_frame is None:
                return left
            right = self._ir_frame_to_bgr(right_frame)
            if right.shape[0] != left.shape[0]:
                right = cv2.resize(right, (left.shape[1], left.shape[0]))
            return np.hstack([left, right])

        return None

    @staticmethod
    def _ir_frame_to_bgr(frame: Any) -> np.ndarray:
        width = frame.get_width()
        height = frame.get_height()
        gray = np.frombuffer(frame.get_data(), dtype=np.uint8).reshape(
            (height, width)
        )
        return cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)

    def _read_obsensor(self) -> np.ndarray | None:
        if self._cap is None or not self._cap.grab():
            return None
        if self.stream == "color":
            ok, bgr = self._cap.retrieve(None, CAP_OBSENSOR_BGR)
            return bgr if ok else None
        if self.stream == "depth":
            ok, depth = self._cap.retrieve(None, CAP_OBSENSOR_DEPTH)
            return _depth_to_colormap(depth) if ok else None
        if self.stream == "depth_color":
            ok_color, bgr = self._cap.retrieve(None, CAP_OBSENSOR_BGR)
            ok_depth, depth = self._cap.retrieve(None, CAP_OBSENSOR_DEPTH)
            if ok_color and ok_depth:
                return _overlay_depth_on_bgr(bgr, depth)
            return bgr if ok_color else None
        return None

    def _read_uvc(self) -> np.ndarray | None:
        if self._cap is None:
            return None
        for _ in range(2):
            self._cap.grab()
        ok, frame = self._cap.read()
        return frame if ok else None

    def encode_jpeg(self, frame: np.ndarray) -> bytes | None:
        """Encode an OpenCV BGR frame as JPEG bytes."""
        ok, buffer = cv2.imencode(
            ".jpg",
            frame,
            [int(cv2.IMWRITE_JPEG_QUALITY), self.jpeg_quality],
        )
        return buffer.tobytes() if ok else None

    def resolution_str(self) -> str:
        """Return a human-readable capture resolution/backend string."""
        return f"{self.color_width}x{self.color_height}@{self._impl or 'none'}"

    def release(self) -> None:
        """Release camera resources."""
        if self._pipeline is not None:
            try:
                self._pipeline.stop()
            except Exception:
                pass
            self._pipeline = None
        if self._cap is not None:
            self._cap.release()
            self._cap = None
        self._impl = None
