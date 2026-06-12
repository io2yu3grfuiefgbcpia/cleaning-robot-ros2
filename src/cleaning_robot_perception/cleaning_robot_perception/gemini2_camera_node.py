"""ROS2 image publisher for direct Orbbec Gemini 2 capture."""

from __future__ import annotations

import time
from typing import Any

import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, CompressedImage, Image

from cleaning_robot_perception.gemini2_camera import Gemini2Camera


class Gemini2CameraNode(Node):
    """Publish Gemini 2 frames as ROS Image messages."""

    def __init__(self) -> None:
        super().__init__("gemini2_camera_node")

        self._declare_parameters()
        self.bridge = CvBridge()
        self.camera: Gemini2Camera | None = None
        self.last_open_attempt = 0.0
        self.last_frame_warning = 0.0

        self.frame_id = str(self.get_parameter("frame_id").value)
        self.retry_sec = float(self.get_parameter("open_retry_sec").value)
        self.publish_compressed = bool(
            self.get_parameter("publish_compressed").value
        )
        self.jpeg_quality = int(self.get_parameter("jpeg_quality").value)

        image_topic = str(self.get_parameter("image_topic").value)
        compat_topic = str(self.get_parameter("compat_image_topic").value)
        camera_info_topic = str(self.get_parameter("camera_info_topic").value)
        compressed_topic = str(self.get_parameter("compressed_topic").value)

        self.image_pub = self.create_publisher(Image, image_topic, 10)
        self.compat_image_pub = None
        if compat_topic and compat_topic != image_topic:
            self.compat_image_pub = self.create_publisher(Image, compat_topic, 10)
        self.camera_info_pub = self.create_publisher(
            CameraInfo,
            camera_info_topic,
            10,
        )
        self.compressed_pub = None
        if self.publish_compressed:
            self.compressed_pub = self.create_publisher(
                CompressedImage,
                compressed_topic,
                10,
            )

        fps = max(float(self.get_parameter("color_fps").value), 1.0)
        self.timer = self.create_timer(1.0 / fps, self.capture_once)
        self.open_camera()

    def _declare_parameters(self) -> None:
        self.declare_parameter("backend", "auto")
        self.declare_parameter("stream", "color")
        self.declare_parameter("device_id", 0)
        self.declare_parameter("device_path", "")
        self.declare_parameter("color_width", 640)
        self.declare_parameter("color_height", 480)
        self.declare_parameter("color_fps", 15)
        self.declare_parameter("jpeg_quality", 70)
        self.declare_parameter("frame_id", "gemini2_color_optical_frame")
        self.declare_parameter(
            "image_topic",
            "/cleaning_robot/gemini2/image_raw",
        )
        self.declare_parameter(
            "compat_image_topic",
            "/cleaning_robot/camera/left/image_raw",
        )
        self.declare_parameter(
            "camera_info_topic",
            "/cleaning_robot/gemini2/camera_info",
        )
        self.declare_parameter("publish_compressed", False)
        self.declare_parameter(
            "compressed_topic",
            "/cleaning_robot/gemini2/image_raw/compressed",
        )
        self.declare_parameter("open_retry_sec", 3.0)

    def _camera_config(self) -> dict[str, Any]:
        config = {
            "backend": self.get_parameter("backend").value,
            "stream": self.get_parameter("stream").value,
            "device_id": self.get_parameter("device_id").value,
            "color_width": self.get_parameter("color_width").value,
            "color_height": self.get_parameter("color_height").value,
            "color_fps": self.get_parameter("color_fps").value,
            "jpeg_quality": self.jpeg_quality,
        }
        device_path = str(self.get_parameter("device_path").value)
        if device_path:
            config["device_path"] = device_path
        return config

    def open_camera(self) -> bool:
        """Open the camera if it is not already open."""
        now = time.monotonic()
        if now - self.last_open_attempt < self.retry_sec:
            return False
        self.last_open_attempt = now

        if self.camera is not None:
            self.camera.release()

        self.camera = Gemini2Camera(self._camera_config())
        if self.camera.open():
            self.get_logger().info(
                "Gemini2 camera open: %s",
                self.camera.resolution_str(),
            )
            return True

        self.get_logger().warning(
            "Gemini2 camera not available; retrying every %.1fs",
            self.retry_sec,
        )
        return False

    def capture_once(self) -> None:
        """Capture and publish one frame."""
        if self.camera is None or self.camera.implementation is None:
            self.open_camera()
            return

        frame = self.camera.read_frame()
        if frame is None:
            now = time.monotonic()
            if now - self.last_frame_warning > 5.0:
                self.get_logger().warning("Gemini2 frame read returned no data")
                self.last_frame_warning = now
            return

        stamp = self.get_clock().now().to_msg()
        image_msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
        image_msg.header.stamp = stamp
        image_msg.header.frame_id = self.frame_id
        self.image_pub.publish(image_msg)
        if self.compat_image_pub is not None:
            self.compat_image_pub.publish(image_msg)

        info_msg = CameraInfo()
        info_msg.header.stamp = stamp
        info_msg.header.frame_id = self.frame_id
        info_msg.width = int(frame.shape[1])
        info_msg.height = int(frame.shape[0])
        self.camera_info_pub.publish(info_msg)

        if self.compressed_pub is not None:
            self.publish_compressed_frame(frame, stamp)

    def publish_compressed_frame(self, frame: Any, stamp: Any) -> None:
        """Publish a JPEG compressed image."""
        ok, buffer = cv2.imencode(
            ".jpg",
            frame,
            [int(cv2.IMWRITE_JPEG_QUALITY), self.jpeg_quality],
        )
        if not ok:
            return
        msg = CompressedImage()
        msg.header.stamp = stamp
        msg.header.frame_id = self.frame_id
        msg.format = "jpeg"
        msg.data = buffer.tobytes()
        self.compressed_pub.publish(msg)

    def destroy_node(self) -> None:
        if self.camera is not None:
            self.camera.release()
        super().destroy_node()


def main(args: list[str] | None = None) -> None:
    """Run the Gemini 2 image publisher."""
    rclpy.init(args=args)
    node = Gemini2CameraNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
