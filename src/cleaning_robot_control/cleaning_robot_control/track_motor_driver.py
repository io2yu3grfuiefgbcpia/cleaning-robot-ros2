"""ROS2 node that converts Twist commands into tracked motor GPIO output."""

from __future__ import annotations

import json
import time
from typing import Any

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import String

from cleaning_robot_control.track_motors import TrackMotors


def _clamp(value: float, limit: float) -> float:
    if limit <= 0:
        return value
    return max(-limit, min(limit, value))


class TrackMotorDriver(Node):
    """Drive a differential tracked base from ROS Twist messages."""

    def __init__(self) -> None:
        super().__init__("track_motor_driver")

        self._declare_parameters()
        self.max_linear_speed = float(
            self.get_parameter("max_linear_speed").value
        )
        self.max_angular_speed = float(
            self.get_parameter("max_angular_speed").value
        )
        self.wheel_base = float(self.get_parameter("wheel_base").value)
        self.command_timeout_sec = float(
            self.get_parameter("command_timeout_sec").value
        )
        self.invert_left = bool(self.get_parameter("invert_left").value)
        self.invert_right = bool(self.get_parameter("invert_right").value)
        self.left_trim = float(self.get_parameter("left_trim").value)
        self.right_trim = float(self.get_parameter("right_trim").value)

        motor_config = self._motor_config_from_parameters()
        self.motors = TrackMotors(motor_config)

        self.last_command_time = 0.0
        self.last_left_percent = 0
        self.last_right_percent = 0
        self._stopped_by_watchdog = True

        cmd_vel_topic = str(self.get_parameter("cmd_vel_topic").value)
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            cmd_vel_topic,
            self.cmd_vel_callback,
            10,
        )

        extra_topic = str(self.get_parameter("extra_cmd_vel_topic").value)
        self.extra_cmd_vel_sub = None
        if extra_topic and extra_topic != cmd_vel_topic:
            self.extra_cmd_vel_sub = self.create_subscription(
                Twist,
                extra_topic,
                self.cmd_vel_callback,
                10,
            )

        self.status_pub = self.create_publisher(
            String,
            "/cleaning_robot/motor_status",
            10,
        )
        self.watchdog_timer = self.create_timer(0.05, self.watchdog_loop)
        self.status_timer = self.create_timer(1.0, self.publish_status)

        state = self.motors.get_state()
        self.get_logger().info(
            "Track motor driver started: topic=%s driver=%s mock=%s backend=%s",
            cmd_vel_topic,
            state["driver"],
            state["mock"],
            state["gpio_backend"],
        )

    def _declare_parameters(self) -> None:
        self.declare_parameter("cmd_vel_topic", "/cleaning_robot/cmd_vel")
        self.declare_parameter("extra_cmd_vel_topic", "")
        self.declare_parameter("max_linear_speed", 0.5)
        self.declare_parameter("max_angular_speed", 1.5)
        self.declare_parameter("wheel_base", 0.15)
        self.declare_parameter("command_timeout_sec", 0.5)
        self.declare_parameter("invert_left", False)
        self.declare_parameter("invert_right", False)
        self.declare_parameter("left_trim", 1.0)
        self.declare_parameter("right_trim", 1.0)

        self.declare_parameter("mock", True)
        self.declare_parameter("pin_mode", "board")
        self.declare_parameter("driver", "pwm_brake")
        self.declare_parameter("pwm_freq", 1000)
        self.declare_parameter("max_duty", 80)

        self.declare_parameter("left_dir", 12)
        self.declare_parameter("left_pwm", 15)
        self.declare_parameter("right_dir", 22)
        self.declare_parameter("right_pwm", 26)

        self.declare_parameter("left_in1", 12)
        self.declare_parameter("left_in2", 15)
        self.declare_parameter("left_en", 7)
        self.declare_parameter("right_in1", 22)
        self.declare_parameter("right_in2", 26)
        self.declare_parameter("right_en", 11)

    def _motor_config_from_parameters(self) -> dict[str, Any]:
        keys = (
            "mock",
            "pin_mode",
            "driver",
            "pwm_freq",
            "max_duty",
            "left_dir",
            "left_pwm",
            "right_dir",
            "right_pwm",
            "left_in1",
            "left_in2",
            "left_en",
            "right_in1",
            "right_in2",
            "right_en",
        )
        return {key: self.get_parameter(key).value for key in keys}

    def cmd_vel_callback(self, msg: Twist) -> None:
        """Convert a Twist command to left/right track percentages."""
        linear = _clamp(float(msg.linear.x), self.max_linear_speed)
        angular = _clamp(float(msg.angular.z), self.max_angular_speed)

        left_mps = linear - angular * self.wheel_base * 0.5
        right_mps = linear + angular * self.wheel_base * 0.5

        peak = max(abs(left_mps), abs(right_mps))
        if self.max_linear_speed > 0 and peak > self.max_linear_speed:
            scale = self.max_linear_speed / peak
            left_mps *= scale
            right_mps *= scale

        left_percent = 0.0
        right_percent = 0.0
        if self.max_linear_speed > 0:
            left_percent = left_mps / self.max_linear_speed * 100.0
            right_percent = right_mps / self.max_linear_speed * 100.0

        if self.invert_left:
            left_percent *= -1.0
        if self.invert_right:
            right_percent *= -1.0

        left_percent *= self.left_trim
        right_percent *= self.right_trim

        self._drive(left_percent, right_percent)
        self.last_command_time = time.monotonic()
        self._stopped_by_watchdog = False

    def _drive(self, left_percent: float, right_percent: float) -> None:
        try:
            self.motors.drive(left_percent, right_percent)
            state = self.motors.get_state()
            self.last_left_percent = int(state["left"])
            self.last_right_percent = int(state["right"])
        except Exception as exc:  # pragma: no cover - hardware dependent
            self.get_logger().error("Motor output failed: %s", exc)
            self.motors.stop()

    def watchdog_loop(self) -> None:
        """Stop the base if velocity commands stop arriving."""
        if self.command_timeout_sec <= 0:
            return
        if self._stopped_by_watchdog:
            return
        age = time.monotonic() - self.last_command_time
        if age > self.command_timeout_sec:
            self.motors.stop()
            self.last_left_percent = 0
            self.last_right_percent = 0
            self._stopped_by_watchdog = True
            self.get_logger().warning("cmd_vel timeout; motors stopped")

    def publish_status(self) -> None:
        """Publish a compact JSON status string for dashboards/logging."""
        state = self.motors.get_state()
        state.update(
            {
                "left": self.last_left_percent,
                "right": self.last_right_percent,
                "timeout_sec": self.command_timeout_sec,
            }
        )
        msg = String()
        msg.data = json.dumps(state, sort_keys=True)
        self.status_pub.publish(msg)

    def destroy_node(self) -> None:
        self.motors.cleanup()
        super().destroy_node()


def main(args: list[str] | None = None) -> None:
    """Run the track motor driver node."""
    rclpy.init(args=args)
    node = TrackMotorDriver()
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
