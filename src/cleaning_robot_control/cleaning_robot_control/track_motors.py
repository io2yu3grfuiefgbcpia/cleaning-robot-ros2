"""GPIO-backed tracked motor helpers for Orange Pi style boards."""

from __future__ import annotations

import logging
import os
import threading
import time
from dataclasses import dataclass
from typing import Any

LOGGER = logging.getLogger(__name__)

# Orange Pi Zero H2+ 26-pin physical pin number to SUNXI GPIO number.
OPI_ZERO_BOARD_TO_SUNXI = {
    7: 6,
    11: 1,
    12: 7,
    15: 3,
    22: 2,
    26: 10,
}


def _cfg_int(config: dict[str, Any], key: str, default: int) -> int:
    value = config.get(key)
    if value is None:
        return default
    return int(value)


@dataclass(frozen=True)
class MotorPins:
    """Pins used by one side of the tracked drive."""

    in1: int
    in2: int
    en: int


class SysfsPWM:
    """Software PWM fallback for boards without a usable GPIO PWM API."""

    def __init__(self, backend: "SysfsGpioBackend", pin: int, freq: int) -> None:
        self._backend = backend
        self._pin = pin
        self._freq = max(int(freq), 1)
        self._duty = 0
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def _run(self) -> None:
        while not self._stop.is_set():
            duty = self._duty
            if duty <= 0:
                self._backend.output(self._pin, 0)
                time.sleep(0.05)
                continue

            period = 1.0 / float(self._freq)
            on_time = period * float(duty) / 100.0
            off_time = max(period - on_time, 0.0001)
            self._backend.output(self._pin, 1)
            time.sleep(on_time)
            self._backend.output(self._pin, 0)
            time.sleep(off_time)

    def start(self, duty: int) -> None:
        """Start PWM with the given duty cycle."""
        self._duty = int(duty)

    def ChangeDutyCycle(self, duty: int) -> None:  # noqa: N802
        """Match the OPi.GPIO/RPi.GPIO PWM method name."""
        self._duty = int(duty)

    def stop(self) -> None:
        """Stop PWM and drive the line low."""
        self._stop.set()
        try:
            self._backend.output(self._pin, 0)
        except OSError:
            pass


class SysfsGpioBackend:
    """Small /sys/class/gpio backend used when OPi.GPIO is unavailable."""

    HIGH = 1
    LOW = 0
    OUT = "out"
    BOARD = "board"
    SUNXI = "sunxi"
    PWM = SysfsPWM

    def __init__(self, pin_mode: str) -> None:
        self.pin_mode = pin_mode
        self._exported: set[int] = set()

    def setwarnings(self, _flag: bool) -> None:
        """Compatibility shim for GPIO libraries."""

    def setmode(self, _mode: Any) -> None:
        """Compatibility shim for GPIO libraries."""

    def _sunxi_pin(self, pin: int) -> int:
        if self.pin_mode == "board":
            return OPI_ZERO_BOARD_TO_SUNXI.get(int(pin), int(pin))
        return int(pin)

    def setup(self, pin: int, _mode: Any) -> None:
        """Export and configure a GPIO as output."""
        gpio = self._sunxi_pin(pin)
        if gpio in self._exported:
            return

        export_path = "/sys/class/gpio/export"
        if os.path.exists(export_path):
            try:
                with open(export_path, "w", encoding="utf-8") as file_obj:
                    file_obj.write(str(gpio))
            except OSError:
                pass

        self._exported.add(gpio)
        direction_path = f"/sys/class/gpio/gpio{gpio}/direction"
        with open(direction_path, "w", encoding="utf-8") as file_obj:
            file_obj.write("out")

    def output(self, pin: int, value: int) -> None:
        """Write a GPIO value."""
        gpio = self._sunxi_pin(pin)
        value_path = f"/sys/class/gpio/gpio{gpio}/value"
        with open(value_path, "w", encoding="utf-8") as file_obj:
            file_obj.write("1" if value else "0")

    def cleanup(self) -> None:
        """Unexport all GPIOs opened by this backend."""
        for gpio in list(self._exported):
            unexport_path = "/sys/class/gpio/unexport"
            if os.path.exists(unexport_path):
                try:
                    with open(unexport_path, "w", encoding="utf-8") as file_obj:
                        file_obj.write(str(gpio))
                except OSError:
                    pass
        self._exported.clear()


class TrackMotors:
    """Dual tracked motor controller with Orange Pi friendly GPIO fallback."""

    def __init__(self, config: dict[str, Any]) -> None:
        self.mock = True if config.get("mock") is None else bool(config["mock"])
        self.pin_mode = str(config.get("pin_mode") or "board").lower()
        self.max_duty = _cfg_int(config, "max_duty", 100)
        self.pwm_freq = _cfg_int(config, "pwm_freq", 1000)

        raw_driver = str(config.get("driver") or "l298n").lower()
        if raw_driver in ("pwm_brake", "ta6586", "zk5ad", "zk-5ad"):
            self.driver = "pwm_brake"
        else:
            self.driver = "l298n"

        if self.driver == "pwm_brake":
            self.left_pins = MotorPins(
                _cfg_int(config, "left_dir", _cfg_int(config, "left_in1", 12)),
                _cfg_int(config, "left_pwm", _cfg_int(config, "left_in2", 15)),
                -1,
            )
            self.right_pins = MotorPins(
                _cfg_int(config, "right_dir", _cfg_int(config, "right_in1", 22)),
                _cfg_int(config, "right_pwm", _cfg_int(config, "right_in2", 26)),
                -1,
            )
        else:
            self.left_pins = MotorPins(
                _cfg_int(config, "left_in1", 12),
                _cfg_int(config, "left_in2", 15),
                _cfg_int(config, "left_en", 7),
            )
            self.right_pins = MotorPins(
                _cfg_int(config, "right_in1", 22),
                _cfg_int(config, "right_in2", 26),
                _cfg_int(config, "right_en", 11),
            )

        self._gpio: Any = None
        self._gpio_name = "mock"
        self._pwms: dict[int, Any] = {}
        self._braked = {
            self.left_pins.in2: True,
            self.right_pins.in2: True,
        }
        self._left = 0
        self._right = 0

        if not self.mock:
            self._init_gpio()

    @staticmethod
    def _load_gpio_module() -> tuple[Any | None, str]:
        errors: list[str] = []
        try:
            from OPi import GPIO as opi_gpio  # type: ignore

            if hasattr(opi_gpio, "setmode"):
                return opi_gpio, "OPi.GPIO"
            errors.append("OPi.GPIO: no setmode")
        except Exception as exc:  # pragma: no cover - hardware dependent
            errors.append(f"OPi.GPIO: {exc}")

        try:
            import OPi.GPIO as opi_gpio_alt  # type: ignore

            if hasattr(opi_gpio_alt, "setmode"):
                return opi_gpio_alt, "OPi.GPIO"
            errors.append("import OPi.GPIO: no setmode")
        except Exception as exc:  # pragma: no cover - hardware dependent
            errors.append(f"import OPi.GPIO: {exc}")

        try:
            import RPi.GPIO as rpi_gpio  # type: ignore

            if hasattr(rpi_gpio, "setmode"):
                return rpi_gpio, "RPi.GPIO"
            errors.append("RPi.GPIO: no setmode")
        except Exception as exc:  # pragma: no cover - hardware dependent
            errors.append(f"RPi.GPIO: {exc}")

        LOGGER.warning("GPIO library unavailable (%s); trying sysfs",
                       "; ".join(errors))
        return None, ""

    def _init_gpio(self) -> None:
        gpio, gpio_name = self._load_gpio_module()
        if gpio is None:
            gpio = SysfsGpioBackend(self.pin_mode)
            gpio_name = "sysfs"

        self._gpio = gpio
        self._gpio_name = gpio_name
        LOGGER.info("Using GPIO backend: %s", gpio_name)

        if gpio_name != "sysfs":
            if self.pin_mode == "board" and hasattr(gpio, "BOARD"):
                gpio.setmode(gpio.BOARD)
            elif self.pin_mode == "sunxi" and hasattr(gpio, "SUNXI"):
                gpio.setmode(gpio.SUNXI)
            elif self.pin_mode == "bcm" and hasattr(gpio, "BCM"):
                gpio.setmode(gpio.BCM)
            else:
                gpio.setmode(gpio.BOARD)
            gpio.setwarnings(False)

        if self.driver == "pwm_brake":
            LOGGER.info(
                "Motor driver pwm_brake: left dir=%s pwm=%s right dir=%s pwm=%s",
                self.left_pins.in1,
                self.left_pins.in2,
                self.right_pins.in1,
                self.right_pins.in2,
            )
            for pins in (self.left_pins, self.right_pins):
                gpio.setup(pins.in1, gpio.OUT)
                gpio.setup(pins.in2, gpio.OUT)
                self._apply_brake(pins)
        else:
            LOGGER.info("Motor driver l298n")
            for pins in (self.left_pins, self.right_pins):
                gpio.setup(pins.in1, gpio.OUT)
                gpio.setup(pins.in2, gpio.OUT)
                gpio.setup(pins.en, gpio.OUT)
                pwm = gpio.PWM(pins.en, self.pwm_freq)
                pwm.start(0)
                self._pwms[pins.en] = pwm

    @staticmethod
    def _clamp(value: float) -> int:
        return max(-100, min(100, int(value)))

    def _stop_pwm_line(self, pwm_pin: int) -> None:
        if pwm_pin in self._pwms:
            self._pwms[pwm_pin].stop()
            del self._pwms[pwm_pin]

    def _start_pwm_line(self, pins: MotorPins) -> None:
        gpio = self._gpio
        pwm_pin = pins.in2
        if pwm_pin not in self._pwms:
            gpio.setup(pwm_pin, gpio.OUT)
            pwm = gpio.PWM(pwm_pin, self.pwm_freq)
            pwm.start(0)
            self._pwms[pwm_pin] = pwm
        self._braked[pwm_pin] = False

    def _apply_brake(self, pins: MotorPins) -> None:
        if self.mock:
            return
        gpio = self._gpio
        self._stop_pwm_line(pins.in2)
        gpio.setup(pins.in1, gpio.OUT)
        gpio.setup(pins.in2, gpio.OUT)
        gpio.output(pins.in1, gpio.HIGH)
        gpio.output(pins.in2, gpio.HIGH)
        self._braked[pins.in2] = True

    def _set_side_l298n(self, pins: MotorPins, speed: int) -> None:
        speed = self._clamp(speed)
        duty = abs(speed) * self.max_duty // 100
        if self.mock:
            return

        gpio = self._gpio
        if speed > 0:
            gpio.output(pins.in1, gpio.HIGH)
            gpio.output(pins.in2, gpio.LOW)
        elif speed < 0:
            gpio.output(pins.in1, gpio.LOW)
            gpio.output(pins.in2, gpio.HIGH)
        else:
            gpio.output(pins.in1, gpio.LOW)
            gpio.output(pins.in2, gpio.LOW)
        self._pwms[pins.en].ChangeDutyCycle(duty)

    def _set_side_pwm_brake(self, pins: MotorPins, speed: int) -> None:
        speed = self._clamp(speed)
        if self.mock:
            return
        if speed == 0:
            self._apply_brake(pins)
            return

        duty = abs(speed) * self.max_duty // 100
        if duty < 1:
            self._apply_brake(pins)
            return

        gpio = self._gpio
        if self._braked.get(pins.in2, True):
            self._start_pwm_line(pins)
        gpio.output(pins.in1, gpio.LOW if speed > 0 else gpio.HIGH)
        self._pwms[pins.in2].ChangeDutyCycle(duty)

    def _set_side(self, pins: MotorPins, speed: int) -> None:
        if self.driver == "pwm_brake":
            self._set_side_pwm_brake(pins, speed)
        else:
            self._set_side_l298n(pins, speed)

    def drive(self, left: float, right: float) -> None:
        """Drive left and right tracks using -100..100 speed percentages."""
        self._left = self._clamp(left)
        self._right = self._clamp(right)
        self._set_side(self.left_pins, self._left)
        self._set_side(self.right_pins, self._right)

    def stop(self) -> None:
        """Stop both tracks."""
        self.drive(0, 0)

    def get_state(self) -> dict[str, Any]:
        """Return current motor command and backend state."""
        return {
            "left": self._left,
            "right": self._right,
            "mock": self.mock,
            "driver": self.driver,
            "gpio_backend": self._gpio_name,
            "pin_mode": self.pin_mode,
        }

    def cleanup(self) -> None:
        """Stop motors and release GPIO resources."""
        self.stop()
        if self.mock or self._gpio is None:
            return

        for pwm in list(self._pwms.values()):
            try:
                pwm.stop()
            except Exception:  # pragma: no cover - hardware dependent
                pass
        self._pwms.clear()

        try:
            self._gpio.cleanup()
        except Exception:  # pragma: no cover - hardware dependent
            pass
