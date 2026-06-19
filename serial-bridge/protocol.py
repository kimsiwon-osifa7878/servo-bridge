from __future__ import annotations

import re
from dataclasses import dataclass
from enum import Enum


class EventKind(str, Enum):
    BOOT = "boot"
    READY = "ready"
    OK = "ok"
    ERROR = "error"
    STATE = "state"
    TARGET = "target"
    RESET = "reset"
    DEBUG = "debug"
    TEST = "test"
    OFFSET = "offset"
    PINMAP = "pinmap"
    STATE_STREAM = "state_stream"
    UNKNOWN = "unknown"


@dataclass(frozen=True)
class ServoState:
    motor: int
    angle_deg: float
    pulse_us: float


@dataclass(frozen=True)
class ServoTarget:
    motor: int
    mode: str
    input_value: float
    pulse_us: float
    pin: int | None
    duration_seconds: float
    delay_seconds: float


@dataclass(frozen=True)
class ProtocolEvent:
    kind: EventKind
    raw: str
    states: tuple[ServoState, ...] = ()
    targets: tuple[ServoTarget, ...] = ()
    error_code: str = ""
    error_message: str = ""
    error_input: str = ""


_ERROR_RE = re.compile(r"^ERR,([^,]+),(.*),INPUT=(.*)$")
_RESET_RE = re.compile(r"^RESET motor(\d+) angle:([-+]?\d+(?:\.\d+)?)$")
_ANGLE_TARGET_RE = re.compile(
    r"^motor(\d+):angle ([-+]?\d+(?:\.\d+)?),pin (\d+),pulse "
    r"([-+]?\d+(?:\.\d+)?)us,duration "
    r"([-+]?\d+(?:\.\d+)?)s(?:,delay ([-+]?\d+(?:\.\d+)?)s)?$"
)
_PULSE_TARGET_RE = re.compile(
    r"^motor(\d+):pulse ([-+]?\d+(?:\.\d+)?)us,pin (\d+),duration "
    r"([-+]?\d+(?:\.\d+)?)s(?:,delay ([-+]?\d+(?:\.\d+)?)s)?$"
)


def build_angle_command(motor: int, angle_deg: float, duration_ms: int) -> str:
    if motor < 1:
        raise ValueError("motor must be 1 or greater")
    if duration_ms < 100 or duration_ms > 9999:
        raise ValueError("duration must be 100..9999 ms")
    return f"#{motor}A{angle_deg:.3f}T{duration_ms}"


def parse_line(line: str) -> ProtocolEvent:
    raw = line.strip()
    if raw == "BOOT":
        return ProtocolEvent(EventKind.BOOT, raw)
    if raw == "READY":
        return ProtocolEvent(EventKind.READY, raw)
    if raw == "OK":
        return ProtocolEvent(EventKind.OK, raw)
    if raw.startswith("PWMDBG,"):
        return ProtocolEvent(EventKind.DEBUG, raw)
    if raw.startswith("TEST,"):
        return ProtocolEvent(EventKind.TEST, raw)
    if raw.startswith("angle offset => "):
        return ProtocolEvent(EventKind.OFFSET, raw)
    if raw.startswith("PINMAP,"):
        return ProtocolEvent(EventKind.PINMAP, raw)
    if raw in {"STATE_STREAM,ON", "STATE_STREAM,OFF"}:
        return ProtocolEvent(EventKind.STATE_STREAM, raw)

    error_match = _ERROR_RE.match(raw)
    if error_match:
        return ProtocolEvent(
            EventKind.ERROR,
            raw,
            error_code=error_match.group(1),
            error_message=error_match.group(2),
            error_input=error_match.group(3),
        )

    if _RESET_RE.match(raw):
        return ProtocolEvent(EventKind.RESET, raw)

    if raw.startswith("STATE,"):
        states: list[ServoState] = []
        for motor, value in enumerate(raw[6:].split(","), start=1):
            parts = value.split("/", maxsplit=1)
            if len(parts) != 2:
                return ProtocolEvent(EventKind.UNKNOWN, raw)
            try:
                states.append(ServoState(motor, float(parts[0]), float(parts[1])))
            except ValueError:
                return ProtocolEvent(EventKind.UNKNOWN, raw)
        return ProtocolEvent(EventKind.STATE, raw, states=tuple(states))

    targets: list[ServoTarget] = []
    for section in raw.split("/"):
        angle_match = _ANGLE_TARGET_RE.match(section)
        if angle_match:
            targets.append(
                ServoTarget(
                    motor=int(angle_match.group(1)),
                    mode="angle",
                    input_value=float(angle_match.group(2)),
                    pin=int(angle_match.group(3)),
                    pulse_us=float(angle_match.group(4)),
                    duration_seconds=float(angle_match.group(5)),
                    delay_seconds=float(angle_match.group(6) or 0.0),
                )
            )
            continue

        pulse_match = _PULSE_TARGET_RE.match(section)
        if pulse_match:
            pulse_us = float(pulse_match.group(2))
            targets.append(
                ServoTarget(
                    motor=int(pulse_match.group(1)),
                    mode="pulse",
                    input_value=pulse_us,
                    pin=int(pulse_match.group(3)),
                    pulse_us=pulse_us,
                    duration_seconds=float(pulse_match.group(4)),
                    delay_seconds=float(pulse_match.group(5) or 0.0),
                )
            )
            continue

        targets = []
        break

    if targets:
        return ProtocolEvent(EventKind.TARGET, raw, targets=tuple(targets))
    return ProtocolEvent(EventKind.UNKNOWN, raw)
