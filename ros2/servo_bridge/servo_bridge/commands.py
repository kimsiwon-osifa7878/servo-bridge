import math
from collections.abc import Sequence

from .models import BridgeConfig


def trajectory_to_serial_command(
    config: BridgeConfig,
    joint_names: Sequence[str],
    positions_rad: Sequence[float],
    duration_seconds: float,
) -> str:
    if not joint_names:
        raise ValueError("trajectory must contain at least one joint")
    if len(joint_names) != len(positions_rad):
        raise ValueError("joint_names and positions must have the same length")
    if len(joint_names) != len(set(joint_names)):
        raise ValueError("joint names must not be duplicated")
    if not math.isfinite(duration_seconds) or duration_seconds < 0.0:
        raise ValueError("duration must be a finite non-negative value")

    duration_ms = max(100, math.ceil(duration_seconds * 1000.0))
    if duration_ms > 9999:
        raise ValueError("duration exceeds the ESP32 limit of 9999 ms")

    channels = config.by_name()
    selected: list[tuple[int, float]] = []
    for name, position_rad in zip(joint_names, positions_rad, strict=True):
        if name not in channels:
            raise ValueError(f"unknown joint: {name}")
        if not math.isfinite(position_rad):
            raise ValueError(f"position must be finite: {name}")
        angle_deg = math.degrees(position_rad)
        channel = channels[name]
        if angle_deg < channel.min_deg or angle_deg > channel.max_deg:
            raise ValueError(
                f"{name} angle {angle_deg:.3f} is outside "
                f"{channel.min_deg:.3f}..{channel.max_deg:.3f} degrees"
            )
        selected.append((channel.motor, angle_deg))

    selected.sort(key=lambda item: item[0])
    command = "".join(
        f"#{motor}A{angle_deg:.3f}T{duration_ms}"
        for motor, angle_deg in selected
    )
    if len(command.encode("ascii")) > 159:
        raise ValueError("generated command exceeds the ESP32 serial buffer")
    return command
