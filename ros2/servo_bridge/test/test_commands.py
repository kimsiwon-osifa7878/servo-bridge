import math
from pathlib import Path

import pytest

from servo_bridge.commands import trajectory_to_serial_command
from servo_bridge.config import load_bridge_config


CONFIG = load_bridge_config(
    Path(__file__).parents[1] / "config" / "servos.yaml"
)


def test_builds_motor_ordered_command_and_clamps_short_duration() -> None:
    command = trajectory_to_serial_command(
        CONFIG,
        ["motor_6", "motor_1"],
        [math.radians(90), math.radians(90)],
        0.02,
    )

    assert command == "#1A90.000T100#6A90.000T100"


def test_rejects_out_of_range_angle() -> None:
    with pytest.raises(ValueError, match="outside"):
        trajectory_to_serial_command(
            CONFIG,
            ["motor_6"],
            [math.radians(20)],
            1.0,
        )


def test_rejects_unknown_and_duplicate_names() -> None:
    with pytest.raises(ValueError, match="unknown"):
        trajectory_to_serial_command(CONFIG, ["missing"], [0.0], 1.0)
    with pytest.raises(ValueError, match="duplicated"):
        trajectory_to_serial_command(CONFIG, ["motor_1", "motor_1"], [0.0, 0.0], 1.0)
