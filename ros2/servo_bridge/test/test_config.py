import io
from pathlib import Path

import pytest

from servo_bridge.config import load_bridge_config


CONFIG = Path(__file__).parents[1] / "config" / "servos.yaml"


def test_loads_channel_roles_and_limits() -> None:
    config = load_bridge_config(CONFIG)

    assert len(config.channels) == 8
    assert config.by_name()["gripper"].motor == 7
    assert config.by_name()["auxiliary"].actuator_type == "auxiliary"
    assert config.by_name()["motor_6"].min_deg == 70.0


def test_rejects_duplicate_motor(monkeypatch: pytest.MonkeyPatch) -> None:
    document = (
        "channels:\n"
        "  - {motor: 1, name: a, min_deg: 0, max_deg: 180}\n"
        "  - {motor: 1, name: b, min_deg: 0, max_deg: 180}\n"
    )
    monkeypatch.setattr(Path, "open", lambda *args, **kwargs: io.StringIO(document))

    with pytest.raises(ValueError, match="motor numbers"):
        load_bridge_config("unused.yaml")
