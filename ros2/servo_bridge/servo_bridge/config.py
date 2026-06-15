from pathlib import Path

import yaml

from .models import BridgeConfig, ChannelConfig


def load_bridge_config(path: str | Path) -> BridgeConfig:
    config_path = Path(path)
    with config_path.open("r", encoding="utf-8") as stream:
        document = yaml.safe_load(stream)

    if not isinstance(document, dict) or not isinstance(document.get("channels"), list):
        raise ValueError("config must contain a channels list")

    channels: list[ChannelConfig] = []
    for raw in document["channels"]:
        if not isinstance(raw, dict):
            raise ValueError("each channel must be a mapping")
        channel = ChannelConfig(
            motor=int(raw["motor"]),
            name=str(raw["name"]),
            display_name=str(raw.get("display_name", raw["name"])),
            actuator_type=str(raw.get("type", "joint")),
            min_deg=float(raw["min_deg"]),
            max_deg=float(raw["max_deg"]),
            visible=bool(raw.get("visible", True)),
        )
        if not 1 <= channel.motor <= 15:
            raise ValueError(f"motor must be 1..15: {channel.motor}")
        if not channel.name:
            raise ValueError("channel name cannot be empty")
        if channel.min_deg >= channel.max_deg:
            raise ValueError(f"invalid angle range for {channel.name}")
        channels.append(channel)

    motors = [channel.motor for channel in channels]
    names = [channel.name for channel in channels]
    if len(motors) != len(set(motors)):
        raise ValueError("motor numbers must be unique")
    if len(names) != len(set(names)):
        raise ValueError("channel names must be unique")
    if not channels:
        raise ValueError("at least one channel is required")

    return BridgeConfig(tuple(sorted(channels, key=lambda item: item.motor)))
