from dataclasses import dataclass


@dataclass(frozen=True)
class ChannelConfig:
    motor: int
    name: str
    display_name: str
    actuator_type: str
    min_deg: float
    max_deg: float
    visible: bool = True


@dataclass(frozen=True)
class BridgeConfig:
    channels: tuple[ChannelConfig, ...]

    def by_name(self) -> dict[str, ChannelConfig]:
        return {channel.name: channel for channel in self.channels}

    def by_motor(self) -> dict[int, ChannelConfig]:
        return {channel.motor: channel for channel in self.channels}
