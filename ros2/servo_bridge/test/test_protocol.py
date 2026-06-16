from servo_bridge.protocol import EventKind, parse_line


def test_parses_state() -> None:
    event = parse_line("STATE,90.0/1500.00,45.0/1000.00")

    assert event.kind is EventKind.STATE
    assert event.states[0].motor == 1
    assert event.states[1].angle_deg == 45.0
    assert event.states[1].pulse_us == 1000.0


def test_parses_angle_and_raw_targets() -> None:
    angle = parse_line(
        "motor1:angle 100.00,pin 7,pulse 1426.00us,duration 2.000s,delay 0.800s"
    )
    legacy_angle = parse_line(
        "motor1:angle 100.00,pulse 1425.72us,count 292,duration 2.000s"
    )
    raw = parse_line("motor6:pulse 1500.00us,pin 21,duration 1.000s")

    assert angle.kind is EventKind.TARGET
    assert angle.targets[0].count is None
    assert angle.targets[0].pin == 7
    assert angle.targets[0].delay_seconds == 0.8
    assert legacy_angle.targets[0].count == 292
    assert legacy_angle.targets[0].pin is None
    assert raw.targets[0].mode == "pulse"
    assert raw.targets[0].pulse_us == 1500.0
    assert raw.targets[0].pin == 21


def test_parses_pinmap() -> None:
    event = parse_line("PINMAP,motor1:gpio7,motor2:gpio8")

    assert event.kind is EventKind.PINMAP


def test_parses_utf8_error_and_unknown_line() -> None:
    error = parse_line(
        "ERR,CHANNEL,모터 번호가 올바르지 않습니다.,INPUT=#99A90"
    )

    assert error.kind is EventKind.ERROR
    assert error.error_code == "CHANNEL"
    assert error.error_input == "#99A90"
    assert parse_line("unexpected").kind is EventKind.UNKNOWN
