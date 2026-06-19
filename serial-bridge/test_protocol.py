from protocol import EventKind, build_angle_command, parse_line


def test_build_angle_command() -> None:
    assert build_angle_command(1, 90.0, 1000) == "#1A90.000T1000"


def test_parse_state() -> None:
    event = parse_line("STATE,90.0/1500,45.0/1000")
    assert event.kind is EventKind.STATE
    assert event.states[0].motor == 1
    assert event.states[0].angle_deg == 90.0
    assert event.states[0].pulse_us == 1500.0


def test_parse_target() -> None:
    event = parse_line(
        "motor1:angle 100.00,pin 7,pulse 1426.00us,duration 1.000s"
    )
    assert event.kind is EventKind.TARGET
    assert event.targets[0].motor == 1
    assert event.targets[0].input_value == 100.0
    assert event.targets[0].pin == 7
    assert event.targets[0].pulse_us == 1426.0


def test_parse_error() -> None:
    event = parse_line("ERR,CHANNEL,모터 번호가 올바르지 않습니다.,INPUT=#99A90")
    assert event.kind is EventKind.ERROR
    assert event.error_code == "CHANNEL"
    assert event.error_input == "#99A90"


def test_parse_state_stream() -> None:
    assert parse_line("STATE_STREAM,ON").kind is EventKind.STATE_STREAM
    assert parse_line("STATE_STREAM,OFF").kind is EventKind.STATE_STREAM


if __name__ == "__main__":
    test_build_angle_command()
    test_parse_state()
    test_parse_target()
    test_parse_error()
    test_parse_state_stream()
    print("protocol tests passed")
