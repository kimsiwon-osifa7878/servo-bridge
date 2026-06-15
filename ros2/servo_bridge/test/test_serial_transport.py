from servo_bridge.serial_transport import CommandRequest


def test_command_request_calls_callback_before_and_after_completion() -> None:
    first: list[tuple[bool, str]] = []
    request = CommandRequest("#RESET")
    request.add_done_callback(
        lambda result: first.append((result.accepted, result.response))
    )

    request.finish(True, "OK")

    second: list[tuple[bool, str]] = []
    request.add_done_callback(
        lambda result: second.append((result.accepted, result.response))
    )
    assert first == [(True, "OK")]
    assert second == [(True, "OK")]
