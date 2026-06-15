from __future__ import annotations

import queue
import threading
import time
from dataclasses import dataclass
from typing import Callable

from .protocol import EventKind, ProtocolEvent, parse_line


@dataclass(frozen=True)
class ConnectionStatus:
    connected: bool
    ready: bool
    port: str
    message: str


class CommandRequest:
    def __init__(self, command: str, is_motion: bool = False) -> None:
        self.command = command
        self.is_motion = is_motion
        self._event = threading.Event()
        self._callback_lock = threading.Lock()
        self._callbacks: list[Callable[[CommandRequest], None]] = []
        self.accepted = False
        self.response = ""

    def finish(self, accepted: bool, response: str) -> None:
        with self._callback_lock:
            if self._event.is_set():
                return
            self.accepted = accepted
            self.response = response
            self._event.set()
            callbacks = tuple(self._callbacks)
            self._callbacks.clear()
        for callback in callbacks:
            callback(self)

    def wait(self, timeout: float | None = None) -> bool:
        return self._event.wait(timeout)

    def add_done_callback(
        self,
        callback: Callable[[CommandRequest], None],
    ) -> None:
        with self._callback_lock:
            if not self._event.is_set():
                self._callbacks.append(callback)
                return
        callback(self)


class SerialTransport:
    def __init__(
        self,
        port: str,
        baud_rate: int = 115200,
        reconnect_interval: float = 2.0,
        command_timeout: float = 2.0,
        max_motion_rate_hz: float = 20.0,
        event_callback: Callable[[ProtocolEvent], None] | None = None,
        status_callback: Callable[[ConnectionStatus], None] | None = None,
    ) -> None:
        self.port = port
        self.baud_rate = baud_rate
        self.reconnect_interval = reconnect_interval
        self.command_timeout = command_timeout
        self.min_motion_interval = (
            1.0 / max_motion_rate_hz if max_motion_rate_hz > 0.0 else 0.0
        )
        self.event_callback = event_callback
        self.status_callback = status_callback

        self._admin_queue: queue.Queue[CommandRequest] = queue.Queue(maxsize=32)
        self._motion_lock = threading.Lock()
        self._latest_motion: CommandRequest | None = None
        self._stop_event = threading.Event()
        self._thread: threading.Thread | None = None
        self._connected = False
        self._ready = False
        self._last_motion_write = 0.0

    @property
    def connected(self) -> bool:
        return self._connected

    @property
    def ready(self) -> bool:
        return self._ready

    def start(self) -> None:
        if self._thread and self._thread.is_alive():
            return
        self._stop_event.clear()
        self._thread = threading.Thread(
            target=self._run,
            name="servo-bridge-serial",
            daemon=True,
        )
        self._thread.start()

    def stop(self) -> None:
        self._stop_event.set()
        if self._thread:
            self._thread.join(timeout=3.0)
        self._fail_queued("serial transport stopped")

    def send_motion(self, command: str) -> CommandRequest:
        request = CommandRequest(command, is_motion=True)
        with self._motion_lock:
            replaced = self._latest_motion
            self._latest_motion = request
        if replaced:
            replaced.finish(False, "replaced by a newer motion command")
        return request

    def execute(self, command: str, timeout: float | None = None) -> CommandRequest:
        request = CommandRequest(command)
        try:
            self._admin_queue.put_nowait(request)
        except queue.Full:
            request.finish(False, "administrative command queue is full")
            return request
        request.wait(timeout)
        return request

    def _run(self) -> None:
        try:
            import serial
        except ImportError:
            self._publish_status(False, False, "pyserial is not installed")
            self._fail_queued("pyserial is not installed")
            return

        while not self._stop_event.is_set():
            serial_port = None
            try:
                serial_port = serial.Serial(
                    self.port,
                    self.baud_rate,
                    timeout=0.05,
                    write_timeout=1.0,
                )
                self._publish_status(True, False, "serial port connected")
                self._service_connection(serial_port)
            except Exception as exc:
                self._publish_status(False, False, self._classify_serial_error(exc))
                self._fail_queued(str(exc))
            finally:
                if serial_port is not None:
                    try:
                        serial_port.close()
                    except Exception:
                        pass
            self._stop_event.wait(self.reconnect_interval)

    def _service_connection(self, serial_port: object) -> None:
        while not self._stop_event.is_set():
            request = self._next_request()
            if request is not None:
                if not self._ready:
                    request.finish(False, "ESP32 is not READY")
                else:
                    self._write_and_wait(serial_port, request)
                continue

            raw_line = serial_port.readline()
            if raw_line:
                self._handle_line(raw_line)

    def _write_and_wait(self, serial_port: object, request: CommandRequest) -> None:
        if request.is_motion and self.min_motion_interval > 0.0:
            remaining = (
                self._last_motion_write
                + self.min_motion_interval
                - time.monotonic()
            )
            if remaining > 0.0 and self._stop_event.wait(remaining):
                request.finish(False, "serial transport stopped")
                return
        try:
            serial_port.write((request.command + "\r\n").encode("ascii"))
        except Exception as exc:
            request.finish(False, str(exc))
            raise
        if request.is_motion:
            self._last_motion_write = time.monotonic()
        deadline = time.monotonic() + self.command_timeout
        try:
            while not self._stop_event.is_set() and time.monotonic() < deadline:
                raw_line = serial_port.readline()
                if not raw_line:
                    continue
                event = self._handle_line(raw_line)
                if event.kind is EventKind.OK:
                    request.finish(True, event.raw)
                    return
                if event.kind is EventKind.ERROR:
                    request.finish(False, event.raw)
                    return
        except Exception as exc:
            request.finish(False, str(exc))
            raise
        request.finish(False, "command response timeout")

    def _handle_line(self, raw_line: bytes) -> ProtocolEvent:
        line = raw_line.decode("utf-8", errors="replace").strip()
        event = parse_line(line)
        if event.kind is EventKind.BOOT:
            self._ready = False
            self._publish_status(True, False, "ESP32 booting")
        elif event.kind is EventKind.READY:
            self._ready = True
            self._publish_status(True, True, "ESP32 ready")
        if self.event_callback:
            self.event_callback(event)
        return event

    def _next_request(self) -> CommandRequest | None:
        try:
            return self._admin_queue.get_nowait()
        except queue.Empty:
            pass
        with self._motion_lock:
            request = self._latest_motion
            self._latest_motion = None
        return request

    def _publish_status(
        self,
        connected: bool,
        ready: bool,
        message: str,
    ) -> None:
        self._connected = connected
        self._ready = ready
        if self.status_callback:
            self.status_callback(
                ConnectionStatus(connected, ready, self.port, message)
            )

    def _fail_queued(self, reason: str) -> None:
        while True:
            try:
                self._admin_queue.get_nowait().finish(False, reason)
            except queue.Empty:
                break
        with self._motion_lock:
            request = self._latest_motion
            self._latest_motion = None
        if request:
            request.finish(False, reason)

    @staticmethod
    def _classify_serial_error(exc: Exception) -> str:
        text = str(exc)
        lowered = text.lower()
        if "access is denied" in lowered or "permissionerror" in lowered:
            return f"serial port is busy or access was denied: {text}"
        if "permission denied" in lowered:
            return f"serial device permission denied: {text}"
        if "cannot find" in lowered or "no such file" in lowered:
            return f"serial device not found: {text}"
        return f"serial connection error: {text}"
