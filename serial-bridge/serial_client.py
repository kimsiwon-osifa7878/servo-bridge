from __future__ import annotations

import queue
import threading
import time
from collections.abc import Callable
from dataclasses import dataclass

from protocol import EventKind, ProtocolEvent, build_angle_command, parse_line


@dataclass(frozen=True)
class ConnectionStatus:
    connected: bool
    ready: bool
    port: str
    message: str


@dataclass(frozen=True)
class QueuedCommand:
    command: str
    require_ready: bool


class SerialClient:
    def __init__(
        self,
        event_callback: Callable[[ProtocolEvent], None],
        status_callback: Callable[[ConnectionStatus], None],
        baud_rate: int = 115200,
        command_timeout: float = 2.0,
    ) -> None:
        self.event_callback = event_callback
        self.status_callback = status_callback
        self.baud_rate = baud_rate
        self.command_timeout = command_timeout

        self._port = ""
        self._serial_port: object | None = None
        self._thread: threading.Thread | None = None
        self._stop_event = threading.Event()
        self._write_lock = threading.Lock()
        self._commands: queue.Queue[QueuedCommand] = queue.Queue(maxsize=64)
        self._connected = False
        self._ready = False

    @property
    def connected(self) -> bool:
        return self._connected

    @property
    def ready(self) -> bool:
        return self._ready

    @property
    def port(self) -> str:
        return self._port

    def connect(self, port: str) -> None:
        self.disconnect()
        self._port = port.strip()
        self._stop_event.clear()
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def disconnect(self) -> None:
        self._stop_event.set()
        if self._thread and self._thread.is_alive():
            self._thread.join(timeout=2.0)
        self._thread = None
        self._close_serial()
        self._publish_status(False, False, "disconnected")
        self._drain_commands()

    def send_angle(self, motor: int, angle_deg: float, duration_ms: int) -> None:
        self.send_command(build_angle_command(motor, angle_deg, duration_ms))

    def send_command(self, command: str) -> None:
        if not self._connected:
            self._publish_status(False, False, "serial port is not connected")
            return
        if not self._ready:
            self._publish_status(True, False, "ESP32 is not READY")
            return
        self.send_probe(command)

    def send_probe(self, command: str) -> None:
        if not self._connected:
            self._publish_status(False, False, "serial port is not connected")
            return
        try:
            self._commands.put_nowait(
                QueuedCommand(command=command, require_ready=False)
            )
        except queue.Full:
            self._publish_status(
                self._connected,
                self._ready,
                "command queue is full",
            )

    def _run(self) -> None:
        try:
            import serial
        except ImportError:
            self._publish_status(False, False, "pyserial is not installed")
            return

        try:
            self._serial_port = serial.Serial(
                self._port,
                self.baud_rate,
                timeout=0.05,
                write_timeout=1.0,
            )
        except Exception as exc:
            self._publish_status(False, False, self._classify_serial_error(exc))
            return

        self._publish_status(True, False, "serial port connected, sending #START")
        self.send_probe("#START")
        self.send_probe("#PINMAP")
        while not self._stop_event.is_set():
            self._write_next_command()
            self._read_next_line()
        self._close_serial()

    def _write_next_command(self) -> None:
        try:
            queued = self._commands.get_nowait()
        except queue.Empty:
            return

        serial_port = self._serial_port
        if serial_port is None:
            return
        if queued.require_ready and not self._ready:
            self._publish_status(True, False, "ESP32 is not READY")
            return

        try:
            with self._write_lock:
                serial_port.write((queued.command + "\r\n").encode("ascii"))
            self.event_callback(ProtocolEvent(EventKind.UNKNOWN, f"> {queued.command}"))
        except Exception as exc:
            self._publish_status(False, False, self._classify_serial_error(exc))
            self._stop_event.set()

    def _read_next_line(self) -> None:
        serial_port = self._serial_port
        if serial_port is None:
            time.sleep(0.05)
            return

        try:
            raw_line = serial_port.readline()
        except Exception as exc:
            self._publish_status(False, False, self._classify_serial_error(exc))
            self._stop_event.set()
            return

        if not raw_line:
            return
        line = raw_line.decode("utf-8", errors="replace").strip()
        event = parse_line(line)
        if event.kind is EventKind.BOOT:
            self._ready = False
            self._publish_status(True, False, "ESP32 booting")
        elif event.kind is EventKind.READY:
            self._ready = True
            self._publish_status(True, True, "ESP32 ready")
        elif event.kind in {
            EventKind.OK,
            EventKind.PINMAP,
            EventKind.STATE,
            EventKind.TARGET,
        } and not self._ready:
            self._ready = True
            self._publish_status(True, True, "ESP32 responded")
        elif event.kind is EventKind.ERROR:
            self._publish_status(True, self._ready, event.raw)
        self.event_callback(event)

    def _close_serial(self) -> None:
        serial_port = self._serial_port
        self._serial_port = None
        self._connected = False
        self._ready = False
        if serial_port is not None:
            try:
                serial_port.close()
            except Exception:
                pass

    def _drain_commands(self) -> None:
        while True:
            try:
                self._commands.get_nowait()
            except queue.Empty:
                return

    def _publish_status(
        self,
        connected: bool,
        ready: bool,
        message: str,
    ) -> None:
        self._connected = connected
        self._ready = ready
        self.status_callback(ConnectionStatus(connected, ready, self._port, message))

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
