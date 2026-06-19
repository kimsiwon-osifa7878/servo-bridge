from __future__ import annotations

import queue
import sys
from dataclasses import dataclass

from protocol import EventKind, ProtocolEvent
from serial_client import ConnectionStatus, SerialClient


@dataclass(frozen=True)
class MotorConfig:
    motor: int
    display_name: str
    min_deg: float
    max_deg: float
    initial_deg: float


MOTORS = (
    MotorConfig(1, "Motor 1", 0.0, 180.0, 90.0),
    MotorConfig(2, "Motor 2", 0.0, 180.0, 90.0),
    MotorConfig(3, "Motor 3", 0.0, 180.0, 90.0),
    MotorConfig(4, "Motor 4", 0.0, 180.0, 90.0),
    MotorConfig(5, "Motor 5", 0.0, 180.0, 90.0),
    MotorConfig(6, "Motor 6", 70.0, 150.0, 90.0),
)


@dataclass
class MotorWidgets:
    config: MotorConfig
    current_angle: object
    current_pulse: object
    target_angle: object
    target_pulse: object
    slider: object
    spinbox: object
    send_button: object


def main() -> None:
    try:
        from PySide6.QtCore import Qt, QTimer
        from PySide6.QtWidgets import (
            QApplication,
            QDoubleSpinBox,
            QFrame,
            QGridLayout,
            QGroupBox,
            QHBoxLayout,
            QLabel,
            QLineEdit,
            QMainWindow,
            QPushButton,
            QSlider,
            QSpinBox,
            QTextEdit,
            QVBoxLayout,
            QWidget,
        )
    except ImportError as exc:
        raise SystemExit(
            "PySide6 is required. Install with: pip install -r requirements.txt"
        ) from exc

    app = QApplication(sys.argv)
    events: queue.Queue[tuple[str, object]] = queue.Queue()

    def on_event(event: ProtocolEvent) -> None:
        events.put(("event", event))

    def on_status(status: ConnectionStatus) -> None:
        events.put(("status", status))

    client = SerialClient(on_event, on_status)

    class MainWindow(QMainWindow):
        def __init__(self) -> None:
            super().__init__()
            self.setWindowTitle("Serial Servo Bridge")
            self.resize(980, 720)
            self.rows: dict[int, MotorWidgets] = {}
            self.command_controls: list[object] = []

            central = QWidget()
            root = QVBoxLayout(central)

            connection = QHBoxLayout()
            connection.addWidget(QLabel("Port"))
            self.port_input = QLineEdit("COM3")
            self.port_input.setMaximumWidth(120)
            connection.addWidget(self.port_input)
            self.connect_button = QPushButton("Connect")
            self.connect_button.clicked.connect(self._toggle_connection)
            connection.addWidget(self.connect_button)
            connection.addWidget(QLabel("Duration ms"))
            self.duration_input = QSpinBox()
            self.duration_input.setRange(100, 9999)
            self.duration_input.setValue(1000)
            connection.addWidget(self.duration_input)
            connection.addStretch()
            root.addLayout(connection)

            self.status_label = QLabel("Disconnected")
            self.status_label.setTextInteractionFlags(
                Qt.TextInteractionFlag.TextSelectableByMouse
            )
            root.addWidget(self.status_label)

            grid = QGridLayout()
            for index, config in enumerate(MOTORS):
                card = self._create_motor_card(config)
                grid.addWidget(card, index // 3, index % 3)
            root.addLayout(grid)

            root.addWidget(self._create_command_panel())

            manual = QHBoxLayout()
            self.manual_command = QLineEdit()
            self.manual_command.setPlaceholderText("#RESET, #OFFSET, #1P1500T1000 ...")
            self.manual_command.returnPressed.connect(self._send_manual_command)
            manual.addWidget(self.manual_command)
            self.manual_send_button = QPushButton("Send Serial")
            self.manual_send_button.clicked.connect(self._send_manual_command)
            manual.addWidget(self.manual_send_button)
            root.addLayout(manual)

            self.log = QTextEdit()
            self.log.setReadOnly(True)
            self.log.setMinimumHeight(180)
            root.addWidget(self.log)

            note = QLabel(
                "Current values are ESP32 open-loop calculations, not sensor feedback. "
                "OK means accepted, not motion complete."
            )
            root.addWidget(note)
            self.setCentralWidget(central)

            timer = QTimer(self)
            timer.timeout.connect(self._drain_events)
            timer.start(50)

            self._set_controls_enabled(False)
            self._set_command_controls_enabled(False)

        def _create_command_panel(self) -> object:
            panel = QGroupBox("Serial Commands")
            layout = QGridLayout(panel)

            self._add_command_button(layout, 0, 0, "START", "#START")
            self._add_command_button(layout, 0, 1, "Reset", "#RESET")
            self._add_command_button(layout, 0, 2, "Reset Boot", "#RESET_BOOT")
            self._add_command_button(layout, 0, 3, "Pin Map", "#PINMAP")
            self._add_command_button(layout, 0, 4, "State Once", "#STATE")
            self._add_command_button(layout, 0, 5, "State On", "#STATE ON")
            self._add_command_button(layout, 0, 6, "State Off", "#STATE OFF")

            self._add_command_button(layout, 1, 0, "Test Toggle", "#TEST")
            self._add_command_button(layout, 1, 1, "Test On", "#TEST ON")
            self._add_command_button(layout, 1, 2, "Test Off", "#TEST OFF")

            layout.addWidget(QLabel("Test Motor"), 1, 3)
            self.test_motor = QSpinBox()
            self.test_motor.setRange(1, 6)
            self.test_motor.setValue(1)
            layout.addWidget(self.test_motor, 1, 4)
            test_motor_button = QPushButton("Send")
            test_motor_button.clicked.connect(
                lambda: self._send_serial(f"#TEST{self.test_motor.value()}")
            )
            layout.addWidget(test_motor_button, 1, 5)

            layout.addWidget(QLabel("Go Pos"), 2, 0)
            self.gopos_index = QSpinBox()
            self.gopos_index.setRange(1, 2)
            self.gopos_index.setValue(1)
            layout.addWidget(self.gopos_index, 2, 1)
            layout.addWidget(QLabel("Duration ms"), 2, 2)
            self.gopos_duration = QSpinBox()
            self.gopos_duration.setRange(100, 9999)
            self.gopos_duration.setValue(1500)
            layout.addWidget(self.gopos_duration, 2, 3)
            gopos_button = QPushButton("Send GOPOS")
            gopos_button.clicked.connect(
                lambda: self._send_serial(
                    f"#GOPOS{self.gopos_index.value()}T{self.gopos_duration.value()}"
                )
            )
            layout.addWidget(gopos_button, 2, 4)

            layout.addWidget(QLabel("Raw PWM Motor"), 3, 0)
            self.raw_motor = QSpinBox()
            self.raw_motor.setRange(1, 6)
            self.raw_motor.setValue(1)
            layout.addWidget(self.raw_motor, 3, 1)
            layout.addWidget(QLabel("Pulse us"), 3, 2)
            self.raw_pulse = QSpinBox()
            self.raw_pulse.setRange(500, 2500)
            self.raw_pulse.setValue(1500)
            layout.addWidget(self.raw_pulse, 3, 3)
            raw_button = QPushButton("Send PWM")
            raw_button.clicked.connect(
                lambda: self._send_serial(
                    f"#{self.raw_motor.value()}P{self.raw_pulse.value()}"
                    f"T{self.duration_input.value()}"
                )
            )
            layout.addWidget(raw_button, 3, 4)

            self._add_command_button(layout, 4, 0, "Offset Query", "#OFFSET")
            self._add_command_button(layout, 4, 1, "Offset Reset", "#OFFSET#RESET")
            layout.addWidget(QLabel("Offset Motor"), 4, 2)
            self.offset_motor = QSpinBox()
            self.offset_motor.setRange(1, 6)
            self.offset_motor.setValue(1)
            layout.addWidget(self.offset_motor, 4, 3)
            self.offset_value = QDoubleSpinBox()
            self.offset_value.setRange(-180.0, 180.0)
            self.offset_value.setDecimals(2)
            self.offset_value.setSingleStep(0.1)
            self.offset_value.setValue(0.0)
            layout.addWidget(self.offset_value, 4, 4)
            offset_button = QPushButton("Set Offset")
            offset_button.clicked.connect(
                lambda: self._send_serial(
                    f"#OFFSET#{self.offset_motor.value()}A"
                    f"{self.offset_value.value():.2f}"
                )
            )
            layout.addWidget(offset_button, 4, 5)

            for widget in (
                self.test_motor,
                test_motor_button,
                self.gopos_index,
                self.gopos_duration,
                gopos_button,
                self.raw_motor,
                self.raw_pulse,
                raw_button,
                self.offset_motor,
                self.offset_value,
                offset_button,
            ):
                self.command_controls.append(widget)
            return panel

        def _add_command_button(
            self,
            layout: object,
            row: int,
            column: int,
            label: str,
            command: str,
        ) -> None:
            button = QPushButton(label)
            button.clicked.connect(lambda checked=False, value=command: self._send_serial(value))
            layout.addWidget(button, row, column)
            self.command_controls.append(button)

        def closeEvent(self, event: object) -> None:
            client.disconnect()
            super().closeEvent(event)

        def _create_motor_card(self, config: MotorConfig) -> object:
            card = QFrame()
            card.setFrameShape(QFrame.Shape.StyledPanel)
            layout = QVBoxLayout(card)

            title = QLabel(config.display_name)
            title.setStyleSheet("font-weight: 600;")
            layout.addWidget(title)

            current_angle = QLabel("-")
            current_pulse = QLabel("-")
            target_angle = QLabel("-")
            target_pulse = QLabel("-")
            for label, value in (
                ("Current deg", current_angle),
                ("Current us", current_pulse),
                ("Target deg", target_angle),
                ("Target us", target_pulse),
            ):
                row = QHBoxLayout()
                row.addWidget(QLabel(label))
                row.addStretch()
                row.addWidget(value)
                layout.addLayout(row)

            spinbox = QDoubleSpinBox()
            spinbox.setRange(config.min_deg, config.max_deg)
            spinbox.setDecimals(2)
            spinbox.setSingleStep(1.0)
            spinbox.setValue(config.initial_deg)

            slider = QSlider(Qt.Orientation.Horizontal)
            slider.setRange(round(config.min_deg * 10), round(config.max_deg * 10))
            slider.setValue(round(config.initial_deg * 10))
            slider.valueChanged.connect(
                lambda value, control=spinbox: control.setValue(value / 10.0)
            )
            spinbox.valueChanged.connect(
                lambda value, control=slider: control.setValue(round(value * 10))
            )
            slider.sliderReleased.connect(
                lambda motor=config.motor: self._send_motor(motor)
            )
            spinbox.editingFinished.connect(
                lambda motor=config.motor: self._send_motor(motor)
            )

            layout.addWidget(slider)
            layout.addWidget(spinbox)

            send_button = QPushButton("Send")
            send_button.clicked.connect(lambda checked=False, motor=config.motor: self._send_motor(motor))
            layout.addWidget(send_button)

            self.rows[config.motor] = MotorWidgets(
                config,
                current_angle,
                current_pulse,
                target_angle,
                target_pulse,
                slider,
                spinbox,
                send_button,
            )
            return card

        def _toggle_connection(self) -> None:
            if client.connected:
                client.disconnect()
                return
            self._append_log(f"Connecting to {self.port_input.text().strip()}...")
            client.connect(self.port_input.text())

        def _send_motor(self, motor: int) -> None:
            row = self.rows[motor]
            angle = float(row.spinbox.value())
            duration_ms = int(self.duration_input.value())
            client.send_angle(motor, angle, duration_ms)

        def _send_serial(self, command: str) -> None:
            client.send_probe(command.strip())

        def _send_manual_command(self) -> None:
            command = self.manual_command.text().strip()
            if not command:
                return
            self._send_serial(command)
            self.manual_command.clear()

        def _drain_events(self) -> None:
            while True:
                try:
                    kind, payload = events.get_nowait()
                except queue.Empty:
                    return
                if kind == "status":
                    self._handle_status(payload)
                elif kind == "event":
                    self._handle_event(payload)

        def _handle_status(self, status: ConnectionStatus) -> None:
            self.connect_button.setText("Disconnect" if status.connected else "Connect")
            self.port_input.setEnabled(not status.connected)
            self._set_controls_enabled(status.connected and status.ready)
            self._set_command_controls_enabled(status.connected)
            self.status_label.setText(
                f"Port {status.port or '-'} | connected={status.connected} | "
                f"ready={status.ready} | {status.message}"
            )

        def _handle_event(self, event: ProtocolEvent) -> None:
            self._append_log(event.raw)
            if event.kind is EventKind.STATE:
                for state in event.states:
                    row = self.rows.get(state.motor)
                    if row is None:
                        continue
                    row.current_angle.setText(f"{state.angle_deg:.2f}")
                    row.current_pulse.setText(f"{state.pulse_us:.0f}")
            elif event.kind is EventKind.TARGET:
                for target in event.targets:
                    row = self.rows.get(target.motor)
                    if row is None:
                        continue
                    if target.mode == "angle":
                        row.target_angle.setText(f"{target.input_value:.2f}")
                    row.target_pulse.setText(f"{target.pulse_us:.2f}")

        def _append_log(self, text: str) -> None:
            self.log.append(text)

        def _set_controls_enabled(self, enabled: bool) -> None:
            for row in self.rows.values():
                row.slider.setEnabled(enabled)
                row.spinbox.setEnabled(enabled)
                row.send_button.setEnabled(enabled)

        def _set_command_controls_enabled(self, enabled: bool) -> None:
            for control in self.command_controls:
                control.setEnabled(enabled)
            self.manual_command.setEnabled(enabled)
            self.manual_send_button.setEnabled(enabled)

    window = MainWindow()
    window.show()
    raise SystemExit(app.exec())


if __name__ == "__main__":
    main()
