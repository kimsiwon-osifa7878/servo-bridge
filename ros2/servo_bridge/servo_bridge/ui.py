from __future__ import annotations

import json
import math
import os
import queue
import subprocess
import sys
import threading
from dataclasses import dataclass
from pathlib import Path

import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray, String
from std_srvs.srv import Trigger
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from .config import load_bridge_config
from .models import ChannelConfig


def _relaunch_windows_ui_environment() -> bool:
    if sys.platform != "win32":
        return False
    default_python = Path(r"C:\pixi_ws\ui-venv\Scripts\python.exe")
    ui_python = Path(os.environ.get("SERVO_BRIDGE_UI_PYTHON", default_python))
    if not ui_python.exists():
        return False
    if Path(sys.executable).resolve() == ui_python.resolve():
        return False

    environment = os.environ.copy()
    pyside_directory = ui_python.parents[1] / "Lib" / "site-packages" / "PySide6"
    required_paths = [
        str(pyside_directory),
        r"C:\pixi_ws\ros2-windows\bin",
        r"C:\pixi_ws\ros2-windows\Scripts",
        r"C:\pixi_ws\.pixi\envs\default\Library\bin",
    ]
    environment["PATH"] = os.pathsep.join(
        required_paths + [environment.get("PATH", "")]
    )
    environment["QT_QPA_PLATFORM_PLUGIN_PATH"] = str(
        pyside_directory / "plugins" / "platforms"
    )
    result = subprocess.run(
        [
            str(ui_python),
            "-c",
            "from servo_bridge.ui import main; main()",
            *sys.argv[1:],
        ],
        env=environment,
        check=False,
    )
    raise SystemExit(result.returncode)


@dataclass
class RowWidgets:
    channel: ChannelConfig
    selected: object
    current_angle: object
    current_pulse: object
    target_angle: object
    target_pulse: object
    command_angle: object
    slider: object
    moving: object


class UiRosNode(Node):
    def __init__(self, events: queue.Queue) -> None:
        super().__init__("servo_bridge_ui")
        self.events = events
        self.trajectory_pub = self.create_publisher(
            JointTrajectory, "/servo_bridge/joint_trajectory", 10
        )
        self.reset_client = self.create_client(Trigger, "/servo_bridge/reset")
        self.create_subscription(
            JointState,
            "/servo_bridge/joint_states",
            lambda message: self.events.put(("current_angles", message)),
            10,
        )
        self.create_subscription(
            JointState,
            "/servo_bridge/target_joint_states",
            lambda message: self.events.put(("target_angles", message)),
            10,
        )
        self.create_subscription(
            Float64MultiArray,
            "/servo_bridge/current_pulse_us",
            lambda message: self.events.put(("current_pulses", message)),
            10,
        )
        self.create_subscription(
            Float64MultiArray,
            "/servo_bridge/target_pulse_us",
            lambda message: self.events.put(("target_pulses", message)),
            10,
        )
        self.create_subscription(
            String,
            "/servo_bridge/status",
            lambda message: self.events.put(("status", message.data)),
            10,
        )
        self.create_subscription(
            String,
            "/servo_bridge/serial_events",
            lambda message: self.events.put(("serial_event", message.data)),
            20,
        )

    def send_trajectory(
        self,
        names: list[str],
        positions_rad: list[float],
        duration_ms: int,
    ) -> None:
        message = JointTrajectory()
        message.joint_names = names
        point = JointTrajectoryPoint()
        point.positions = positions_rad
        point.time_from_start.sec = duration_ms // 1000
        point.time_from_start.nanosec = (duration_ms % 1000) * 1_000_000
        message.points = [point]
        self.trajectory_pub.publish(message)

    def reset(self) -> None:
        if not self.reset_client.service_is_ready():
            self.events.put(("reset_result", "Reset service is not available"))
            return
        future = self.reset_client.call_async(Trigger.Request())
        future.add_done_callback(self._reset_done)

    def _reset_done(self, future: object) -> None:
        try:
            result = future.result()
            message = (
                result.message
                if result.success
                else f"Reset failed: {result.message}"
            )
        except Exception as exc:
            message = f"Reset failed: {exc}"
        self.events.put(("reset_result", message))


def _run_ros(node: UiRosNode, stop_event: threading.Event) -> None:
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    while rclpy.ok() and not stop_event.is_set():
        executor.spin_once(timeout_sec=0.1)
    executor.remove_node(node)


def main(args: list[str] | None = None) -> None:
    _relaunch_windows_ui_environment()
    try:
        from PySide6.QtCore import Qt, QTimer
        from PySide6.QtWidgets import (
            QApplication,
            QCheckBox,
            QDoubleSpinBox,
            QHBoxLayout,
            QLabel,
            QMainWindow,
            QPushButton,
            QSlider,
            QSpinBox,
            QTableWidget,
            QTableWidgetItem,
            QVBoxLayout,
            QWidget,
        )
    except ImportError as exc:
        raise SystemExit("PySide6 is required: pip install PySide6") from exc

    rclpy.init(args=args)
    events: queue.Queue = queue.Queue()
    node = UiRosNode(events)
    stop_event = threading.Event()
    ros_thread = threading.Thread(
        target=_run_ros,
        args=(node, stop_event),
        daemon=True,
    )
    ros_thread.start()

    config_path = (
        Path(get_package_share_directory("servo_bridge"))
        / "config"
        / "servos.yaml"
    )
    config = load_bridge_config(config_path)
    app = QApplication(sys.argv)

    class MainWindow(QMainWindow):
        def __init__(self) -> None:
            super().__init__()
            self.setWindowTitle("ROS2 Servo Bridge")
            self.resize(1250, 520)
            self.rows: dict[str, RowWidgets] = {}
            self.current_angles: dict[str, float] = {}
            self.target_angles: dict[str, float] = {}

            central = QWidget()
            layout = QVBoxLayout(central)
            self.status_label = QLabel("ROS2 bridge status unavailable")
            self.status_label.setTextInteractionFlags(
                Qt.TextInteractionFlag.TextSelectableByMouse
            )
            self.event_label = QLabel("Last event: -")
            layout.addWidget(self.status_label)
            layout.addWidget(self.event_label)

            table = QTableWidget(len(config.channels), 11)
            table.setHorizontalHeaderLabels(
                [
                    "Use",
                    "Motor",
                    "Name",
                    "Type",
                    "Current deg",
                    "Current us",
                    "Target deg",
                    "Target us",
                    "Command deg",
                    "Slider",
                    "Moving",
                ]
            )
            for row, channel in enumerate(config.channels):
                selected = QCheckBox()
                selected.setChecked(channel.visible)
                table.setCellWidget(row, 0, selected)
                table.setItem(row, 1, QTableWidgetItem(str(channel.motor)))
                table.setItem(row, 2, QTableWidgetItem(channel.display_name))
                table.setItem(row, 3, QTableWidgetItem(channel.actuator_type))
                current_angle = QTableWidgetItem("-")
                current_pulse = QTableWidgetItem("-")
                target_angle = QTableWidgetItem("-")
                target_pulse = QTableWidgetItem("-")
                moving = QTableWidgetItem("-")
                table.setItem(row, 4, current_angle)
                table.setItem(row, 5, current_pulse)
                table.setItem(row, 6, target_angle)
                table.setItem(row, 7, target_pulse)
                table.setItem(row, 10, moving)

                command_angle = QDoubleSpinBox()
                command_angle.setRange(channel.min_deg, channel.max_deg)
                command_angle.setDecimals(2)
                command_angle.setValue(
                    min(max(90.0, channel.min_deg), channel.max_deg)
                )
                slider = QSlider(Qt.Orientation.Horizontal)
                slider.setRange(
                    round(channel.min_deg * 10),
                    round(channel.max_deg * 10),
                )
                slider.setValue(round(command_angle.value() * 10))
                command_angle.valueChanged.connect(
                    lambda value, control=slider: control.setValue(round(value * 10))
                )
                slider.valueChanged.connect(
                    lambda value, control=command_angle: control.setValue(value / 10.0)
                )
                table.setCellWidget(row, 8, command_angle)
                table.setCellWidget(row, 9, slider)
                self.rows[channel.name] = RowWidgets(
                    channel,
                    selected,
                    current_angle,
                    current_pulse,
                    target_angle,
                    target_pulse,
                    command_angle,
                    slider,
                    moving,
                )
            table.resizeColumnsToContents()
            table.horizontalHeader().setStretchLastSection(True)
            layout.addWidget(table)

            controls = QHBoxLayout()
            controls.addWidget(QLabel("Duration ms"))
            self.duration = QSpinBox()
            self.duration.setRange(100, 9999)
            self.duration.setValue(1000)
            controls.addWidget(self.duration)
            send_selected = QPushButton("Send selected")
            send_selected.clicked.connect(self._send_selected)
            controls.addWidget(send_selected)
            send_all = QPushButton("Send all")
            send_all.clicked.connect(self._send_all)
            controls.addWidget(send_all)
            reset = QPushButton("Reset")
            reset.clicked.connect(node.reset)
            controls.addWidget(reset)
            controls.addStretch()
            layout.addLayout(controls)
            layout.addWidget(
                QLabel(
                    "Current values are ESP32 open-loop calculations, not sensor feedback. "
                    "OK means accepted, not motion complete."
                )
            )
            self.setCentralWidget(central)

            timer = QTimer(self)
            timer.timeout.connect(self._drain_events)
            timer.start(50)

        def _send_selected(self) -> None:
            selected = [
                row for row in self.rows.values() if row.selected.isChecked()
            ]
            if not selected:
                self.event_label.setText("Last event: no channels selected")
                return
            node.send_trajectory(
                [row.channel.name for row in selected],
                [math.radians(row.command_angle.value()) for row in selected],
                self.duration.value(),
            )
            self.event_label.setText("Last event: trajectory published")

        def _send_all(self) -> None:
            node.send_trajectory(
                [row.channel.name for row in self.rows.values()],
                [
                    math.radians(row.command_angle.value())
                    for row in self.rows.values()
                ],
                self.duration.value(),
            )
            self.event_label.setText("Last event: full trajectory published")

        def _drain_events(self) -> None:
            while True:
                try:
                    kind, payload = events.get_nowait()
                except queue.Empty:
                    break
                if kind == "status":
                    status = json.loads(payload)
                    self.status_label.setText(
                        f"Port {status['port']} | connected={status['connected']} | "
                        f"ready={status['ready']} | {status['message']} | "
                        f"feedback={status['feedback']}"
                    )
                elif kind == "serial_event":
                    self.event_label.setText(f"Last event: {payload}")
                elif kind == "reset_result":
                    self.event_label.setText(f"Last event: {payload}")
                elif kind == "current_angles":
                    for name, position in zip(
                        payload.name, payload.position, strict=False
                    ):
                        if name in self.rows:
                            value = math.degrees(position)
                            self.current_angles[name] = value
                            self.rows[name].current_angle.setText(f"{value:.2f}")
                    self._update_moving()
                elif kind == "target_angles":
                    for name, position in zip(
                        payload.name, payload.position, strict=False
                    ):
                        if name in self.rows and math.isfinite(position):
                            value = math.degrees(position)
                            self.target_angles[name] = value
                            self.rows[name].target_angle.setText(f"{value:.2f}")
                    self._update_moving()
                elif kind == "current_pulses":
                    for row, value in zip(
                        self.rows.values(), payload.data, strict=False
                    ):
                        row.current_pulse.setText(
                            f"{value:.2f}" if math.isfinite(value) else "-"
                        )
                elif kind == "target_pulses":
                    for row, value in zip(
                        self.rows.values(), payload.data, strict=False
                    ):
                        row.target_pulse.setText(
                            f"{value:.2f}" if math.isfinite(value) else "-"
                        )

        def _update_moving(self) -> None:
            for name, row in self.rows.items():
                current = self.current_angles.get(name)
                target = self.target_angles.get(name)
                if current is None or target is None:
                    row.moving.setText("-")
                else:
                    row.moving.setText(
                        "YES" if abs(current - target) > 0.2 else "NO"
                    )

    window = MainWindow()
    window.show()
    exit_code = app.exec()
    stop_event.set()
    ros_thread.join(timeout=2.0)
    node.destroy_node()
    rclpy.shutdown()
    raise SystemExit(exit_code)
