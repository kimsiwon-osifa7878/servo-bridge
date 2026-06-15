from __future__ import annotations

import json
import math
import queue
import time
from pathlib import Path

import rclpy
from ament_index_python.packages import get_package_share_directory
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray, String
from std_srvs.srv import Trigger
from trajectory_msgs.msg import JointTrajectory

from .commands import trajectory_to_serial_command
from .config import load_bridge_config
from .protocol import EventKind, ProtocolEvent
from .serial_transport import ConnectionStatus, SerialTransport


class ServoBridgeNode(Node):
    def __init__(self) -> None:
        super().__init__("servo_bridge")
        default_config = str(
            Path(get_package_share_directory("servo_bridge"))
            / "config"
            / "servos.yaml"
        )
        self.declare_parameter("serial_port", "COM3")
        self.declare_parameter("baud_rate", 115200)
        self.declare_parameter("config_file", default_config)
        self.declare_parameter("reconnect_interval", 2.0)
        self.declare_parameter("command_timeout", 2.0)
        self.declare_parameter("max_command_rate_hz", 20.0)

        self.config = load_bridge_config(
            self.get_parameter("config_file").get_parameter_value().string_value
        )
        self._events: queue.Queue[
            ProtocolEvent | ConnectionStatus | tuple[str, bool, str]
        ] = queue.Queue()
        self._connection = ConnectionStatus(
            False,
            False,
            self.get_parameter("serial_port").value,
            "not connected",
        )
        self._last_rx_time = 0.0
        self._last_error = ""
        self._last_event = ""
        self._current_angles = {channel.motor: math.nan for channel in self.config.channels}
        self._current_pulses = {channel.motor: math.nan for channel in self.config.channels}
        self._target_angles = {channel.motor: math.nan for channel in self.config.channels}
        self._target_pulses = {channel.motor: math.nan for channel in self.config.channels}

        self.transport = SerialTransport(
            port=str(self.get_parameter("serial_port").value),
            baud_rate=int(self.get_parameter("baud_rate").value),
            reconnect_interval=float(self.get_parameter("reconnect_interval").value),
            command_timeout=float(self.get_parameter("command_timeout").value),
            max_motion_rate_hz=float(
                self.get_parameter("max_command_rate_hz").value
            ),
            event_callback=self._events.put,
            status_callback=self._events.put,
        )

        callback_group = ReentrantCallbackGroup()
        self.create_subscription(
            JointTrajectory,
            "/servo_bridge/joint_trajectory",
            self._trajectory_callback,
            10,
            callback_group=callback_group,
        )
        self.create_service(
            Trigger,
            "/servo_bridge/reset",
            self._reset_callback,
            callback_group=callback_group,
        )
        self._joint_state_pub = self.create_publisher(
            JointState, "/servo_bridge/joint_states", 10
        )
        self._target_state_pub = self.create_publisher(
            JointState, "/servo_bridge/target_joint_states", 10
        )
        self._current_pulse_pub = self.create_publisher(
            Float64MultiArray, "/servo_bridge/current_pulse_us", 10
        )
        self._target_pulse_pub = self.create_publisher(
            Float64MultiArray, "/servo_bridge/target_pulse_us", 10
        )
        self._event_pub = self.create_publisher(
            String, "/servo_bridge/serial_events", 20
        )
        self._status_pub = self.create_publisher(
            String, "/servo_bridge/status", 10
        )
        self._diagnostic_pub = self.create_publisher(
            DiagnosticArray, "/servo_bridge/diagnostics", 10
        )
        self.create_timer(0.02, self._drain_events, callback_group=callback_group)
        self.create_timer(0.5, self._publish_status, callback_group=callback_group)
        self.transport.start()

    def destroy_node(self) -> bool:
        self.transport.stop()
        return super().destroy_node()

    def _trajectory_callback(self, message: JointTrajectory) -> None:
        if not message.points:
            self.get_logger().error("trajectory has no points")
            return
        point = message.points[0]
        duration = (
            float(point.time_from_start.sec)
            + float(point.time_from_start.nanosec) / 1_000_000_000.0
        )
        try:
            command = trajectory_to_serial_command(
                self.config,
                message.joint_names,
                point.positions,
                duration,
            )
        except ValueError as exc:
            self._last_error = str(exc)
            self.get_logger().error(str(exc))
            return
        request = self.transport.send_motion(command)
        request.add_done_callback(
            lambda result: self._events.put(
                ("command_result", result.accepted, result.response)
            )
        )

    def _reset_callback(
        self,
        request: Trigger.Request,
        response: Trigger.Response,
    ) -> Trigger.Response:
        del request
        result = self.transport.execute(
            "#RESET",
            timeout=float(self.get_parameter("command_timeout").value) + 0.5,
        )
        response.success = result.accepted
        response.message = result.response
        return response

    def _drain_events(self) -> None:
        while True:
            try:
                item = self._events.get_nowait()
            except queue.Empty:
                break
            if isinstance(item, ConnectionStatus):
                self._connection = item
                if not item.connected:
                    self._last_error = item.message
                continue
            if isinstance(item, tuple):
                _, accepted, response = item
                if not accepted:
                    self._last_error = response
                    self.get_logger().error(f"command rejected: {response}")
                continue
            self._handle_protocol_event(item)

    def _handle_protocol_event(self, event: ProtocolEvent) -> None:
        self._last_rx_time = time.time()
        self._last_event = event.raw
        self._event_pub.publish(String(data=event.raw))
        if event.kind is EventKind.ERROR:
            self._last_error = event.raw
        elif event.kind is EventKind.STATE:
            initialized_target = False
            for state in event.states:
                if state.motor in self._current_angles:
                    self._current_angles[state.motor] = state.angle_deg
                    self._current_pulses[state.motor] = state.pulse_us
                    if math.isnan(self._target_angles[state.motor]):
                        self._target_angles[state.motor] = state.angle_deg
                        self._target_pulses[state.motor] = state.pulse_us
                        initialized_target = True
            self._publish_current_state()
            if initialized_target:
                self._publish_target_state()
        elif event.kind is EventKind.TARGET:
            for target in event.targets:
                if target.motor not in self._target_pulses:
                    continue
                self._target_pulses[target.motor] = target.pulse_us
                if target.mode == "angle":
                    self._target_angles[target.motor] = target.input_value
            self._publish_target_state()

    def _publish_current_state(self) -> None:
        message = JointState()
        message.header.stamp = self.get_clock().now().to_msg()
        message.name = [channel.name for channel in self.config.channels]
        message.position = [
            math.radians(self._current_angles[channel.motor])
            for channel in self.config.channels
        ]
        self._joint_state_pub.publish(message)
        self._current_pulse_pub.publish(
            Float64MultiArray(
                data=[
                    self._current_pulses[channel.motor]
                    for channel in self.config.channels
                ]
            )
        )

    def _publish_target_state(self) -> None:
        message = JointState()
        message.header.stamp = self.get_clock().now().to_msg()
        message.name = [channel.name for channel in self.config.channels]
        message.position = [
            math.radians(self._target_angles[channel.motor])
            for channel in self.config.channels
        ]
        self._target_state_pub.publish(message)
        self._target_pulse_pub.publish(
            Float64MultiArray(
                data=[
                    self._target_pulses[channel.motor]
                    for channel in self.config.channels
                ]
            )
        )

    def _publish_status(self) -> None:
        payload = {
            "connected": self._connection.connected,
            "ready": self._connection.ready,
            "port": self._connection.port,
            "message": self._connection.message,
            "last_error": self._last_error,
            "last_event": self._last_event,
            "last_rx_unix": self._last_rx_time,
            "feedback": "open_loop",
        }
        self._status_pub.publish(
            String(data=json.dumps(payload, ensure_ascii=False))
        )

        diagnostic = DiagnosticArray()
        diagnostic.header.stamp = self.get_clock().now().to_msg()
        status = DiagnosticStatus()
        status.name = "servo_bridge/serial"
        status.hardware_id = self._connection.port
        if self._connection.ready:
            status.level = DiagnosticStatus.OK
            status.message = "ESP32 ready; state is open-loop"
        elif self._connection.connected:
            status.level = DiagnosticStatus.WARN
            status.message = self._connection.message
        else:
            status.level = DiagnosticStatus.ERROR
            status.message = self._connection.message
        status.values = [
            KeyValue(key="connected", value=str(self._connection.connected)),
            KeyValue(key="ready", value=str(self._connection.ready)),
            KeyValue(key="feedback", value="open_loop"),
            KeyValue(key="last_error", value=self._last_error),
        ]
        diagnostic.status = [status]
        self._diagnostic_pub.publish(diagnostic)


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = ServoBridgeNode()
    executor = MultiThreadedExecutor(num_threads=3)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()
