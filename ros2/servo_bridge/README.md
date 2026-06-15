# ROS2 Servo Bridge

ROS2 Jazzy에서 ESP32 서보 펌웨어와 통신하는 Python 브리지와 PySide6 UI다.
Windows 10/11과 Ubuntu 24.04를 같은 코드로 지원한다.

## Interfaces

- Subscribe: `/servo_bridge/joint_trajectory`
  (`trajectory_msgs/msg/JointTrajectory`)
- Publish: `/servo_bridge/joint_states`
  (`sensor_msgs/msg/JointState`, open-loop)
- Publish: `/servo_bridge/target_joint_states`
- Publish: `/servo_bridge/current_pulse_us`,
  `/servo_bridge/target_pulse_us`
- Publish: `/servo_bridge/status`, `/servo_bridge/serial_events`
- Publish: `/servo_bridge/diagnostics`
- Service: `/servo_bridge/reset` (`std_srvs/srv/Trigger`)

The first trajectory point is converted to one ESP32 multi-motor command.
ROS angles are radians; the ESP32 protocol and UI use degrees and microseconds.

## Build

Run these commands in a ROS2 Jazzy terminal.

```powershell
cd C:\pixi_ws
pixi shell
call C:\pixi_ws\ros2-windows\local_setup.bat
cd C:\dev\servo-bridge\ros2
colcon build --symlink-install
call install\local_setup.bat
```

Windows에서는 ROS2와 브리지 노드가 `C:\pixi_ws`의 Pixi Python을 사용한다.
PySide6 UI는 Qt5/Qt6 DLL 충돌을 피하기 위해 NumPy와 함께
`C:\pixi_ws\ui-venv`의 Python 3.12 환경으로 자동 재실행된다.
환경 위치를 바꾸려면 `SERVO_BRIDGE_UI_PYTHON`에 UI Python 경로를 지정한다.

Ubuntu:

```bash
cd ~/servo-bridge/ros2
python3 -m pip install pyserial PyYAML PySide6
colcon build --symlink-install
source install/setup.bash
```

## Run

Windows:

```powershell
ros2 launch servo_bridge bridge.launch.py serial_port:=COM3
ros2 run servo_bridge control_ui
```

Ubuntu:

```bash
ros2 launch servo_bridge bridge.launch.py serial_port:=/dev/ttyACM0
ros2 run servo_bridge control_ui
```

Edit `config/servos.yaml` to change ROS names, channel roles, and logical angle
limits. Only the bridge node opens the serial port.
