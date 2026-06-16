# ESP32 ROS2 Servo Bridge

ESP32-C3가 서보를 직접 GPIO PWM으로 제어하고, Windows의 ROS2 브리지와 PySide6 UI에서 각도와 펄스 상태를 확인하는 프로젝트입니다. PCA9685는 사용하지 않습니다.

## 1. 배선

- 서보 신호선:
  - motor 1 -> GPIO 7
  - motor 2 -> GPIO 8
  - motor 3 -> GPIO 9
  - motor 4 -> GPIO 10
  - motor 5 -> GPIO 20
  - motor 6 -> GPIO 21
- 시리얼에서 `#PINMAP`을 보내면 현재 펌웨어의 모터 번호와 GPIO 매핑을 확인할 수 있다.
- 서보 전원은 별도 전원을 사용한다.
- ESP32 GND와 서보 전원 GND를 공통으로 연결한다.
- ESP32 보드에서 서보 전원 레일에 전원을 공급하지 않는다.

## 2. ESP32 업로드

Arduino IDE에서 다음을 준비한다.

- 보드: `Nologo ESP32C3 Super Mini`
- 옵션: `USB CDC On Boot` 활성화
- 라이브러리: `ServoEasing`, `ESP32Servo`
- 파일: `servo-bridge.ino`

업로드 후 시리얼 모니터를 `115200 baud`로 열고 `READY`가 출력되는지 확인한다.

## 3. ROS2 브리지 빌드

```powershell
cd C:\pixi_ws
pixi shell
call C:\pixi_ws\ros2-windows\local_setup.bat
cd C:\dev\servo-bridge\ros2
colcon build --symlink-install
```

## 4. 브리지 실행

```powershell
cd C:\pixi_ws
pixi shell
call C:\pixi_ws\ros2-windows\local_setup.bat
call C:\dev\servo-bridge\ros2\install\local_setup.bat
ros2 launch servo_bridge bridge.launch.py serial_port:=COM3
```

ESP32 포트가 `COM3`가 아니면 장치 관리자에서 확인한 포트로 바꾼다.

## 5. UI 실행

브리지를 실행한 상태에서 다른 PowerShell 창을 열고 실행한다.

```powershell
cd C:\pixi_ws
pixi shell
call C:\pixi_ws\ros2-windows\local_setup.bat
call C:\dev\servo-bridge\ros2\install\local_setup.bat
ros2 run servo_bridge control_ui
```

UI 사용 순서:

1. 상단에서 `connected=True`, `ready=True`인지 확인한다.
2. 움직일 모터의 `Use`를 선택한다.
3. `Command deg` 또는 슬라이더로 목표 각도를 정한다.
4. `Duration ms`에 이동 시간을 입력한다.
5. `Send selected` 또는 `Send all`을 누른다.
6. 초기 위치로 이동하려면 `Reset`을 누른다.

펌웨어 리셋 순서는 기본적으로 motor 2, 3, 4, 5, 6, 1 순서이다.

## 6. 설정 변경

모터 이름, 각도 제한, UI 표시 이름은 다음 파일에서 바꾼다.

```text
ros2/servo_bridge/config/servos.yaml
```

펌웨어 기본값은 6개 모터이다. 다른 ESP32 보드에서 더 많은 채널을 쓰려면 `servo-bridge.ino`의 `SERVO_COUNT`, `SERVO_PINS`, 모든 축별 배열, ROS2 `servos.yaml`을 함께 늘린다.

오프셋 명령:

- `#PINMAP`: 모터 번호와 GPIO 핀맵 조회
- `#OFFSET`: 현재 오프셋 조회
- `#OFFSET#1A-9.45#3A2.70`: 오프셋 저장
- `#OFFSET#RESET`: 저장된 오프셋 삭제 후 기본값으로 복귀

채널 수가 바뀌어도 저장된 오프셋 blob의 앞쪽 값은 현재 사용하는 축에 맞춰 가능한 만큼 적용된다.

## 문제 해결

- `connected=False`: COM 포트 번호와 다른 프로그램의 포트 점유를 확인한다.
- `ready=False`: ESP32 시리얼에 `READY`가 출력되는지 확인한다.
- UI가 열리지 않음: ROS2 빌드 후 `install\local_setup.bat`을 다시 실행한다.
- 서보가 움직이지 않음: 별도 서보 전원, 공통 GND, 신호선 GPIO 번호를 확인한다.
- ESP32 리셋 중 신호 보장: 직접 GPIO 방식은 리셋 순간 출력 상태를 완전히 보장하지 못한다. 필요하면 서보 전원 스위칭 또는 외부 풀다운 회로를 추가한다.
