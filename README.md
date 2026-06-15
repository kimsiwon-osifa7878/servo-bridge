# ESP32 ROS2 Servo Bridge

ESP32-C3와 PCA9685로 최대 8개 서보를 제어하고, Windows의 ROS2 브리지와
PySide6 UI에서 각도와 PWM 상태를 확인하는 프로젝트다.

## 1. 배선 확인

- PCA9685 `VCC` → ESP32 `3.3V`
- PCA9685 `V+` → 별도 서보 전원
- PCA9685 `SDA` → GPIO 8
- PCA9685 `SCL` → GPIO 9
- PCA9685 `OE` → GPIO 10
- `OE`와 PCA9685 `VCC` 사이에 10kΩ 풀업 저항 연결
- ESP32, PCA9685, 서보 전원의 GND를 공통 연결

서보 전원은 ESP32에서 공급하지 않는다.

## 2. ESP32 펌웨어 업로드

Arduino IDE에서 다음을 준비한다.

- 보드: `Nologo ESP32C3 Super Mini`
- 라이브러리: `Adafruit PWM Servo Driver Library`
- 파일: `servo-bridge.ino`

보드를 연결하고 업로드한다. 업로드 후 시리얼 모니터를 `115200 baud`로
열었을 때 부팅 과정 뒤에 `READY`가 출력되어야 한다.

## 3. ROS2 패키지 빌드

처음 한 번 또는 Python 코드를 수정한 뒤 실행한다.

```powershell
cd C:\pixi_ws
pixi shell
call C:\pixi_ws\ros2-windows\local_setup.bat
cd C:\dev\servo-bridge\ros2
colcon build --symlink-install
```

## 4. 브리지 실행

새 PowerShell 창을 열고 다음 명령을 실행한다.

```powershell
cd C:\pixi_ws
pixi shell
call C:\pixi_ws\ros2-windows\local_setup.bat
call C:\dev\servo-bridge\ros2\install\local_setup.bat
ros2 launch servo_bridge bridge.launch.py serial_port:=COM3
```

ESP32 포트가 `COM3`이 아니면 장치 관리자에서 확인한 포트로 바꾼다.

## 5. UI 실행

브리지를 실행한 상태에서 두 번째 PowerShell 창을 열고 실행한다.

```powershell
cd C:\pixi_ws
pixi shell
call C:\pixi_ws\ros2-windows\local_setup.bat
call C:\dev\servo-bridge\ros2\install\local_setup.bat
ros2 run servo_bridge control_ui
```

UI 사용 순서:

1. 상단에서 `connected=True`, `ready=True`인지 확인한다.
2. 움직일 채널의 `Use`를 선택한다.
3. `Command deg` 또는 슬라이더로 목표 각도를 정한다.
4. `Duration ms`에 이동 시간을 입력한다.
5. `Send selected` 또는 `Send all`을 누른다.
6. 초기 위치로 이동하려면 `Reset`을 누른다.

`Current deg/us`는 센서 측정값이 아니라 ESP32가 계산한 현재 상태다.
`Target deg/us`는 ESP32가 접수한 목표값이다.

## 설정 변경

채널 이름, 종류, 각도 제한은 다음 파일에서 변경한다.

```text
ros2/servo_bridge/config/servos.yaml
```

7번과 8번은 관절뿐 아니라 그리퍼 또는 보조 장치로 사용할 수 있다.
설정 변경 후 ROS2 패키지를 다시 빌드한다.

## 종료

UI를 먼저 닫고 브리지 PowerShell에서 `Ctrl+C`를 누른다.

## 문제 해결

- `connected=False`: COM 포트 번호와 다른 시리얼 프로그램의 포트 점유를 확인한다.
- `ready=False`: ESP32 시리얼에 `READY`가 출력되는지 확인한다.
- UI가 열리지 않음: ROS2 빌드 후 `install\local_setup.bat`을 다시 실행한다.
- 서보가 움직이지 않음: 별도 서보 전원, 공통 GND, PCA9685 `V+`를 확인한다.
- RTI Connext 경고: 현재 사용하는 Cyclone DDS와 무관하므로 무시해도 된다.
