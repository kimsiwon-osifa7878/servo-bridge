# Serial Servo Bridge

ROS2 없이 ESP32 `servo-bridge.ino` 펌웨어에 직접 시리얼 명령을 보내는 간단한 제어 UI입니다.

## 설치

```powershell
cd C:\dev\servo-bridge\serial-bridge
python -m venv .venv
.\.venv\Scripts\python -m pip install -r requirements.txt
```

## 실행

```powershell
cd C:\dev\servo-bridge\serial-bridge
.\run.ps1
```

직접 실행하려면 다음 명령을 사용해도 됩니다.

```powershell
.\.venv\Scripts\python app.py
```

기본 포트는 `COM3`이고 baud rate는 `115200`입니다. 포트 입력칸에서 다른 COM 포트로 바꿀 수 있습니다.

## 동작

- `Connect`를 누르면 시리얼 포트를 열고 `#START`를 자동 전송한 뒤 ESP32의 `READY`를 기다립니다.
- 앱이 `READY`를 놓친 경우를 대비해 연결 직후 `#PINMAP`을 자동 전송합니다.
- `OK`, `PINMAP`, `STATE`, 모터 상세 응답 중 하나를 받으면 ESP32가 응답 가능한 상태로 보고 조작을 활성화합니다.
- `READY` 전에는 모터 슬라이더와 전송 버튼이 비활성화됩니다.
- `Serial Commands` 영역에서 펌웨어의 관리 명령을 버튼으로 보낼 수 있습니다.
  - `#START`, `#RESET`, `#RESET_BOOT`
  - `#PINMAP`
  - `#STATE`, `#STATE ON`, `#STATE OFF`
  - `#TEST`, `#TEST ON`, `#TEST OFF`, `#TEST<모터번호>`
  - `#GOPOS<번호>T<duration_ms>`
  - `#OFFSET`, `#OFFSET#RESET`, `#OFFSET#<모터번호>A<offset>`
  - Raw PWM `#<모터번호>P<pulse_us>T<duration_ms>`
- 로그 화면 바로 위 입력창에서는 Arduino IDE 시리얼 모니터처럼 직접 명령을 입력해 보낼 수 있습니다.
- 슬라이더를 놓거나 숫자 입력을 확정하면 `#<motor>A<angle>T<duration_ms>` 명령을 보냅니다.
- 기본 duration은 `1000ms`입니다.
- `STATE`는 현재 open-loop 계산값으로 표시하며 실제 센서 측정값이 아닙니다.
- `OK`는 명령 접수 의미이며 물리 이동 완료 의미가 아닙니다.

## 향후 MuJoCo 연동

UI와 시리얼 전송은 분리되어 있습니다. MuJoCo 연동 코드는 `SerialClient.send_angle(motor, angle_deg, duration_ms)`를 호출하는 별도 모듈로 추가하면 됩니다.
