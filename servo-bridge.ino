#include <Adafruit_PWMServoDriver.h>
#include <Preferences.h>
#include <Wire.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>

#if !defined(ARDUINO_USB_CDC_ON_BOOT) || !ARDUINO_USB_CDC_ON_BOOT
#error "Enable Tools > USB CDC On Boot, or select Nologo ESP32C3 Super Mini."
#endif

/*
  ESP32-C3 SuperMini - ROS2 8축 서보 브리지

  필요한 라이브러리:
    Arduino Library Manager에서 "Adafruit PWM Servo Driver Library" 설치

  시리얼 설정:
    115200 baud, 8 data bits, no parity, 1 stop bit
    각 명령 끝에는 반드시 줄바꿈('\n')을 붙인다.
    명령은 대문자를 사용한다.

  ROS2/pyserial 송신 예:
    serial_port.write(b"#RESET\r\n")
    serial_port.write(b"#1A30.4\r\n")
    serial_port.write(b"#1P1500#2P1500T1000D800\r\n")
    serial_port.write(b"#1A45.5#2P1600#3A120T2000\r\n")

  프로토콜:
    #RESET                       모든 축을 초기 위치로 이동
    #TEST1 ~ #TEST8              지정 모터를 선택하고 자동 테스트 시작
    #TEST ON / #TEST OFF         선택된 축의 자동 테스트 켜기/끄기
    #TEST                        자동 테스트 상태 토글
    #OFFSET                      현재 축별 각도 오프셋 조회
    #OFFSET#1A-9.45#3A2.7        축별 각도 오프셋 저장
    #GOPOS1T1000                 기본 포지션 1로 이동
    #<채널>A<각도>[T시간][D지연]  degree 명령, 소수점 사용 가능
    #<채널>P<펄스>[T시간][D지연]  raw PWM 명령, 500~2500 us
    T와 D는 바로 앞 채널에만 적용되며 각각 100~9999 ms이다.

    예: #1A110#2A110T3000#3A70T2000D1000
    같은 채널이 여러 번 나오면 마지막 값이 적용된다.
    T가 기본 속도보다 빠르면 안전한 최소 실행시간으로 자동 연장된다.
    D 대기 중 같은 채널의 새 명령이 오면 해당 채널만 교체된다.

  정상적으로 명령을 받으면 "OK"를 반환한다.
  잘못된 명령은 "ERR,<코드>,<한국어 설명>,INPUT=<입력>"을 반환한다.
  이동 중에는 각 20 ms 모션 갱신 후 아래 형식으로 현재 명령 각도를 반환한다.
    STATE,<1번 각도>,<2번 각도>,...,<8번 각도>
  STATE 값은 센서 측정값이 아니라 ESP32가 계산한 현재 보간 각도이다.
  ROS2 브리지는 줄의 첫 필드로 READY, OK, ERR, STATE를 구분하면 된다.

  A 명령:
    각 축의 SERVO_MIN_DEG와 SERVO_MAX_DEG 범위로 제한된다.
    180도 서보는 논리각과 물리각이 같다.
    270도 서보는 중앙 180도 구간을 사용하므로 논리각에 45도를 더한다.
    예: 논리각 90도 -> 물리각 135도, 논리각 180도 -> 물리각 225도.

  P 명령:
    500~2500 us 범위의 물리 PWM을 직접 지정한다.
    축별 각도 offset과 방향 설정을 적용하지 않는다.

  각 축은 독립적인 실행시간과 지연시간을 사용한다.
  지정하지 않은 축은 기존 움직임과 대기 명령을 계속한다.
*/

#define SERVO_COUNT 8
#define SERVO_POSITION_COUNT 2
#define SERVO_POSITION_DEFAULT_TIME_MS 2000
#define SERIAL_BAUD_RATE 115200
#define SERVO_UPDATE_INTERVAL_MS 20
#define SERVO_SPEED_DEG_PER_SEC 120.0f //1초동안 몇도 이상 못 움직이게. 값을 올리면 빨라진다. 안전을 위해 일부러 느리게 만들었음
#define SERVO_DEADZONE_COUNTS 3
#define SERVO_FINAL_STEP_COUNTS 3
#define SERVO_RESET_STAGGER_MS 1000
#define SERIAL_BUFFER_SIZE 160
#define PWM_INITIALIZATION_DEBUG true

// PCB/배선 진단용 자동 왕복 테스트. 테스트가 끝나면 false로 변경한다. AXIS 1은 1번 모터를 지칭한다. 0 번 채털이 아니다.
#define SERVO_TEST_MODE false
#define SERVO_TEST_AXIS 1
#define SERVO_TEST_LOW_DEG 45.0f
#define SERVO_TEST_HIGH_DEG 135.0f
#define SERVO_TEST_START_DELAY_MS 1500
#define SERVO_TEST_PAUSE_MS 500

#define SERVO_DIR_NORMAL 1
#define SERVO_DIR_REVERSED -1

const uint8_t SERVO_PWM_I2C_ADDRESS = 0x40;
const uint8_t PCA9685_SDA_PIN = 8;
const uint8_t PCA9685_SCL_PIN = 9;
const uint8_t PCA9685_OE_PIN = 10;
const uint32_t PCA9685_I2C_FREQUENCY_HZ = 400000;
const uint32_t PCA9685_OSCILLATOR_HZ = 25000000;
const uint16_t PCA9685_PWM_FREQUENCY_HZ = 50;
const uint8_t PCA9685_50HZ_PRESCALE = 121;
const float PCA9685_ACTUAL_FREQUENCY_HZ =
  (float)PCA9685_OSCILLATOR_HZ
    / (4096.0f * ((float)PCA9685_50HZ_PRESCALE + 1.0f));

const uint8_t SERVO_CHANNELS[SERVO_COUNT] = {1, 2, 3, 4, 5, 6, 7, 8};
const char OFFSET_NVS_NAMESPACE[] = "servo-bridge";
const char OFFSET_NVS_KEY[] = "offsets";
const float SERVO_MIN_OFFSET_DEG = -180.0f;
const float SERVO_MAX_OFFSET_DEG = 180.0f;

static_assert(SERVO_COUNT <= 15, "PCA9685 channel 0 is reserved.");
static_assert(
  SERVO_TEST_AXIS >= 1 && SERVO_TEST_AXIS <= SERVO_COUNT,
  "SERVO_TEST_AXIS must be a motor number from 1 to SERVO_COUNT."
);



// 각 축에 연결한 서보에 맞춰 180.0f 또는 270.0f로 수정한다.
const float SERVO_TRAVEL_DEG[SERVO_COUNT] = {
  270.0f, 270.0f, 270.0f, 270.0f,
  180.0f, 180.0f, 180.0f, 180.0f
};

const float SERVO_INITIAL_DEG[SERVO_COUNT] = {
  90.0f, 90.0f, 90.0f, 90.0f,
  90.0f, 90.0f, 90.0f, 90.0f
};

const float SERVO_POSITIONS[SERVO_POSITION_COUNT][SERVO_COUNT] = {
  {90.0f, 70.0f, 160.0f, 0.0f, 90.0f, 130.0f, 90.0f, 90.0f},
  {45.0f, 45.0f, 45.0f, 45.0f, 45.0f, 45.0f, 45.0f, 45.0f}
};

const int8_t SERVO_DIRECTION[SERVO_COUNT] = {
  SERVO_DIR_REVERSED, SERVO_DIR_REVERSED, SERVO_DIR_NORMAL,
  SERVO_DIR_REVERSED, SERVO_DIR_NORMAL, SERVO_DIR_NORMAL,
  SERVO_DIR_NORMAL, SERVO_DIR_NORMAL
};

// 저장된 오프셋이 없을 때 사용하는 기본값이며 단위는 degree이다.
const float DEFAULT_SERVO_OFFSET_DEG[SERVO_COUNT] = {
  0.0f, 0.0f,0.0f,0.0f,0.0f,0.0f, 0.0f, 0.0f
};
float servoOffsetDeg[SERVO_COUNT];
const float SERVO_MIN_DEG[SERVO_COUNT] = {
  0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 70.0f, 0.0f, 0.0f
};
const float SERVO_MAX_DEG[SERVO_COUNT] = {
  180.0f, 180.0f,180.0f,180.0f,180.0f,150.0f,180.0f,180.0f
};

const float SERVO_MIN_PULSE_US = 500.0f;
const float SERVO_MAX_PULSE_US = 2500.0f;
const float SERVO_PULSE_RANGE_US =
  SERVO_MAX_PULSE_US - SERVO_MIN_PULSE_US;

Adafruit_PWMServoDriver pwm(SERVO_PWM_I2C_ADDRESS, Wire);
bool pca9685Ready = false;
enum PwmDebugPhase : uint8_t {
  PWM_DEBUG_NONE,
  PWM_DEBUG_BOOT,
  PWM_DEBUG_RESET
};
PwmDebugPhase pwmDebugPhase = PWM_DEBUG_NONE;
uint32_t pwmDebugWriteCount = 0;

// 모든 펄스 상태값은 실제 출력되는 raw PWM 값이다.
float currentPulseUs[SERVO_COUNT];
float startPulseUs[SERVO_COUNT];
float targetPulseUs[SERVO_COUNT];
float approachPulseUs[SERVO_COUNT];
int lastWrittenPwmCount[SERVO_COUNT];

bool axisMotionActive[SERVO_COUNT];
bool axisFinalStepEnabled[SERVO_COUNT];
bool axisApproachWritten[SERVO_COUNT];
uint32_t axisMotionStartMs[SERVO_COUNT];
uint32_t axisMotionDurationMs[SERVO_COUNT];
uint32_t lastMotionUpdateMs = 0;

char serialBuffer[SERIAL_BUFFER_SIZE];
size_t serialLength = 0;
bool serialOverflow = false;

struct PendingCommand {
  bool selected[SERVO_COUNT];
  bool angleMode[SERVO_COUNT];
  float inputValue[SERVO_COUNT];
  float targetPulseUs[SERVO_COUNT];
  bool hasRequestedTime[SERVO_COUNT];
  uint32_t requestedTimeMs[SERVO_COUNT];
  uint32_t delayMs[SERVO_COUNT];
};

struct PendingAxisCommand {
  bool active;
  bool reportResetStart;
  float targetPulseUs;
  bool hasRequestedTime;
  uint32_t requestedTimeMs;
  uint32_t executeAtMs;
};

PendingAxisCommand pendingAxisCommand[SERVO_COUNT] = {};
bool bootInitializationPending = false;

bool servoTestEnabled = SERVO_TEST_MODE;
size_t servoTestAxis = SERVO_TEST_AXIS - 1;
bool testMovingToHigh = true;
uint32_t nextTestMotionMs = 0;

float clampFloat(float value, float minimum, float maximum) {
  if (value < minimum) {
    return minimum;
  }
  if (value > maximum) {
    return maximum;
  }
  return value;
}

float applyDirectionToAngle(size_t axis, float angleDeg) {
  if (SERVO_DIRECTION[axis] == SERVO_DIR_REVERSED) {
    return 180.0f - angleDeg;
  }
  return angleDeg;
}

float angleToPulseUs(size_t axis, float angleDeg) {
  const float safeAngle =
    clampFloat(angleDeg, SERVO_MIN_DEG[axis], SERVO_MAX_DEG[axis]);
  const float extraTravelDeg =
    (SERVO_TRAVEL_DEG[axis] - 180.0f) * 0.5f;
  const float correctedAngle =
    clampFloat(
      safeAngle + servoOffsetDeg[axis],
      -extraTravelDeg,
      180.0f + extraTravelDeg
    );
  const float directedAngle =
    applyDirectionToAngle(axis, correctedAngle);
  const float centeredPhysicalAngle =
    directedAngle + extraTravelDeg;

  return SERVO_MIN_PULSE_US
    + (centeredPhysicalAngle / SERVO_TRAVEL_DEG[axis])
      * SERVO_PULSE_RANGE_US;
}

float pulseToAngleDeg(size_t axis, float pulseUs) {
  const float physicalAngle = (pulseUs - SERVO_MIN_PULSE_US)
    * SERVO_TRAVEL_DEG[axis] / SERVO_PULSE_RANGE_US;
  const float directedAngle =
    physicalAngle - (SERVO_TRAVEL_DEG[axis] - 180.0f) * 0.5f;
  const float correctedAngle =
    applyDirectionToAngle(axis, directedAngle);

  return correctedAngle - servoOffsetDeg[axis];
}

void useDefaultServoOffsets() {
  memcpy(
    servoOffsetDeg,
    DEFAULT_SERVO_OFFSET_DEG,
    sizeof(servoOffsetDeg)
  );
}

void loadServoOffsets() {
  useDefaultServoOffsets();

  Preferences preferences;
  if (!preferences.begin(OFFSET_NVS_NAMESPACE, true)) {
    return;
  }

  if (preferences.getBytesLength(OFFSET_NVS_KEY)
      == sizeof(servoOffsetDeg)) {
    float storedOffsets[SERVO_COUNT];
    const size_t bytesRead = preferences.getBytes(
      OFFSET_NVS_KEY,
      storedOffsets,
      sizeof(storedOffsets)
    );
    bool valid = bytesRead == sizeof(storedOffsets);
    for (size_t axis = 0; valid && axis < SERVO_COUNT; ++axis) {
      valid = isfinite(storedOffsets[axis])
        && storedOffsets[axis] >= SERVO_MIN_OFFSET_DEG
        && storedOffsets[axis] <= SERVO_MAX_OFFSET_DEG;
    }
    if (valid) {
      memcpy(servoOffsetDeg, storedOffsets, sizeof(servoOffsetDeg));
    }
  }

  preferences.end();
}

bool saveServoOffsets(const float offsets[SERVO_COUNT]) {
  Preferences preferences;
  if (!preferences.begin(OFFSET_NVS_NAMESPACE, false)) {
    return false;
  }

  const size_t bytesWritten = preferences.putBytes(
    OFFSET_NVS_KEY,
    offsets,
    sizeof(servoOffsetDeg)
  );
  preferences.end();
  return bytesWritten == sizeof(servoOffsetDeg);
}

void reportServoOffsets() {
  Serial.print("angle offset => ");
  for (size_t axis = 0; axis < SERVO_COUNT; ++axis) {
    if (axis > 0) {
      Serial.print(',');
    }
    Serial.print("motor");
    Serial.print(axis + 1);
    Serial.print(':');
    Serial.print(servoOffsetDeg[axis], 2);
  }
  Serial.println();
}

float pulseDeltaToDegrees(size_t axis, float pulseDeltaUs) {
  return fabsf(pulseDeltaUs) * SERVO_TRAVEL_DEG[axis]
    / SERVO_PULSE_RANGE_US;
}

uint16_t pulseUsToPwmCount(float pulseUs) {
  const float count =
    pulseUs * PCA9685_ACTUAL_FREQUENCY_HZ * 4096.0f / 1000000.0f;
  return (uint16_t)clampFloat(lroundf(count), 0.0f, 4095.0f);
}

float pwmCountToPulseUs(uint16_t count) {
  return (float)count * 1000000.0f
    / (PCA9685_ACTUAL_FREQUENCY_HZ * 4096.0f);
}

const char *pwmDebugPhaseName() {
  return pwmDebugPhase == PWM_DEBUG_BOOT ? "BOOT" : "RESET";
}

void startPwmDebug(PwmDebugPhase phase) {
  if (!PWM_INITIALIZATION_DEBUG) {
    return;
  }

  pwmDebugPhase = phase;
  pwmDebugWriteCount = 0;
  Serial.print("PWMDBG,START,phase=");
  Serial.println(pwmDebugPhaseName());
}

void stopPwmDebug() {
  if (pwmDebugPhase == PWM_DEBUG_NONE) {
    return;
  }

  Serial.print("PWMDBG,STOP,phase=");
  Serial.print(pwmDebugPhaseName());
  Serial.print(",total=");
  Serial.println(pwmDebugWriteCount);
  pwmDebugPhase = PWM_DEBUG_NONE;
}

void setServoPwm(size_t axis, uint16_t onCount, uint16_t offCount) {
  pwm.setPWM(SERVO_CHANNELS[axis], onCount, offCount);

  if (pwmDebugPhase == PWM_DEBUG_NONE) {
    return;
  }

  ++pwmDebugWriteCount;
  Serial.print("PWMDBG,seq=");
  Serial.print(pwmDebugWriteCount);
  Serial.print(",phase=");
  Serial.print(pwmDebugPhaseName());
  Serial.print(",time=");
  Serial.print(millis());
  Serial.print(",motor=");
  Serial.print(axis + 1);
  Serial.print(",channel=");
  Serial.print(SERVO_CHANNELS[axis]);
  Serial.print(",on=");
  Serial.print(onCount);
  Serial.print(",off=");
  Serial.print(offCount);
  Serial.print(",pulse=");
  if (onCount == 0 && offCount == 0) {
    Serial.println("OFF");
  } else {
    Serial.print(pwmCountToPulseUs(offCount), 2);
    Serial.println("us");
  }
}

void writeServoPulse(size_t axis, float logicalPulseUs, bool forceWrite) {
  if (!pca9685Ready) {
    return;
  }

  const float outputPulseUs =
    clampFloat(logicalPulseUs, SERVO_MIN_PULSE_US, SERVO_MAX_PULSE_US);

  const uint16_t pwmCount = pulseUsToPwmCount(outputPulseUs);
  const int changeCounts =
    abs((int)pwmCount - lastWrittenPwmCount[axis]);

  if (forceWrite || changeCounts >= SERVO_DEADZONE_COUNTS) {
    setServoPwm(axis, 0, pwmCount);
    lastWrittenPwmCount[axis] = pwmCount;
  }
}

void reportCurrentAngles() {
  return;
  Serial.print("STATE");
  for (size_t axis = 0; axis < SERVO_COUNT; ++axis) {
    Serial.print(',');
    Serial.print(pulseToAngleDeg(axis, currentPulseUs[axis]), 1);
    Serial.print('/');
    Serial.print(currentPulseUs[axis]);
  }
  Serial.println();
  
}

bool updateMotion(uint32_t nowMs, bool forceEvaluation) {
  if (!forceEvaluation
      && (uint32_t)(nowMs - lastMotionUpdateMs)
        < SERVO_UPDATE_INTERVAL_MS) {
    return false;
  }

  lastMotionUpdateMs = nowMs;
  bool anyMotionUpdated = false;

  for (size_t axis = 0; axis < SERVO_COUNT; ++axis) {
    if (!axisMotionActive[axis]) {
      continue;
    }

    const uint32_t elapsedMs = nowMs - axisMotionStartMs[axis];
    bool finished = elapsedMs >= axisMotionDurationMs[axis];
    bool wroteApproachThisUpdate = false;

    if (axisFinalStepEnabled[axis]) {
      const uint32_t approachDurationMs =
        axisMotionDurationMs[axis] - SERVO_UPDATE_INTERVAL_MS;

      if (!axisApproachWritten[axis]
          && elapsedMs >= approachDurationMs) {
        currentPulseUs[axis] = approachPulseUs[axis];
        writeServoPulse(axis, currentPulseUs[axis], true);
        axisApproachWritten[axis] = true;
        wroteApproachThisUpdate = true;
      } else if (!axisApproachWritten[axis]) {
        const float progress =
          (float)elapsedMs / (float)approachDurationMs;
        const float easedProgress =
          0.5f - 0.5f * cosf(PI * progress);
        currentPulseUs[axis] = startPulseUs[axis]
          + (approachPulseUs[axis] - startPulseUs[axis])
            * easedProgress;
        writeServoPulse(axis, currentPulseUs[axis], false);
      }
    } else if (!finished) {
      const float progress =
        (float)elapsedMs / (float)axisMotionDurationMs[axis];
      const float easedProgress =
        0.5f - 0.5f * cosf(PI * progress);
      currentPulseUs[axis] = startPulseUs[axis]
        + (targetPulseUs[axis] - startPulseUs[axis]) * easedProgress;
      writeServoPulse(axis, currentPulseUs[axis], false);
    }

    if (finished && wroteApproachThisUpdate) {
      axisMotionDurationMs[axis] =
        elapsedMs + SERVO_UPDATE_INTERVAL_MS;
      finished = false;
    }

    if (finished) {
      currentPulseUs[axis] = targetPulseUs[axis];
      writeServoPulse(axis, currentPulseUs[axis], true);
    }

    anyMotionUpdated = true;

    if (finished) {
      axisMotionActive[axis] = false;
    }
  }

  if (anyMotionUpdated) {
    reportCurrentAngles();
  }
  return anyMotionUpdated;
}

uint32_t calculateMinimumDurationMs(
  size_t axis,
  float requestedPulseUs
) {
  const float deltaDeg = pulseDeltaToDegrees(
    axis,
    requestedPulseUs - currentPulseUs[axis]
  );
  if (deltaDeg <= 0.0f) {
    return 0;
  }

  uint32_t minimumDurationMs = (uint32_t)ceilf(
    deltaDeg * 1000.0f / SERVO_SPEED_DEG_PER_SEC
  );
  if (minimumDurationMs < SERVO_UPDATE_INTERVAL_MS) {
    minimumDurationMs = SERVO_UPDATE_INTERVAL_MS;
  }
  return minimumDurationMs;
}

void executeAxisCommand(
  size_t axis,
  float requestedPulseUs,
  bool hasRequestedTime,
  uint32_t requestedTimeMs,
  uint32_t nowMs
) {
  updateMotion(nowMs, true);

  const uint32_t minimumDurationMs = calculateMinimumDurationMs(
    axis,
    requestedPulseUs
  );
  uint32_t actualDurationMs = minimumDurationMs;
  if (hasRequestedTime && requestedTimeMs > actualDurationMs) {
    actualDurationMs = requestedTimeMs;
  }

  startPulseUs[axis] = currentPulseUs[axis];
  targetPulseUs[axis] = clampFloat(
    requestedPulseUs,
    SERVO_MIN_PULSE_US,
    SERVO_MAX_PULSE_US
  );
  axisFinalStepEnabled[axis] =
    actualDurationMs > SERVO_UPDATE_INTERVAL_MS;
  axisApproachWritten[axis] = false;

  if (axisFinalStepEnabled[axis]) {
    const int targetCount = pulseUsToPwmCount(targetPulseUs[axis]);
    const int direction =
      targetPulseUs[axis] >= startPulseUs[axis] ? 1 : -1;
    const int approachCount =
      targetCount - direction * SERVO_FINAL_STEP_COUNTS;
    const int minimumCount = pulseUsToPwmCount(SERVO_MIN_PULSE_US);
    const int maximumCount = pulseUsToPwmCount(SERVO_MAX_PULSE_US);

    if (approachCount < minimumCount || approachCount > maximumCount) {
      axisFinalStepEnabled[axis] = false;
    } else {
      approachPulseUs[axis] =
        pwmCountToPulseUs((uint16_t)approachCount);
    }
  }

  if (fabsf(targetPulseUs[axis] - startPulseUs[axis]) < 0.5f) {
    currentPulseUs[axis] = targetPulseUs[axis];
    axisMotionActive[axis] = false;
    axisFinalStepEnabled[axis] = false;
    writeServoPulse(axis, currentPulseUs[axis], true);
  } else {
    axisMotionStartMs[axis] = nowMs;
    axisMotionDurationMs[axis] = actualDurationMs;
    axisMotionActive[axis] = true;
  }

  lastMotionUpdateMs = nowMs;
}

bool anyAxisMoving() {
  for (size_t axis = 0; axis < SERVO_COUNT; ++axis) {
    if (axisMotionActive[axis]) {
      return true;
    }
  }
  return false;
}

void clearCommand(PendingCommand &command) {
  for (size_t axis = 0; axis < SERVO_COUNT; ++axis) {
    command.selected[axis] = false;
    command.angleMode[axis] = true;
    command.inputValue[axis] = 0.0f;
    command.targetPulseUs[axis] = 0.0f;
    command.hasRequestedTime[axis] = false;
    command.requestedTimeMs[axis] = 0;
    command.delayMs[axis] = 0;
  }
}

bool timeReached(uint32_t nowMs, uint32_t targetMs) {
  return (int32_t)(nowMs - targetMs) >= 0;
}

bool anyPendingAxisCommand() {
  for (size_t axis = 0; axis < SERVO_COUNT; ++axis) {
    if (pendingAxisCommand[axis].active) {
      return true;
    }
  }
  return false;
}

void updatePendingCommand(uint32_t nowMs) {
  for (size_t axis = 0; axis < SERVO_COUNT; ++axis) {
    if (!pendingAxisCommand[axis].active
        || !timeReached(nowMs, pendingAxisCommand[axis].executeAtMs)) {
      continue;
    }

    const PendingAxisCommand command = pendingAxisCommand[axis];
    pendingAxisCommand[axis].active = false;
    if (command.reportResetStart) {
      Serial.print("RESET motor");
      Serial.print(axis + 1);
      Serial.print(" angle:");
      Serial.println(SERVO_INITIAL_DEG[axis], 2);
    }
    executeAxisCommand(
      axis,
      command.targetPulseUs,
      command.hasRequestedTime,
      command.requestedTimeMs,
      nowMs
    );
    if (bootInitializationPending
        && pca9685Ready
        && axis == SERVO_COUNT - 1) {
      digitalWrite(PCA9685_OE_PIN, LOW);
    }
  }
}

void scheduleSequentialReset(uint32_t nowMs, bool reportStarts) {
  updateMotion(nowMs, true);

  for (size_t axis = 0; axis < SERVO_COUNT; ++axis) {
    axisMotionActive[axis] = false;
    axisFinalStepEnabled[axis] = false;

    pendingAxisCommand[axis].active = true;
    pendingAxisCommand[axis].reportResetStart = reportStarts;
    pendingAxisCommand[axis].targetPulseUs =
      angleToPulseUs(axis, SERVO_INITIAL_DEG[axis]);
    pendingAxisCommand[axis].hasRequestedTime = false;
    pendingAxisCommand[axis].requestedTimeMs = 0;
    pendingAxisCommand[axis].executeAtMs =
      nowMs + (SERVO_COUNT - 1 - axis) * SERVO_RESET_STAGGER_MS;
  }
}

void updateServoTest(uint32_t nowMs) {
  if (!servoTestEnabled || anyAxisMoving()
      || !timeReached(nowMs, nextTestMotionMs)) {
    return;
  }
  for (size_t axis = 0; axis < SERVO_COUNT; ++axis) {
    if (pendingAxisCommand[axis].active) {
      return;
    }
  }

  const float testAngle =
    testMovingToHigh ? SERVO_TEST_HIGH_DEG : SERVO_TEST_LOW_DEG;
  const float testPulseUs =
    angleToPulseUs(servoTestAxis, testAngle);

  Serial.print("TEST,SERVO,");
  Serial.print(servoTestAxis + 1);
  Serial.print(",ANGLE,");
  Serial.println(testAngle, 1);

  executeAxisCommand(
    servoTestAxis,
    testPulseUs,
    false,
    0,
    nowMs
  );
  testMovingToHigh = !testMovingToHigh;
  nextTestMotionMs =
    nowMs + axisMotionDurationMs[servoTestAxis] + SERVO_TEST_PAUSE_MS;
}

void sendError(const char *reason, const char *input) {
  Serial.print("ERR,");
  Serial.print(reason);
  Serial.print(',');

  if (strcmp(reason, "EMPTY") == 0) {
    Serial.print("제어할 모터가 없습니다. 예를 들어 #1A90 또는 #1P1500처럼 입력하세요.");
  } else if (strcmp(reason, "CHANNEL") == 0) {
    if (strncmp(input, "#0", 2) == 0) {
      Serial.print("확장보드 0번은 사용하지 않습니다. PCA9685의 1번 채널과 1번 모터를 연결하세요.");
    } else {
      Serial.print("모터 번호가 올바르지 않습니다. 현재 사용할 수 있는 모터 번호는 1번부터 ");
      Serial.print(SERVO_COUNT);
      Serial.print("번까지입니다.");
    }
  } else if (strcmp(reason, "MODE") == 0) {
    Serial.print("제어 방식은 각도 A 또는 펄스 P만 사용할 수 있습니다. 예를 들어 #1A90 또는 #1P1500처럼 입력하세요.");
  } else if (strcmp(reason, "VALUE") == 0) {
    Serial.print("각도 또는 펄스 값이 없거나 숫자가 아닙니다. A 뒤에는 각도를 P 뒤에는 500부터 2500 사이의 펄스를 입력하세요.");
  } else if (strcmp(reason, "TIME") == 0) {
    Serial.print("수행시간 T는 100ms부터 9999ms까지 입력해야 합니다. 예를 들어 #1A90T1000처럼 입력하세요.");
  } else if (strcmp(reason, "DELAY") == 0) {
    Serial.print("지연시간 D는 100ms부터 9999ms까지 입력해야 합니다. 예를 들어 #1A90D500처럼 입력하세요.");
  } else if (strcmp(reason, "FORMAT") == 0) {
    Serial.print("명령 형식이 올바르지 않습니다. 형식은 #모터번호A각도 또는 #모터번호P펄스이며 T와 D를 추가할 수 있습니다.");
  } else if (strcmp(reason, "TOO_LONG") == 0) {
    Serial.print("명령이 너무 깁니다. 줄바꿈 문자를 제외하고 최대 ");
    Serial.print(SERIAL_BUFFER_SIZE - 1);
    Serial.print("바이트까지 입력할 수 있습니다.");
  } else if (strcmp(reason, "TEST_AXIS") == 0) {
    if (strcmp(input, "#TEST0") == 0) {
      Serial.print("확장보드 0번은 사용하지 않습니다. PCA9685의 1번 채널과 1번 모터를 연결하세요.");
    } else {
      Serial.print("테스트할 모터 번호가 올바르지 않습니다. #TEST1부터 #TEST");
      Serial.print(SERVO_COUNT);
      Serial.print("까지 사용할 수 있습니다.");
    }
  } else if (strcmp(reason, "TEST_FORMAT") == 0) {
    Serial.print("테스트 모터 번호 앞에는 공백을 넣지 않습니다. 예를 들어 모터 5를 테스트하려면 #TEST5로 입력하세요.");
  } else if (strcmp(reason, "PCA9685_NOT_FOUND") == 0) {
    Serial.print("PCA9685를 찾지 못했습니다. 주소 0x40과 SDA GPIO8 SCL GPIO9 배선 및 전원을 확인하세요.");
  } else if (strcmp(reason, "OFFSET_EMPTY") == 0) {
    Serial.print("저장할 오프셋이 없습니다. 예를 들어 #OFFSET#1A-9.45#3A2.7처럼 입력하세요.");
  } else if (strcmp(reason, "OFFSET_CHANNEL") == 0) {
    Serial.print("오프셋을 설정할 모터 번호가 올바르지 않습니다. 모터 번호는 1번부터 ");
    Serial.print(SERVO_COUNT);
    Serial.print("번까지 사용할 수 있습니다.");
  } else if (strcmp(reason, "OFFSET_MODE") == 0) {
    Serial.print("오프셋 값 앞에는 각도를 뜻하는 A를 사용해야 합니다. 예를 들어 #OFFSET#1A-9.45처럼 입력하세요.");
  } else if (strcmp(reason, "OFFSET_VALUE") == 0) {
    Serial.print("오프셋이 숫자가 아니거나 허용 범위를 벗어났습니다. 각 오프셋은 ");
    Serial.print(SERVO_MIN_OFFSET_DEG, 1);
    Serial.print("도부터 ");
    Serial.print(SERVO_MAX_OFFSET_DEG, 1);
    Serial.print("도까지 입력하세요.");
  } else if (strcmp(reason, "OFFSET_FORMAT") == 0) {
    Serial.print("오프셋 명령 형식이 올바르지 않습니다. 형식은 #OFFSET#모터번호A오프셋이며 여러 모터를 연속해서 지정할 수 있습니다.");
  } else if (strcmp(reason, "OFFSET_SAVE") == 0) {
    Serial.print("오프셋을 ESP32 비휘발성 저장소에 기록하지 못했습니다. 저장 공간 상태를 확인한 뒤 다시 시도하세요.");
  } else if (strcmp(reason, "GOPOS_INDEX") == 0) {
    Serial.print("기본 포지션 번호가 올바르지 않습니다. #GOPOS1T1000 또는 #GOPOS2T1000처럼 입력하세요.");
  } else if (strcmp(reason, "GOPOS_TIME") == 0) {
    Serial.print("기본 포지션 이동시간은 T 뒤에 100ms부터 9999ms까지 입력해야 합니다. T를 생략하면 2000ms가 적용됩니다. 예를 들어 #GOPOS1 또는 #GOPOS1T1000처럼 입력하세요.");
  } else if (strcmp(reason, "GOPOS_FORMAT") == 0) {
    Serial.print("기본 포지션 명령 형식이 올바르지 않습니다. 예를 들어 #GOPOS1 또는 #GOPOS1T1000처럼 입력하세요.");
  } else {
    Serial.print("알 수 없는 오류가 발생했습니다.");
  }

  Serial.print(",INPUT=");
  Serial.println(input);
}

bool parseUnsignedInteger(
  const char *&cursor,
  uint32_t &value
) {
  if (*cursor < '0' || *cursor > '9') {
    return false;
  }

  char *end = nullptr;
  const unsigned long parsed = strtoul(cursor, &end, 10);
  if (end == cursor || parsed > UINT32_MAX) {
    return false;
  }
  cursor = end;
  value = (uint32_t)parsed;
  return true;
}

bool parseFloatValue(const char *&cursor, float &value) {
  char *end = nullptr;
  value = strtof(cursor, &end);
  if (end == cursor || !isfinite(value)) {
    return false;
  }
  cursor = end;
  return true;
}

bool parseOffsetCommand(
  const char *line,
  float offsets[SERVO_COUNT],
  const char *&errorReason
) {
  memcpy(offsets, servoOffsetDeg, sizeof(servoOffsetDeg));
  errorReason = "OFFSET_FORMAT";
  const char *cursor = line + strlen("#OFFSET");
  bool hasOffset = false;

  while (*cursor == '#') {
    ++cursor;

    uint32_t channel = 0;
    if (!parseUnsignedInteger(cursor, channel)
        || channel < 1 || channel > SERVO_COUNT) {
      errorReason = "OFFSET_CHANNEL";
      return false;
    }

    if (*cursor++ != 'A') {
      errorReason = "OFFSET_MODE";
      return false;
    }

    float offsetDeg = 0.0f;
    if (!parseFloatValue(cursor, offsetDeg)
        || offsetDeg < SERVO_MIN_OFFSET_DEG
        || offsetDeg > SERVO_MAX_OFFSET_DEG) {
      errorReason = "OFFSET_VALUE";
      return false;
    }

    offsets[channel - 1] = offsetDeg;
    hasOffset = true;
  }

  if (!hasOffset) {
    errorReason = "OFFSET_EMPTY";
    return false;
  }
  if (*cursor != '\0') {
    errorReason = "OFFSET_FORMAT";
    return false;
  }
  return true;
}

bool parseGotoPositionCommand(
  const char *line,
  PendingCommand &command,
  const char *&errorReason
) {
  clearCommand(command);
  const char *cursor = line + strlen("#GOPOS");

  uint32_t positionNumber = 0;
  if (!parseUnsignedInteger(cursor, positionNumber)
      || positionNumber < 1
      || positionNumber > SERVO_POSITION_COUNT) {
    errorReason = "GOPOS_INDEX";
    return false;
  }

  uint32_t requestedTimeMs = SERVO_POSITION_DEFAULT_TIME_MS;
  if (*cursor == 'T') {
    ++cursor;
    if (!parseUnsignedInteger(cursor, requestedTimeMs)
        || requestedTimeMs < 100
        || requestedTimeMs > 9999) {
      errorReason = "GOPOS_TIME";
      return false;
    }
  } else if (*cursor != '\0') {
    errorReason = "GOPOS_FORMAT";
    return false;
  }
  if (*cursor != '\0') {
    errorReason = "GOPOS_FORMAT";
    return false;
  }

  const size_t positionIndex = (size_t)(positionNumber - 1);
  for (size_t axis = 0; axis < SERVO_COUNT; ++axis) {
    const float angleDeg = SERVO_POSITIONS[positionIndex][axis];
    command.selected[axis] = true;
    command.angleMode[axis] = true;
    command.inputValue[axis] = angleDeg;
    command.targetPulseUs[axis] = angleToPulseUs(axis, angleDeg);
    command.hasRequestedTime[axis] = true;
    command.requestedTimeMs[axis] = requestedTimeMs;
    command.delayMs[axis] = 0;
  }
  return true;
}

bool parseCommandLine(
  const char *line,
  PendingCommand &command,
  const char *&errorReason
) {
  clearCommand(command);
  errorReason = "FORMAT";
  const char *cursor = line;
  bool hasChannel = false;

  while (*cursor == '#') {
    ++cursor;

    uint32_t channel = 0;
    if (!parseUnsignedInteger(cursor, channel)
        || channel < 1 || channel > SERVO_COUNT) {
      errorReason = "CHANNEL";
      return false;
    }

    const char mode = *cursor++;
    if (mode != 'A' && mode != 'P') {
      errorReason = "MODE";
      return false;
    }

    float value = 0.0f;
    if (!parseFloatValue(cursor, value)) {
      errorReason = "VALUE";
      return false;
    }

    const size_t axis = (size_t)(channel - 1);
    command.selected[axis] = true;
    command.angleMode[axis] = mode == 'A';
    command.inputValue[axis] = value;
    command.targetPulseUs[axis] =
      mode == 'A'
        ? angleToPulseUs(axis, value)
        : clampFloat(value, SERVO_MIN_PULSE_US, SERVO_MAX_PULSE_US);

    command.hasRequestedTime[axis] = false;
    command.requestedTimeMs[axis] = 0;
    command.delayMs[axis] = 0;

    if (*cursor == 'T') {
      ++cursor;
      if (!parseUnsignedInteger(cursor, command.requestedTimeMs[axis])
          || command.requestedTimeMs[axis] < 100
          || command.requestedTimeMs[axis] > 9999) {
        errorReason = "TIME";
        return false;
      }
      command.hasRequestedTime[axis] = true;
    }

    if (*cursor == 'D') {
      ++cursor;
      if (!parseUnsignedInteger(cursor, command.delayMs[axis])
          || command.delayMs[axis] < 100
          || command.delayMs[axis] > 9999) {
        errorReason = "DELAY";
        return false;
      }
    }

    hasChannel = true;
  }

  if (!hasChannel) {
    errorReason = "EMPTY";
    return false;
  }

  if (*cursor != '\0') {
    errorReason = "FORMAT";
    return false;
  }

  return true;
}

void reportAcceptedCommand(
  const PendingCommand &command,
  const uint32_t durationMs[SERVO_COUNT]
) {
  bool first = true;
  for (size_t axis = 0; axis < SERVO_COUNT; ++axis) {
    if (!command.selected[axis]) {
      continue;
    }

    if (!first) {
      Serial.print('/');
    }
    first = false;

    Serial.print("motor");
    Serial.print(axis + 1);
    Serial.print(command.angleMode[axis] ? ":angle " : ":pulse ");
    Serial.print(command.inputValue[axis], 2);
    if (command.angleMode[axis]) {
      const uint16_t targetCount =
        pulseUsToPwmCount(command.targetPulseUs[axis]);
      Serial.print(",pulse ");
      Serial.print(pwmCountToPulseUs(targetCount), 2);
      Serial.print("us,count ");
      Serial.print(targetCount);
    } else {
      Serial.print("us");
    }
    Serial.print(",duration ");
    Serial.print((float)durationMs[axis] / 1000.0f, 3);
    Serial.print('s');
    if (command.delayMs[axis] > 0) {
      Serial.print(",delay ");
      Serial.print((float)command.delayMs[axis] / 1000.0f, 3);
      Serial.print('s');
    }
  }
  Serial.println();
}

void processCommand(const char *line) {
  if (strcmp(line, "#OFFSET") == 0) {
    Serial.println("OK");
    reportServoOffsets();
    return;
  }

  if (strncmp(line, "#OFFSET", strlen("#OFFSET")) == 0) {
    float updatedOffsets[SERVO_COUNT];
    const char *errorReason = nullptr;
    if (!parseOffsetCommand(line, updatedOffsets, errorReason)) {
      sendError(errorReason, line);
      return;
    }
    if (!saveServoOffsets(updatedOffsets)) {
      sendError("OFFSET_SAVE", line);
      return;
    }

    memcpy(servoOffsetDeg, updatedOffsets, sizeof(servoOffsetDeg));
    Serial.println("OK");
    reportServoOffsets();
    return;
  }

  if (strcmp(line, "#TEST") == 0
      || strcmp(line, "#TEST ON") == 0
      || strcmp(line, "#TEST OFF") == 0) {
    if (!pca9685Ready) {
      sendError("PCA9685_NOT_FOUND", line);
      return;
    }

    const bool enableTest =
      strcmp(line, "#TEST ON") == 0
        ? true
        : strcmp(line, "#TEST OFF") == 0
          ? false
          : !servoTestEnabled;

    servoTestEnabled = enableTest;
    if (servoTestEnabled) {
      testMovingToHigh = true;
      nextTestMotionMs = millis() + SERVO_TEST_START_DELAY_MS;
      Serial.print("TEST,ENABLED,SERVO,");
      Serial.println(servoTestAxis + 1);
    } else {
      Serial.println("TEST,DISABLED");
    }
    Serial.println("OK");
    return;
  }

  if (strncmp(line, "#TEST", 5) == 0 && line[5] >= '0' && line[5] <= '9') {
    const char *cursor = line + 5;
    uint32_t requestedMotor = 0;
    if (!parseUnsignedInteger(cursor, requestedMotor)
        || *cursor != '\0'
        || requestedMotor < 1
        || requestedMotor > SERVO_COUNT) {
      sendError("TEST_AXIS", line);
      return;
    }
    if (!pca9685Ready) {
      sendError("PCA9685_NOT_FOUND", line);
      return;
    }

    servoTestAxis = (size_t)(requestedMotor - 1);
    servoTestEnabled = true;
    testMovingToHigh = true;
    nextTestMotionMs = millis() + SERVO_TEST_START_DELAY_MS;
    Serial.print("TEST,ENABLED,SERVO,");
    Serial.println(requestedMotor);
    Serial.println("OK");
    return;
  }

  if (strncmp(line, "#TEST", 5) == 0) {
    sendError("TEST_FORMAT", line);
    return;
  }

  if (strcmp(line, "#RESET") == 0) {
    if (!pca9685Ready) {
      sendError("PCA9685_NOT_FOUND", line);
      return;
    }

    startPwmDebug(PWM_DEBUG_RESET);
    scheduleSequentialReset(millis(), true);
    Serial.println("OK");
    return;
  }

  PendingCommand command;
  const char *errorReason = nullptr;

  const bool gotoPositionCommand =
    strncmp(line, "#GOPOS", strlen("#GOPOS")) == 0;
  const bool parsed = gotoPositionCommand
    ? parseGotoPositionCommand(line, command, errorReason)
    : parseCommandLine(line, command, errorReason);
  if (!parsed) {
    sendError(errorReason, line);
    return;
  }
  if (!pca9685Ready) {
    sendError("PCA9685_NOT_FOUND", line);
    return;
  }

  const uint32_t nowMs = millis();
  updateMotion(nowMs, true);
  uint32_t acceptedDurationMs[SERVO_COUNT] = {};

  for (size_t axis = 0; axis < SERVO_COUNT; ++axis) {
    if (!command.selected[axis]) {
      continue;
    }

    acceptedDurationMs[axis] = calculateMinimumDurationMs(
      axis,
      command.targetPulseUs[axis]
    );
    if (command.hasRequestedTime[axis]
        && command.requestedTimeMs[axis] > acceptedDurationMs[axis]) {
      acceptedDurationMs[axis] = command.requestedTimeMs[axis];
    }

    pendingAxisCommand[axis].active = false;

    if (command.delayMs[axis] > 0) {
      pendingAxisCommand[axis].active = true;
      pendingAxisCommand[axis].reportResetStart = false;
      pendingAxisCommand[axis].targetPulseUs =
        command.targetPulseUs[axis];
      pendingAxisCommand[axis].hasRequestedTime =
        command.hasRequestedTime[axis];
      pendingAxisCommand[axis].requestedTimeMs =
        command.requestedTimeMs[axis];
      pendingAxisCommand[axis].executeAtMs =
        nowMs + command.delayMs[axis];
    } else {
      executeAxisCommand(
        axis,
        command.targetPulseUs[axis],
        command.hasRequestedTime[axis],
        command.requestedTimeMs[axis],
        nowMs
      );
    }
  }

  Serial.println("OK");
  reportAcceptedCommand(command, acceptedDurationMs);
}

void readSerialCommands() {
  while (Serial.available() > 0) {
    const char incoming = (char)Serial.read();

    if (incoming == '\n') {
      if (serialOverflow) {
        serialBuffer[SERIAL_BUFFER_SIZE - 1] = '\0';
        sendError("TOO_LONG", serialBuffer);
      } else if (serialLength > 0) {
        serialBuffer[serialLength] = '\0';
        processCommand(serialBuffer);
      }

      serialLength = 0;
      serialOverflow = false;
      continue;
    }

    if (incoming == '\r') {
      continue;
    }

    if (serialOverflow) {
      continue;
    }

    if (serialLength < SERIAL_BUFFER_SIZE - 1) {
      serialBuffer[serialLength++] = incoming;
    } else {
      serialOverflow = true;
    }
  }
}

void setup() {
  digitalWrite(PCA9685_OE_PIN, HIGH);
  pinMode(PCA9685_OE_PIN, OUTPUT);

  Serial.begin(SERIAL_BAUD_RATE);
  const uint32_t serialWaitStartMs = millis();
  while (!Serial && millis() - serialWaitStartMs < 2000) {
    delay(10);
  }
  Serial.println("BOOT");
  startPwmDebug(PWM_DEBUG_BOOT);

  loadServoOffsets();

  Wire.begin(PCA9685_SDA_PIN, PCA9685_SCL_PIN);
  Wire.setClock(PCA9685_I2C_FREQUENCY_HZ);
  pca9685Ready = pwm.begin();
  if (pca9685Ready) {
    pwm.setOscillatorFrequency(PCA9685_OSCILLATOR_HZ);
    pwm.setPWMFreq(PCA9685_PWM_FREQUENCY_HZ);
  } else {
    sendError("PCA9685_NOT_FOUND", "BOOT");
  }

  for (size_t axis = 0; axis < SERVO_COUNT; ++axis) {
    const float initialPulseUs =
      angleToPulseUs(axis, SERVO_INITIAL_DEG[axis]);
    currentPulseUs[axis] = initialPulseUs;
    startPulseUs[axis] = initialPulseUs;
    targetPulseUs[axis] = initialPulseUs;
    approachPulseUs[axis] = initialPulseUs;
    axisMotionActive[axis] = false;
    axisFinalStepEnabled[axis] = false;
    axisApproachWritten[axis] = false;
    axisMotionStartMs[axis] = 0;
    axisMotionDurationMs[axis] = 0;
    lastWrittenPwmCount[axis] = -1;
    if (pca9685Ready) {
      setServoPwm(axis, 0, 0);
    }
  }

  bootInitializationPending = true;
  scheduleSequentialReset(millis(), false);
}

void loop() {
  if (bootInitializationPending) {
    const uint32_t nowMs = millis();
    updatePendingCommand(nowMs);
    if (!anyPendingAxisCommand()) {
      bootInitializationPending = false;
      stopPwmDebug();
      Serial.println("READY");
      if (servoTestEnabled) {
        Serial.print("TEST,ENABLED,SERVO,");
        Serial.println(servoTestAxis + 1);
        nextTestMotionMs = nowMs + SERVO_TEST_START_DELAY_MS;
      }
    }
    return;
  }

  readSerialCommands();
  const uint32_t nowMs = millis();
  updateMotion(nowMs, false);
  updatePendingCommand(nowMs);
  if (pwmDebugPhase == PWM_DEBUG_RESET
      && !anyPendingAxisCommand()
      && !anyAxisMoving()) {
    stopPwmDebug();
  }
  updateServoTest(nowMs);
}
