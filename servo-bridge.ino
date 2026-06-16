#include <Arduino.h>
#include <Preferences.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>

#if !defined(ARDUINO_USB_CDC_ON_BOOT) || !ARDUINO_USB_CDC_ON_BOOT
#error "Enable Tools > USB CDC On Boot, or select Nologo ESP32C3 Super Mini."
#endif

#define SERVO_COUNT 6
#define MAX_EASING_SERVOS SERVO_COUNT
#include <ServoEasing.hpp>

#define SERVO_POSITION_COUNT 2
#define SERVO_POSITION_DEFAULT_TIME_MS 1500
#define SERIAL_BAUD_RATE 115200
#define SERVO_UPDATE_INTERVAL_MS 20
#define SERVO_SPEED_DEG_PER_SEC 120.0f
#define SERVO_DEADZONE_US 1.0f
#define SERVO_RESET_STAGGER_MS 800
#define SERVO_RESET_MOVE_TIME_MS 2000
#define SERIAL_BUFFER_SIZE 160
#define PWM_INITIALIZATION_DEBUG false

#define SERVO_TEST_MODE false
#define SERVO_TEST_AXIS 1
#define SERVO_TEST_LOW_DEG 45.0f
#define SERVO_TEST_HIGH_DEG 135.0f
#define SERVO_TEST_START_DELAY_MS 1500
#define SERVO_TEST_PAUSE_MS 500

#define SERVO_DIR_NORMAL 1
#define SERVO_DIR_REVERSED -1

const uint16_t SERVO_PWM_FREQUENCY_HZ = 50;
const uint8_t SERVO_LEDC_RESOLUTION_BITS = 16;
const uint8_t SERVO_PINS[SERVO_COUNT] = {6, 7, 9, 10, 20, 21};
constexpr uint8_t SERVO_RESET_ORDER[SERVO_COUNT] = {2, 3, 4, 5, 6, 1};

const char OFFSET_NVS_NAMESPACE[] = "servo-bridge";
const char OFFSET_NVS_KEY[] = "offsets";
const float SERVO_MIN_OFFSET_DEG = -180.0f;
const float SERVO_MAX_OFFSET_DEG = 180.0f;

static_assert(SERVO_COUNT <= MAX_EASING_SERVOS, "MAX_EASING_SERVOS is too small.");
static_assert(
  SERVO_TEST_AXIS >= 1 && SERVO_TEST_AXIS <= SERVO_COUNT,
  "SERVO_TEST_AXIS must be a motor number from 1 to SERVO_COUNT."
);
static_assert(
  sizeof(SERVO_RESET_ORDER) / sizeof(SERVO_RESET_ORDER[0]) == SERVO_COUNT,
  "SERVO_RESET_ORDER must define one motor number per servo."
);
static_assert(
  SERVO_RESET_ORDER[0] == 2
    && SERVO_RESET_ORDER[1] == 3
    && SERVO_RESET_ORDER[2] == 4
    && SERVO_RESET_ORDER[3] == 5
    && SERVO_RESET_ORDER[4] == 6
    && SERVO_RESET_ORDER[5] == 1,
  "SERVO_RESET_ORDER must be 2,3,4,5,6,1."
);

const float SERVO_TRAVEL_DEG[SERVO_COUNT] = {
  270.0f, 270.0f, 270.0f, 270.0f, 180.0f, 180.0f
};

const float SERVO_INITIAL_DEG[SERVO_COUNT] = {
  90.0f, 90.0f, 90.0f, 90.0f, 90.0f, 90.0f
};

const float SERVO_POSITIONS[SERVO_POSITION_COUNT][SERVO_COUNT] = {
  {90.0f, 70.0f, 160.0f, 0.0f, 90.0f, 130.0f},
  {45.0f, 45.0f, 45.0f, 45.0f, 45.0f, 45.0f}
};

const int8_t SERVO_DIRECTION[SERVO_COUNT] = {
  SERVO_DIR_REVERSED, SERVO_DIR_REVERSED, SERVO_DIR_NORMAL,
  SERVO_DIR_REVERSED, SERVO_DIR_NORMAL, SERVO_DIR_NORMAL
};

const float DEFAULT_SERVO_OFFSET_DEG[SERVO_COUNT] = {
  0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f
};
float servoOffsetDeg[SERVO_COUNT];

const float SERVO_MIN_DEG[SERVO_COUNT] = {
  0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 70.0f
};

const float SERVO_MAX_DEG[SERVO_COUNT] = {
  180.0f, 180.0f, 180.0f, 180.0f, 180.0f, 150.0f
};

const float SERVO_MIN_PULSE_US = 500.0f;
const float SERVO_MAX_PULSE_US = 2500.0f;
const float SERVO_PULSE_RANGE_US =
  SERVO_MAX_PULSE_US - SERVO_MIN_PULSE_US;

ServoEasing servoOutputs[SERVO_COUNT];
bool servoOutputReady = false;
bool servoAttached[SERVO_COUNT] = {};

enum PwmDebugPhase : uint8_t {
  PWM_DEBUG_NONE,
  PWM_DEBUG_BOOT,
  PWM_DEBUG_RESET
};
PwmDebugPhase pwmDebugPhase = PWM_DEBUG_NONE;
uint32_t pwmDebugWriteCount = 0;

float currentPulseUs[SERVO_COUNT];
float targetPulseUs[SERVO_COUNT];
float lastWrittenPulseUs[SERVO_COUNT];

bool axisMotionActive[SERVO_COUNT];
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

float applyDirectionFromAngle(size_t axis, float directedAngleDeg) {
  if (SERVO_DIRECTION[axis] == SERVO_DIR_REVERSED) {
    return 180.0f - directedAngleDeg;
  }
  return directedAngleDeg;
}

float logicalToPhysicalAngle(size_t axis, float logicalAngleDeg) {
  const float clampedLogical =
    clampFloat(logicalAngleDeg, SERVO_MIN_DEG[axis], SERVO_MAX_DEG[axis]);
  const float directed = applyDirectionToAngle(axis, clampedLogical);
  const float offsetApplied = directed + servoOffsetDeg[axis];
  const float extraTravelDeg =
    (SERVO_TRAVEL_DEG[axis] - 180.0f) * 0.5f;

  return clampFloat(
    offsetApplied + extraTravelDeg,
    0.0f,
    SERVO_TRAVEL_DEG[axis]
  );
}

float physicalToLogicalAngle(size_t axis, float physicalAngleDeg) {
  const float extraTravelDeg =
    (SERVO_TRAVEL_DEG[axis] - 180.0f) * 0.5f;
  const float directed =
    physicalAngleDeg - extraTravelDeg - servoOffsetDeg[axis];
  return clampFloat(
    applyDirectionFromAngle(axis, directed),
    SERVO_MIN_DEG[axis],
    SERVO_MAX_DEG[axis]
  );
}

float physicalAngleToPulseUs(size_t axis, float physicalAngleDeg) {
  const float pulse =
    SERVO_MIN_PULSE_US
      + (physicalAngleDeg / SERVO_TRAVEL_DEG[axis]) * SERVO_PULSE_RANGE_US;
  return clampFloat(pulse, SERVO_MIN_PULSE_US, SERVO_MAX_PULSE_US);
}

float pulseToPhysicalAngle(size_t axis, float pulseUs) {
  const float clampedPulse =
    clampFloat(pulseUs, SERVO_MIN_PULSE_US, SERVO_MAX_PULSE_US);
  return (clampedPulse - SERVO_MIN_PULSE_US)
    * SERVO_TRAVEL_DEG[axis] / SERVO_PULSE_RANGE_US;
}

float angleToPulseUs(size_t axis, float logicalAngleDeg) {
  return physicalAngleToPulseUs(
    axis,
    logicalToPhysicalAngle(axis, logicalAngleDeg)
  );
}

float pulseToAngleDeg(size_t axis, float pulseUs) {
  return physicalToLogicalAngle(axis, pulseToPhysicalAngle(axis, pulseUs));
}

float pulseDeltaToDegrees(size_t axis, float pulseDeltaUs) {
  return fabsf(pulseDeltaUs) * SERVO_TRAVEL_DEG[axis]
    / SERVO_PULSE_RANGE_US;
}

int pulseUsToOutputUs(float pulseUs) {
  return (int)clampFloat(
    lroundf(pulseUs),
    SERVO_MIN_PULSE_US,
    SERVO_MAX_PULSE_US
  );
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

void printPwmDebug(size_t axis, int pulseUs, const char *action) {
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
  Serial.print(",pin=");
  Serial.print(SERVO_PINS[axis]);
  Serial.print(",action=");
  Serial.print(action);
  Serial.print(",pulse=");
  Serial.print(pulseUs);
  Serial.println("us");
}

bool attachServoOutput(size_t axis, int initialPulseUs) {
  if (servoAttached[axis]) {
    return true;
  }

  servoOutputs[axis].setPeriodHertz(SERVO_PWM_FREQUENCY_HZ);
  servoOutputs[axis].setTimerWidth(SERVO_LEDC_RESOLUTION_BITS);
  if (servoOutputs[axis].readTimerWidth() != SERVO_LEDC_RESOLUTION_BITS) {
    return false;
  }

  const uint8_t result = servoOutputs[axis].attach(
    SERVO_PINS[axis],
    initialPulseUs,
    (int)SERVO_MIN_PULSE_US,
    (int)SERVO_MAX_PULSE_US
  );
  if (result == INVALID_SERVO) {
    return false;
  }

  servoOutputs[axis].setEasingType(EASE_SINE_IN_OUT);
  servoAttached[axis] = true;
  lastWrittenPulseUs[axis] = initialPulseUs;
  printPwmDebug(axis, initialPulseUs, "ATTACH");
  return true;
}

void detachServoOutput(size_t axis) {
  if (!servoAttached[axis]) {
    return;
  }

  servoOutputs[axis].detach();
  servoAttached[axis] = false;
  printPwmDebug(axis, pulseUsToOutputUs(lastWrittenPulseUs[axis]), "DETACH");
  lastWrittenPulseUs[axis] = -1.0f;
}

void writeServoPulse(size_t axis, float logicalPulseUs, bool forceWrite) {
  if (!servoOutputReady) {
    return;
  }

  const int outputPulseUs = pulseUsToOutputUs(logicalPulseUs);
  if (!attachServoOutput(axis, outputPulseUs)) {
    servoOutputReady = false;
    return;
  }

  const float changeUs = fabsf(outputPulseUs - lastWrittenPulseUs[axis]);
  if (forceWrite || changeUs >= SERVO_DEADZONE_US) {
    servoOutputs[axis].writeMicroseconds(outputPulseUs);
    lastWrittenPulseUs[axis] = outputPulseUs;
    printPwmDebug(axis, outputPulseUs, "WRITE");
  }
}

bool startServoEase(size_t axis, float targetPulse, uint32_t durationMs) {
  const int outputPulseUs = pulseUsToOutputUs(targetPulse);
  if (!attachServoOutput(axis, pulseUsToOutputUs(currentPulseUs[axis]))) {
    servoOutputReady = false;
    return false;
  }

  servoOutputs[axis].setEasingType(EASE_SINE_IN_OUT);
  servoOutputs[axis].startEaseToD(
    (unsigned int)outputPulseUs,
    (uint_fast16_t)durationMs,
    DO_NOT_START_UPDATE_BY_INTERRUPT
  );
  lastWrittenPulseUs[axis] = outputPulseUs;
  printPwmDebug(axis, outputPulseUs, "EASE");
  return true;
}

void reportCurrentAngles() {
  Serial.print("STATE");
  for (size_t axis = 0; axis < SERVO_COUNT; ++axis) {
    Serial.print(',');
    Serial.print(pulseToAngleDeg(axis, currentPulseUs[axis]), 1);
    Serial.print('/');
    Serial.print(currentPulseUs[axis], 0);
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

    if (servoAttached[axis]) {
      servoOutputs[axis].update();
      currentPulseUs[axis] = servoOutputs[axis].readMicroseconds();
    }

    const bool finished =
      (uint32_t)(nowMs - axisMotionStartMs[axis])
        >= axisMotionDurationMs[axis]
      || !servoOutputs[axis].isMoving();

    if (finished) {
      currentPulseUs[axis] = targetPulseUs[axis];
      writeServoPulse(axis, currentPulseUs[axis], true);
      axisMotionActive[axis] = false;
    }

    anyMotionUpdated = true;
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

uint32_t calculateAcceptedDurationMs(
  size_t axis,
  float requestedPulseUs,
  bool hasRequestedTime,
  uint32_t requestedTimeMs
) {
  const uint32_t minimumDurationMs =
    calculateMinimumDurationMs(axis, requestedPulseUs);
  return hasRequestedTime && requestedTimeMs > minimumDurationMs
    ? requestedTimeMs
    : minimumDurationMs;
}

void executeAxisCommand(
  size_t axis,
  float requestedPulseUs,
  bool hasRequestedTime,
  uint32_t requestedTimeMs,
  uint32_t nowMs
) {
  updateMotion(nowMs, true);

  const uint32_t actualDurationMs = calculateAcceptedDurationMs(
    axis,
    requestedPulseUs,
    hasRequestedTime,
    requestedTimeMs
  );

  targetPulseUs[axis] = clampFloat(
    requestedPulseUs,
    SERVO_MIN_PULSE_US,
    SERVO_MAX_PULSE_US
  );

  if (fabsf(targetPulseUs[axis] - currentPulseUs[axis]) < 0.5f
      || actualDurationMs == 0) {
    currentPulseUs[axis] = targetPulseUs[axis];
    axisMotionActive[axis] = false;
    writeServoPulse(axis, currentPulseUs[axis], true);
  } else if (startServoEase(axis, targetPulseUs[axis], actualDurationMs)) {
    axisMotionStartMs[axis] = nowMs;
    axisMotionDurationMs[axis] = actualDurationMs;
    axisMotionActive[axis] = true;
  } else {
    currentPulseUs[axis] = targetPulseUs[axis];
    axisMotionActive[axis] = false;
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
  }
}

void scheduleSequentialReset(uint32_t nowMs, bool reportStarts) {
  updateMotion(nowMs, true);

  for (size_t axis = 0; axis < SERVO_COUNT; ++axis) {
    axisMotionActive[axis] = false;
    pendingAxisCommand[axis].active = false;
  }

  for (size_t orderIndex = 0; orderIndex < SERVO_COUNT; ++orderIndex) {
    const uint8_t motorNumber = SERVO_RESET_ORDER[orderIndex];
    if (motorNumber < 1 || motorNumber > SERVO_COUNT) {
      continue;
    }
    const size_t axis = motorNumber - 1;

    pendingAxisCommand[axis].active = true;
    pendingAxisCommand[axis].reportResetStart = reportStarts;
    pendingAxisCommand[axis].targetPulseUs =
      angleToPulseUs(axis, SERVO_INITIAL_DEG[axis]);
    pendingAxisCommand[axis].hasRequestedTime = reportStarts;
    pendingAxisCommand[axis].requestedTimeMs =
      reportStarts ? SERVO_RESET_MOVE_TIME_MS : 0;
    pendingAxisCommand[axis].executeAtMs =
      nowMs + orderIndex * SERVO_RESET_STAGGER_MS;
  }
}

void beginBootSoftAlign(uint32_t nowMs) {
  bootInitializationPending = true;
  scheduleSequentialReset(nowMs, false);
}

bool updateBootSoftAlign(uint32_t nowMs) {
  updatePendingCommand(nowMs);
  updateMotion(nowMs, false);
  return !anyPendingAxisCommand() && !anyAxisMoving();
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

void startServoTest(size_t axis) {
  servoTestAxis = axis;
  servoTestEnabled = true;
  testMovingToHigh = true;
  nextTestMotionMs = millis() + SERVO_TEST_START_DELAY_MS;
  Serial.print("TEST,ENABLED,SERVO,");
  Serial.println(servoTestAxis + 1);
}

void sendError(const char *reason, const char *input) {
  Serial.print("ERR,");
  Serial.print(reason);
  Serial.print(',');

  if (strcmp(reason, "EMPTY") == 0) {
    Serial.print("제어할 모터가 없습니다. 예: #1A90 또는 #1P1500");
  } else if (strcmp(reason, "CHANNEL") == 0) {
    Serial.print("모터 번호는 1부터 ");
    Serial.print(SERVO_COUNT);
    Serial.print("까지 사용할 수 있습니다.");
  } else if (strcmp(reason, "MODE") == 0) {
    Serial.print("제어 방식은 각도 A 또는 펄스 P만 사용할 수 있습니다.");
  } else if (strcmp(reason, "VALUE") == 0) {
    Serial.print("각도 또는 펄스 값이 올바르지 않습니다.");
  } else if (strcmp(reason, "TIME") == 0) {
    Serial.print("이동시간 T는 100ms부터 9999ms까지 입력해야 합니다.");
  } else if (strcmp(reason, "DELAY") == 0) {
    Serial.print("지연시간 D는 100ms부터 9999ms까지 입력해야 합니다.");
  } else if (strcmp(reason, "FORMAT") == 0) {
    Serial.print("명령 형식이 올바르지 않습니다.");
  } else if (strcmp(reason, "TOO_LONG") == 0) {
    Serial.print("명령이 너무 깁니다.");
  } else if (strcmp(reason, "TEST_AXIS") == 0) {
    Serial.print("#TEST1부터 #TEST");
    Serial.print(SERVO_COUNT);
    Serial.print("까지 사용할 수 있습니다.");
  } else if (strcmp(reason, "TEST_FORMAT") == 0) {
    Serial.print("테스트 모터 번호 앞에 공백을 넣지 마세요. 예: #TEST5");
  } else if (strcmp(reason, "SERVO_OUTPUT_INIT") == 0) {
    Serial.print("ESP32 서보 출력 초기화에 실패했습니다. 50Hz 16-bit LEDC 지원과 GPIO 핀을 확인하세요.");
  } else if (strcmp(reason, "OFFSET_EMPTY") == 0) {
    Serial.print("저장할 오프셋이 없습니다.");
  } else if (strcmp(reason, "OFFSET_CHANNEL") == 0) {
    Serial.print("오프셋 모터 번호가 올바르지 않습니다.");
  } else if (strcmp(reason, "OFFSET_MODE") == 0) {
    Serial.print("오프셋 값 앞에는 A를 사용해야 합니다.");
  } else if (strcmp(reason, "OFFSET_VALUE") == 0) {
    Serial.print("오프셋 값은 -180도부터 180도까지입니다.");
  } else if (strcmp(reason, "OFFSET_FORMAT") == 0) {
    Serial.print("오프셋 명령 형식이 올바르지 않습니다.");
  } else if (strcmp(reason, "OFFSET_SAVE") == 0) {
    Serial.print("오프셋을 NVS에 저장하지 못했습니다.");
  } else if (strcmp(reason, "OFFSET_RESET") == 0) {
    Serial.print("오프셋 저장값을 삭제하지 못했습니다.");
  } else if (strcmp(reason, "GOPOS_INDEX") == 0) {
    Serial.print("기본 포지션 번호가 올바르지 않습니다.");
  } else if (strcmp(reason, "GOPOS_TIME") == 0) {
    Serial.print("기본 포지션 이동시간은 100ms부터 9999ms까지입니다.");
  } else if (strcmp(reason, "GOPOS_FORMAT") == 0) {
    Serial.print("기본 포지션 명령 형식이 올바르지 않습니다.");
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

void loadServoOffsets() {
  memcpy(servoOffsetDeg, DEFAULT_SERVO_OFFSET_DEG, sizeof(servoOffsetDeg));

  Preferences preferences;
  if (!preferences.begin(OFFSET_NVS_NAMESPACE, true)) {
    return;
  }

  const size_t storedSize = preferences.getBytesLength(OFFSET_NVS_KEY);
  if (storedSize > 0 && storedSize % sizeof(float) == 0) {
    const size_t storedOffsetCount = storedSize / sizeof(float);
    const size_t appliedOffsetCount =
      storedOffsetCount < SERVO_COUNT ? storedOffsetCount : SERVO_COUNT;
    float storedOffsets[SERVO_COUNT];
    bool valid = appliedOffsetCount > 0;

    if (valid) {
      const size_t bytesToRead = appliedOffsetCount * sizeof(float);
      const size_t bytesRead = preferences.getBytes(
        OFFSET_NVS_KEY,
        storedOffsets,
        bytesToRead
      );
      valid = bytesRead == bytesToRead;
    }

    for (size_t axis = 0; valid && axis < appliedOffsetCount; ++axis) {
      valid = isfinite(storedOffsets[axis])
        && storedOffsets[axis] >= SERVO_MIN_OFFSET_DEG
        && storedOffsets[axis] <= SERVO_MAX_OFFSET_DEG;
    }

    if (valid) {
      memcpy(
        servoOffsetDeg,
        storedOffsets,
        appliedOffsetCount * sizeof(float)
      );
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
    sizeof(float) * SERVO_COUNT
  );
  preferences.end();
  return bytesWritten == sizeof(float) * SERVO_COUNT;
}

bool resetServoOffsets() {
  Preferences preferences;
  if (!preferences.begin(OFFSET_NVS_NAMESPACE, false)) {
    return false;
  }

  const bool hasStoredOffsets = preferences.getBytesLength(OFFSET_NVS_KEY) > 0;
  const bool removed = !hasStoredOffsets || preferences.remove(OFFSET_NVS_KEY);
  preferences.end();
  if (removed) {
    memcpy(servoOffsetDeg, DEFAULT_SERVO_OFFSET_DEG, sizeof(servoOffsetDeg));
  }
  return removed;
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

bool parseOffsetCommand(
  const char *line,
  float offsets[SERVO_COUNT],
  const char *&errorReason
) {
  memcpy(offsets, servoOffsetDeg, sizeof(float) * SERVO_COUNT);
  const char *cursor = line + strlen("#OFFSET");
  if (*cursor == '\0') {
    errorReason = "OFFSET_EMPTY";
    return false;
  }

  while (*cursor != '\0') {
    if (*cursor != '#') {
      errorReason = "OFFSET_FORMAT";
      return false;
    }
    ++cursor;

    uint32_t motor = 0;
    if (!parseUnsignedInteger(cursor, motor)
        || motor < 1
        || motor > SERVO_COUNT) {
      errorReason = "OFFSET_CHANNEL";
      return false;
    }

    if (*cursor != 'A') {
      errorReason = "OFFSET_MODE";
      return false;
    }
    ++cursor;

    float offset = 0.0f;
    if (!parseFloatValue(cursor, offset)
        || offset < SERVO_MIN_OFFSET_DEG
        || offset > SERVO_MAX_OFFSET_DEG) {
      errorReason = "OFFSET_VALUE";
      return false;
    }

    offsets[motor - 1] = offset;
  }

  return true;
}

bool parseTimingSuffix(
  const char *&cursor,
  PendingCommand &command,
  size_t axis,
  const char *&errorReason
) {
  while (*cursor == 'T' || *cursor == 'D') {
    const char mode = *cursor++;
    uint32_t value = 0;
    if (!parseUnsignedInteger(cursor, value)
        || value < 100
        || value > 9999) {
      errorReason = mode == 'T' ? "TIME" : "DELAY";
      return false;
    }

    if (mode == 'T') {
      command.hasRequestedTime[axis] = true;
      command.requestedTimeMs[axis] = value;
    } else {
      command.delayMs[axis] = value;
    }
  }
  return true;
}

bool parseCommandLine(
  const char *line,
  PendingCommand &command,
  const char *&errorReason
) {
  clearCommand(command);
  const char *cursor = line;
  bool hasMotor = false;

  while (*cursor != '\0') {
    if (*cursor != '#') {
      errorReason = "FORMAT";
      return false;
    }
    ++cursor;

    uint32_t motor = 0;
    if (!parseUnsignedInteger(cursor, motor)
        || motor < 1
        || motor > SERVO_COUNT) {
      errorReason = "CHANNEL";
      return false;
    }
    const size_t axis = motor - 1;

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

    float targetPulse = 0.0f;
    if (mode == 'A') {
      if (value < SERVO_MIN_DEG[axis] || value > SERVO_MAX_DEG[axis]) {
        errorReason = "VALUE";
        return false;
      }
      targetPulse = angleToPulseUs(axis, value);
    } else {
      if (value < SERVO_MIN_PULSE_US || value > SERVO_MAX_PULSE_US) {
        errorReason = "VALUE";
        return false;
      }
      targetPulse = value;
    }

    command.selected[axis] = true;
    command.angleMode[axis] = mode == 'A';
    command.inputValue[axis] = value;
    command.targetPulseUs[axis] = targetPulse;
    hasMotor = true;

    if (!parseTimingSuffix(cursor, command, axis, errorReason)) {
      return false;
    }
  }

  if (!hasMotor) {
    errorReason = "EMPTY";
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

  uint32_t position = 0;
  if (!parseUnsignedInteger(cursor, position)
      || position < 1
      || position > SERVO_POSITION_COUNT) {
    errorReason = "GOPOS_INDEX";
    return false;
  }

  uint32_t durationMs = SERVO_POSITION_DEFAULT_TIME_MS;
  if (*cursor == 'T') {
    ++cursor;
    if (!parseUnsignedInteger(cursor, durationMs)
        || durationMs < 100
        || durationMs > 9999) {
      errorReason = "GOPOS_TIME";
      return false;
    }
  }

  if (*cursor != '\0') {
    errorReason = "GOPOS_FORMAT";
    return false;
  }

  const size_t positionIndex = position - 1;
  for (size_t axis = 0; axis < SERVO_COUNT; ++axis) {
    command.selected[axis] = true;
    command.angleMode[axis] = true;
    command.inputValue[axis] = SERVO_POSITIONS[positionIndex][axis];
    command.targetPulseUs[axis] =
      angleToPulseUs(axis, command.inputValue[axis]);
    command.hasRequestedTime[axis] = true;
    command.requestedTimeMs[axis] = durationMs;
    command.delayMs[axis] = 0;
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
    if (command.angleMode[axis]) {
      Serial.print(":angle ");
      Serial.print(command.inputValue[axis], 2);
      Serial.print(",pin ");
      Serial.print(SERVO_PINS[axis]);
      Serial.print(",pulse ");
      Serial.print((float)pulseUsToOutputUs(command.targetPulseUs[axis]), 2);
      Serial.print("us");
    } else {
      Serial.print(":pulse ");
      Serial.print(command.inputValue[axis], 2);
      Serial.print("us");
      Serial.print(",pin ");
      Serial.print(SERVO_PINS[axis]);
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

void reportPinMap() {
  Serial.print("PINMAP");
  for (size_t axis = 0; axis < SERVO_COUNT; ++axis) {
    Serial.print(",motor");
    Serial.print(axis + 1);
    Serial.print(":gpio");
    Serial.print(SERVO_PINS[axis]);
  }
  Serial.println();
}

void processCommand(const char *line) {
  if (strcmp(line, "#PINMAP") == 0) {
    Serial.println("OK");
    reportPinMap();
    return;
  }

  if (strcmp(line, "#OFFSET") == 0) {
    Serial.println("OK");
    reportServoOffsets();
    return;
  }

  if (strcmp(line, "#OFFSET#RESET") == 0) {
    if (!resetServoOffsets()) {
      sendError("OFFSET_RESET", line);
      return;
    }
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
    if (!servoOutputReady) {
      sendError("SERVO_OUTPUT_INIT", line);
      return;
    }

    const bool enableTest =
      strcmp(line, "#TEST ON") == 0
        ? true
        : strcmp(line, "#TEST OFF") == 0
          ? false
          : !servoTestEnabled;

    if (enableTest) {
      startServoTest(servoTestAxis);
    } else {
      servoTestEnabled = false;
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
    if (!servoOutputReady) {
      sendError("SERVO_OUTPUT_INIT", line);
      return;
    }

    startServoTest((size_t)(requestedMotor - 1));
    Serial.println("OK");
    return;
  }

  if (strncmp(line, "#TEST", 5) == 0) {
    sendError("TEST_FORMAT", line);
    return;
  }

  if (strcmp(line, "#START") == 0) {
    servoOutputReady = true;
    startPwmDebug(PWM_DEBUG_BOOT);
    beginBootSoftAlign(millis());
    Serial.println("OK");
    return;
  }

  if (strcmp(line, "#RESET_BOOT") == 0) {
    if (!servoOutputReady) {
      sendError("SERVO_OUTPUT_INIT", line);
      return;
    }

    startPwmDebug(PWM_DEBUG_BOOT);
    beginBootSoftAlign(millis());
    Serial.println("OK");
    return;
  }

  if (strcmp(line, "#RESET") == 0) {
    if (!servoOutputReady) {
      sendError("SERVO_OUTPUT_INIT", line);
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
  if (!servoOutputReady) {
    sendError("SERVO_OUTPUT_INIT", line);
    return;
  }

  const uint32_t nowMs = millis();
  updateMotion(nowMs, true);
  uint32_t acceptedDurationMs[SERVO_COUNT] = {};

  for (size_t axis = 0; axis < SERVO_COUNT; ++axis) {
    if (!command.selected[axis]) {
      continue;
    }

    acceptedDurationMs[axis] = calculateAcceptedDurationMs(
      axis,
      command.targetPulseUs[axis],
      command.hasRequestedTime[axis],
      command.requestedTimeMs[axis]
    );

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
  for (size_t axis = 0; axis < SERVO_COUNT; ++axis) {
    pinMode(SERVO_PINS[axis], OUTPUT);
    digitalWrite(SERVO_PINS[axis], LOW);
  }

  Serial.begin(SERIAL_BAUD_RATE);
  const uint32_t serialWaitStartMs = millis();
  while (!Serial && millis() - serialWaitStartMs < 2000) {
    delay(10);
  }
  Serial.println("BOOT");
  startPwmDebug(PWM_DEBUG_BOOT);

  loadServoOffsets();

  for (size_t axis = 0; axis < SERVO_COUNT; ++axis) {
    const float initialPulseUs =
      angleToPulseUs(axis, SERVO_INITIAL_DEG[axis]);
    currentPulseUs[axis] = initialPulseUs;
    targetPulseUs[axis] = initialPulseUs;
    axisMotionActive[axis] = false;
    axisMotionStartMs[axis] = 0;
    axisMotionDurationMs[axis] = 0;
    lastWrittenPulseUs[axis] = -1.0f;
  }

  stopPwmDebug();
  Serial.println("READY");
  reportCurrentAngles();
  if (servoTestEnabled) {
    Serial.print("TEST,ENABLED,SERVO,");
    Serial.println(servoTestAxis + 1);
    nextTestMotionMs = millis() + SERVO_TEST_START_DELAY_MS;
  }
}

void loop() {
  if (bootInitializationPending) {
    const uint32_t nowMs = millis();
    const bool bootAlignComplete = updateBootSoftAlign(nowMs);
    if (!servoOutputReady) {
      stopPwmDebug();
      sendError("SERVO_OUTPUT_INIT", "BOOT");
      bootInitializationPending = false;
      return;
    }
    if (bootAlignComplete) {
      bootInitializationPending = false;
      stopPwmDebug();
      Serial.println("READY");
      reportCurrentAngles();
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
