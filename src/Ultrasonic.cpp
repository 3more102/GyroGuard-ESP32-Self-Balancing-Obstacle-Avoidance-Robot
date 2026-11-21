#include "Ultrasonic.h"
#include "Config.h"

namespace {
constexpr int SERVO_CHANNEL = 2;  // motor PWM already uses LEDC 0 and 1
constexpr int SERVO_PERIOD_US = 20000; // 50 Hz
constexpr uint32_t SERVO_MAX_DUTY = 65535;

// Use conservative 1000..2000 us pulses until the mount is inspected.
uint32_t servoDuty(int deg) {
  deg = constrain(deg, 0, 180);
  const uint32_t pulseUs = 1000u + static_cast<uint32_t>(deg) * 1000u / 180u;
  return (pulseUs * SERVO_MAX_DUTY) / SERVO_PERIOD_US;
}
}

void IRAM_ATTR UltrasonicScanner::echoInterrupt(void *arg) {
  UltrasonicScanner *scanner = static_cast<UltrasonicScanner *>(arg);
  // Latch the first complete echo only: a subsequent ringing/noise edge
  // must not overwrite a range sample before tick() consumes it.
  if (!scanner->waitingEcho_ || scanner->ready_) return;
  const uint32_t t = micros();
  if (digitalRead(cfg::SONAR_ECHO) == HIGH) {
    if (!scanner->sawRise_) {
      scanner->riseUs_ = t;
      scanner->sawRise_ = true;
    }
  } else if (scanner->sawRise_) {
    scanner->widthUs_ = t - scanner->riseUs_;
    scanner->ready_ = true;
    scanner->sawRise_ = false;
  }
}

void UltrasonicScanner::moveTo(gyroguard::LookDirection direction) {
  int angle = cfg::SERVO_FRONT_DEG;
  if (direction == gyroguard::LookDirection::Left) angle = cfg::SERVO_LEFT_DEG;
  if (direction == gyroguard::LookDirection::Right) angle = cfg::SERVO_RIGHT_DEG;
  ledcWrite(SERVO_CHANNEL, servoDuty(angle));
  pointing_ = direction;
  servoMovedMs_ = millis();
}

void UltrasonicScanner::begin() {
  pinMode(cfg::SONAR_TRIG, OUTPUT);
  digitalWrite(cfg::SONAR_TRIG, LOW);
  pinMode(cfg::SONAR_ECHO, INPUT);
  // Existing project pins Arduino-ESP32 2.x LEDC channel-based API.
  ledcSetup(SERVO_CHANNEL, 50, 16);
  ledcAttachPin(cfg::SERVO_PWM, SERVO_CHANNEL);
  schedule_.reset();
  moveTo(gyroguard::LookDirection::Front);
  attachInterruptArg(cfg::SONAR_ECHO, echoInterrupt, this, CHANGE);
}

void UltrasonicScanner::finishPing(bool gotEcho) {
  uint32_t duration = 0;
  noInterrupts();
  duration = widthUs_;
  waitingEcho_ = false;
  ready_ = false;
  sawRise_ = false;
  interrupts();
  // Distance is half the roundtrip duration; usable max ~260 cm for 25 ms.
  const bool valid = gotEcho && duration >= 116u && duration <= 15160u;
  const float cm = valid ? static_cast<float>(duration) * 0.01715f : 0.0f;
  schedule_.measurement(pointing_, cm, valid, millis());
}

void UltrasonicScanner::tick() {
  const uint32_t nowUs = micros();
  if (waitingEcho_) {
    if (ready_) finishPing(true);
    else if (nowUs - startedPingUs_ >= cfg::SONAR_ECHO_TIMEOUT_US)
      finishPing(false);
    return;
  }

  const auto wanted = schedule_.target();
  if (wanted != pointing_) {
    moveTo(wanted);
    return;
  }
  if (millis() - servoMovedMs_ < cfg::SERVO_SETTLE_MS) return;
  if (everPinged_ && nowUs - lastPingUs_ < cfg::SONAR_INTERVAL_US) return;

  // A stuck-high echo is invalidated instead of stalling the control loop.
  if (digitalRead(cfg::SONAR_ECHO) == HIGH) {
    lastPingUs_ = nowUs;
    everPinged_ = true;
    schedule_.measurement(pointing_, 0.0f, false, millis());
    return;
  }

  noInterrupts();
  widthUs_ = 0;
  sawRise_ = false;
  ready_ = false;
  waitingEcho_ = true;
  interrupts();
  digitalWrite(cfg::SONAR_TRIG, LOW);
  delayMicroseconds(2);
  digitalWrite(cfg::SONAR_TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(cfg::SONAR_TRIG, LOW);
  lastPingUs_ = startedPingUs_ = micros();
  everPinged_ = true;
}
