#include "GyroCore.h"
#include <cmath>

namespace gyroguard {
bool ArmButtonGuard::update(bool pressed, uint32_t nowMs) {
  if (!initialized_) {
    initialized_ = true;
    rawPressed_ = stablePressed_ = pressed;
    lastChangeMs_ = nowMs;
    return false;
  }
  if (pressed != rawPressed_) {
    rawPressed_ = pressed;
    lastChangeMs_ = nowMs;
  }
  // Unsigned subtraction tolerates millis() overflow.
  if (static_cast<uint32_t>(nowMs - lastChangeMs_) < 40u) return false;
  if (rawPressed_ != stablePressed_) {
    stablePressed_ = rawPressed_;
    if (!stablePressed_) { released_ = true; return false; }
    if (released_) { released_ = false; return true; }
  }
  // A boot-release must remain stable for at least 40 ms.
  if (!stablePressed_) released_ = true;
  return false;
}

float pitchFromAccelXZ(float accelX, float accelZ) {
  return -std::atan2(accelX, accelZ) * 57.2957795f;
}
float clampFloat(float value, float low, float high) {
  return value < low ? low : (value > high ? high : value);
}

BalanceController::BalanceController(BalanceGains gains) : gains_(gains) {}
void BalanceController::reset() { integral_ = 0.0f; }

MotorCommand BalanceController::update(float pitchDeg, float pitchRateDegPerSec,
                                       float desiredDrive, float desiredTurn,
                                       float dtSec) {
  // Invalid navigation intent must never turn into a non-finite motor output.
  // Treat corrupt or uninitialized intent as a control fault and fail closed.
  if (!std::isfinite(pitchDeg) || !std::isfinite(pitchRateDegPerSec) ||
      !std::isfinite(desiredDrive) || !std::isfinite(desiredTurn) ||
      !std::isfinite(dtSec) || dtSec <= 0.0f || dtSec > 0.030f) {
    reset();
    return {};
  }
  const float drive = clampFloat(desiredDrive, -1.0f, 1.0f);
  const float turn = clampFloat(desiredTurn, -1.0f, 1.0f);
  // A negative pitch target biases forward wheel acceleration.
  const float targetPitch = -drive * gains_.maxLeanDeg;
  const float error = pitchDeg - targetPitch;
  integral_ = clampFloat(integral_ + error * dtSec,
                         -gains_.integralLimit, gains_.integralLimit);
  const float upright = gains_.kp * error + gains_.ki * integral_ +
                        gains_.kd * pitchRateDegPerSec;
  const float steer = turn * gains_.maxSteer;
  return {clampFloat(upright + steer, -gains_.maxPower, gains_.maxPower),
          clampFloat(upright - steer, -gains_.maxPower, gains_.maxPower)};
}


ArmingInterlock::ArmingInterlock(float maxAbsPitchDeg,
                               float maxAbsPitchRateDegPerSec,
                               uint32_t stableHoldMs)
    : maxAbsPitchDeg_(maxAbsPitchDeg),
      maxAbsPitchRateDegPerSec_(maxAbsPitchRateDegPerSec),
      stableHoldMs_(stableHoldMs) {}

void ArmingInterlock::reset() {
  tracking_ = false;
  ready_ = false;
  stableSinceMs_ = 0;
  lastSampleMs_ = 0;
}

bool ArmingInterlock::observe(float pitchDeg, float pitchRateDegPerSec,
                              bool imuHealthy, uint32_t nowMs) {
  if (!imuHealthy || !std::isfinite(pitchDeg) ||
      !std::isfinite(pitchRateDegPerSec) ||
      std::fabs(pitchDeg) > maxAbsPitchDeg_ ||
      std::fabs(pitchRateDegPerSec) > maxAbsPitchRateDegPerSec_) {
    reset();
    return false;
  }
  // A missed sample breaks the required continuous stable interval. The
  // controller normally samples every 5 ms; gaps >20 ms reset readiness.
  if (tracking_ && nowMs - lastSampleMs_ > 20u) reset();
  if (!tracking_) {
    tracking_ = true;
    stableSinceMs_ = nowMs;
  }
  lastSampleMs_ = nowMs;
  // Unsigned subtraction preserves elapsed time across millis() rollover.
  ready_ = (nowMs - stableSinceMs_ >= stableHoldMs_);
  return ready_;
}

void ServoScanScheduler::reset() {
  front_ = {};
  left_ = {};
  right_ = {};
  target_ = LookDirection::Front;
  scanning_ = false;
  haveStartedSweep_ = false;
  sweepStartedMs_ = 0;
}

DistanceReading ServoScanScheduler::fresh(const Snapshot &sample,
                                         uint32_t nowMs,
                                         uint32_t maxAgeMs) {
  if (!sample.range.valid || nowMs - sample.measuredMs > maxAgeMs)
    return {};
  return sample.range;
}

ObstacleReadings ServoScanScheduler::readings(uint32_t nowMs) const {
  // Front updates frequently while pointing ahead. Sides remain snapshots
  // from the last sweep and intentionally have a longer expiry.
  return {fresh(front_, nowMs, 350u), fresh(left_, nowMs, 2200u),
          fresh(right_, nowMs, 2200u)};
}

void ServoScanScheduler::measurement(LookDirection direction, float cm,
                                    bool valid, uint32_t nowMs) {
  // Ignore delayed echoes attributed to an earlier servo position.
  if (direction != target_) return;
  Snapshot *sample = direction == LookDirection::Front ? &front_ :
                     (direction == LookDirection::Left ? &left_ : &right_);
  sample->range.valid = valid && std::isfinite(cm) && cm >= 2.0f && cm <= 260.0f;
  sample->range.cm = sample->range.valid ? cm : 0.0f;
  sample->measuredMs = nowMs;

  if (direction == LookDirection::Left) {
    target_ = LookDirection::Right;
  } else if (direction == LookDirection::Right) {
    target_ = LookDirection::Front;
  } else if (scanning_) {
    // A new front reading must be acquired before re-enabling navigation.
    scanning_ = false;
  } else if (sample->range.valid && cm < 50.0f &&
             (!haveStartedSweep_ || nowMs - sweepStartedMs_ >= 1500u)) {
    scanning_ = true;
    haveStartedSweep_ = true;
    sweepStartedMs_ = nowMs;
    left_.range.valid = false;
    right_.range.valid = false;
    target_ = LookDirection::Left;
  }
}

namespace {
// Even a reading marked valid must lie within the HC-SR04's configured
// accepted measurement envelope. Fail closed on corrupt/non-finite inputs.
bool usableDistance(const DistanceReading &d) {
  return d.valid && std::isfinite(d.cm) && d.cm >= 2.0f && d.cm <= 260.0f;
}
}

void ObstacleNavigator::reset() { turnDirection_ = 0; }

MotionIntent ObstacleNavigator::update(const ObstacleReadings &d) {
  if (!usableDistance(d.front)) {
    reset();
    return {};
  }

  // Require both recent side measurements to continue a committed turn.
  if (turnDirection_ != 0 && d.front.cm < 85.0f) {
    if (!usableDistance(d.left) || !usableDistance(d.right)) {
      reset();
      return {};
    }
    return {0.0f, 0.55f * static_cast<float>(turnDirection_)};
  }

  if (d.front.cm < 50.0f) {
    if (!usableDistance(d.left) || !usableDistance(d.right)) {
      reset();
      return {};
    }
    const bool canLeft = d.left.cm > 35.0f;
    const bool canRight = d.right.cm > 35.0f;
    if (!canLeft && !canRight) {
      reset();
      return {};
    }
    turnDirection_ = canRight && (!canLeft || d.right.cm >= d.left.cm) ? 1 : -1;
    return {0.0f, 0.55f * static_cast<float>(turnDirection_)};
  }

  turnDirection_ = 0;
  if (d.front.cm < 85.0f) return {0.08f, 0.0f};

  float turn = 0.0f;
  if (usableDistance(d.left) && d.left.cm < 25.0f) turn = 0.25f;
  if (usableDistance(d.right) && d.right.cm < 25.0f) {
    turn = (turn > 0.0f) ? 0.0f : -0.25f;
  }
  return {0.18f, turn};
}
} // namespace gyroguard
