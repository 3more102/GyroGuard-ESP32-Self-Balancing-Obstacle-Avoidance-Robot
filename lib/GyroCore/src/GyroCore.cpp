#include "GyroCore.h"
#include <cmath>

namespace gyroguard {
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
  if (!std::isfinite(pitchDeg) || !std::isfinite(pitchRateDegPerSec) ||
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

void ObstacleNavigator::reset() { turnDirection_ = 0; }

MotionIntent ObstacleNavigator::update(const ObstacleReadings &d) {
  // Unknown/stale front range = fail closed: no forward command.
  if (!d.front.valid || !std::isfinite(d.front.cm) || d.front.cm <= 0.0f) {
    reset();
    return {};
  }
  if (turnDirection_ != 0 && d.front.cm < 85.0f) {
    if (!d.left.valid || !d.right.valid) return {};
    return {0.0f, 0.55f * static_cast<float>(turnDirection_)};
  }
  if (d.front.cm < 50.0f) {
    if (!d.left.valid || !d.right.valid ||
        !std::isfinite(d.left.cm) || !std::isfinite(d.right.cm)) return {};
    const bool canLeft = d.left.cm > 35.0f;
    const bool canRight = d.right.cm > 35.0f;
    if (!canLeft && !canRight) return {};
    turnDirection_ = canRight && (!canLeft || d.right.cm >= d.left.cm) ? 1 : -1;
    return {0.0f, 0.55f * static_cast<float>(turnDirection_)};
  }
  turnDirection_ = 0;
  if (d.front.cm < 85.0f) return {0.08f, 0.0f};
  // Side-wall clearance: steer away while approaching open space.
  float turn = 0.0f;
  if (d.left.valid && d.left.cm < 25.0f) turn = 0.25f;
  if (d.right.valid && d.right.cm < 25.0f) {
    turn = (turn > 0.0f) ? 0.0f : -0.25f;
  }
  return {0.18f, turn};
}
} // namespace gyroguard
