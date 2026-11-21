#pragma once
#include "WheelVelocity.h"
#include <cmath>

namespace gyroguard {
// Experimental outer-loop wheel-velocity PI. Not connected to motor actuation.
// Gains and limits MUST be calibrated before any hardware use.
struct VelocityPISettings {
  float kp = 0.0f;       // output fraction per (tick/s) error
  float ki = 0.0f;       // output fraction per accumulated tick error
  float maxOutput = 0.15f;
  float maxTargetTicksPerSec = 500.0f;
  float maxMeasuredTicksPerSec = 5000.0f;
  float maxIntegral = 500.0f;
};

struct VelocityPreview {
  float left = 0.0f;   // normalized prospective left wheel correction
  float right = 0.0f;  // normalized prospective right wheel correction
  bool valid = false;
};

class WheelVelocityPI {
public:
  explicit WheelVelocityPI(VelocityPISettings settings = {}) : settings_(settings) {}

  void enable(bool value) {
    if (enabled_ != value) {
      integralLeft_ = 0.0f;
      integralRight_ = 0.0f;
    }
    enabled_ = value;
  }
  void reset() { enable(false); }
  bool enabled() const { return enabled_; }

  // Intended for a slower, e.g. 10 Hz, outer control loop, NOT the 200 Hz
  // stabilization loop. Invalid measurements latch the preview disabled.
  VelocityPreview update(float targetLeftTps, float targetRightTps,
                         WheelRate measured, float dtSec) {
    if (!enabled_) return {};
    if (!settingsValid() || !measured.valid ||
        !std::isfinite(targetLeftTps) || !std::isfinite(targetRightTps) ||
        !std::isfinite(measured.leftTicksPerSec) ||
        !std::isfinite(measured.rightTicksPerSec) ||
        !std::isfinite(dtSec) || dtSec < 0.02f || dtSec > 0.25f ||
        std::fabs(targetLeftTps) > settings_.maxTargetTicksPerSec ||
        std::fabs(targetRightTps) > settings_.maxTargetTicksPerSec ||
        std::fabs(measured.leftTicksPerSec) > settings_.maxMeasuredTicksPerSec ||
        std::fabs(measured.rightTicksPerSec) > settings_.maxMeasuredTicksPerSec) {
      reset();
      return {};
    }
    const float left = step(targetLeftTps - measured.leftTicksPerSec,
                            integralLeft_, dtSec);
    const float right = step(targetRightTps - measured.rightTicksPerSec,
                             integralRight_, dtSec);
    return {left, right, true};
  }

private:
  static float clamp(float x, float lo, float hi) {
    return x < lo ? lo : (x > hi ? hi : x);
  }
  bool settingsValid() const {
    return std::isfinite(settings_.kp) && settings_.kp >= 0.0f &&
           std::isfinite(settings_.ki) && settings_.ki >= 0.0f &&
           (settings_.kp > 0.0f || settings_.ki > 0.0f) &&
           std::isfinite(settings_.maxOutput) &&
           settings_.maxOutput > 0.0f && settings_.maxOutput <= 1.0f &&
           std::isfinite(settings_.maxTargetTicksPerSec) &&
           settings_.maxTargetTicksPerSec > 0.0f &&
           std::isfinite(settings_.maxMeasuredTicksPerSec) &&
           settings_.maxMeasuredTicksPerSec > 0.0f &&
           std::isfinite(settings_.maxIntegral) && settings_.maxIntegral > 0.0f;
  }
  float step(float error, float &integral, float dtSec) const {
    const float candidate = clamp(integral + error * dtSec,
                                  -settings_.maxIntegral, settings_.maxIntegral);
    const float requested = settings_.kp * error + settings_.ki * candidate;
    // Conditional integration prevents further windup into saturation.
    if (!((requested > settings_.maxOutput && error > 0.0f) ||
          (requested < -settings_.maxOutput && error < 0.0f))) {
      integral = candidate;
    }
    return clamp(settings_.kp * error + settings_.ki * integral,
                 -settings_.maxOutput, settings_.maxOutput);
  }
  VelocityPISettings settings_;
  bool enabled_ = false;
  float integralLeft_ = 0.0f;
  float integralRight_ = 0.0f;
};
} // namespace gyroguard
