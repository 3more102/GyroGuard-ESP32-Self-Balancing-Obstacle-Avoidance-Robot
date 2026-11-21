#pragma once
#include <cstdint>
#include <cmath>

namespace gyroguard {
// Diagnostic-only wheel tick rates. "valid" does not prove an encoder exists.
struct WheelRate {
  float leftTicksPerSec = 0.0f;
  float rightTicksPerSec = 0.0f;
  bool valid = false;
};

class WheelVelocityEstimator {
public:
  void reset() {
    initialized_ = false;
    lastLeft_ = lastRight_ = lastUs_ = 0u;
    rate_ = {};
  }
  WheelRate update(uint32_t left, uint32_t right, uint32_t nowUs) {
    if (!initialized_) {
      initialized_ = true;
      lastLeft_ = left;
      lastRight_ = right;
      lastUs_ = nowUs;
      return rate_;
    }
    // Integrate ticks for at least 50 ms to limit quantization noise.
    const uint32_t elapsed = nowUs - lastUs_;
    if (elapsed < 50000u) return rate_;
    const int64_t leftDelta = signedDelta(left, lastLeft_);
    const int64_t rightDelta = signedDelta(right, lastRight_);
    lastLeft_ = left;
    lastRight_ = right;
    lastUs_ = nowUs;
    if (elapsed > 250000u) {
      rate_ = {}; // control loop stalled: discard old window
      return rate_;
    }
    const float scale = 1000000.0f / static_cast<float>(elapsed);
    const float lv = static_cast<float>(leftDelta) * scale;
    const float rv = static_cast<float>(rightDelta) * scale;
    // High rates indicate noise or an incorrect encoder configuration.
    // 50k ticks/s is only an anomaly threshold, not a motor specification.
    if (!std::isfinite(lv) || !std::isfinite(rv) ||
        std::fabs(lv) > 50000.0f || std::fabs(rv) > 50000.0f) {
      rate_ = {};
      return rate_;
    }
    rate_ = {lv, rv, true};
    return rate_;
  }
private:
  static int64_t signedDelta(uint32_t now, uint32_t previous) {
    const uint32_t d = now - previous; // unsigned counter wrap
    return d <= 0x7FFFFFFFu ? static_cast<int64_t>(d)
                            : static_cast<int64_t>(d) - 4294967296LL;
  }
  bool initialized_ = false;
  uint32_t lastLeft_ = 0, lastRight_ = 0, lastUs_ = 0;
  WheelRate rate_;
};
} // namespace gyroguard
