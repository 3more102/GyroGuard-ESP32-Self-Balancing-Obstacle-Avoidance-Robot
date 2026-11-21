#pragma once
#include <cmath>
#include <cstdint>

namespace gyroguard {
// Diagnostic-only wheel tick rate in A-rising counts/second.
// A numerically valid reading does not prove encoder connectivity.
struct EncoderTickRates {
  float left = 0.0f;
  float right = 0.0f;
  bool valid = false;
};

class EncoderTickRateEstimator {
public:
  explicit EncoderTickRateEstimator(float maxTicksPerSecond = 20000.0f)
      : maxRate_(maxTicksPerSecond) {}

  void reset() {
    initialized_ = false;
    left_ = right_ = 0;
    lastMs_ = 0;
  }

  EncoderTickRates observe(int32_t left, int32_t right, uint32_t nowMs) {
    if (!initialized_) {
      initialized_ = true;
      left_ = left;
      right_ = right;
      lastMs_ = nowMs;
      return {};
    }
    const uint32_t dt = nowMs - lastMs_; // millis() rollover supported
    if (dt < 20u) return {};            // preserve baseline
    // Use double intermediates to avoid signed integer subtraction overflow.
    const double l = (static_cast<double>(left) - left_) * 1000.0 / dt;
    const double r = (static_cast<double>(right) - right_) * 1000.0 / dt;
    left_ = left;
    right_ = right;
    lastMs_ = nowMs;
    if (dt > 300u || !std::isfinite(maxRate_) || maxRate_ <= 0.0f ||
        !std::isfinite(l) || !std::isfinite(r) ||
        std::fabs(l) > maxRate_ || std::fabs(r) > maxRate_)
      return {};
    return {static_cast<float>(l), static_cast<float>(r), true};
  }

private:
  float maxRate_;
  int32_t left_ = 0;
  int32_t right_ = 0;
  uint32_t lastMs_ = 0;
  bool initialized_ = false;
};
} // namespace gyroguard
