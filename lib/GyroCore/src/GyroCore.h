#pragma once
#include <cstdint>

namespace gyroguard {
struct MotorCommand {
  float left = 0.0f;     // normalized -1 .. +1
  float right = 0.0f;
};

struct BalanceGains {
  float kp = 0.030f;     // per degree, starting values ONLY
  float ki = 0.0f;       // anti-windup integral is optional
  float kd = 0.0016f;    // per deg/s
  float integralLimit = 12.0f;
  float maxLeanDeg = 2.0f;
  float maxSteer = 0.14f;
  float maxPower = 0.70f;
};

// Arming is permitted only after a continuous sequence of fresh, upright,
// low-angular-rate IMU samples. This is a software interlock, not a hardware
// emergency stop. All times use overflow-safe uint32_t differences.
class ArmingInterlock {
public:
  ArmingInterlock(float maxTiltDeg, float maxRateDegPerSec,
                  uint32_t stableDurationMs, uint32_t maxSampleAgeMs);
  void reset();
  void observe(float pitchDeg, float pitchRateDegPerSec, uint32_t nowMs);
  bool ready(uint32_t nowMs) const;
private:
  float maxTiltDeg_;
  float maxRateDegPerSec_;
  uint32_t stableDurationMs_;
  uint32_t maxSampleAgeMs_;
  bool tracking_ = false;
  uint32_t stableSinceMs_ = 0;
  uint32_t lastSampleMs_ = 0;
};

class BalanceController {
public:
  explicit BalanceController(BalanceGains gains = {});
  void reset();
  // Positive pitch means the robot tilts forward. Motors must drive forward
  // for positive output; verify pitch/motor signs on a supported bench rig.
  MotorCommand update(float pitchDeg, float pitchRateDegPerSec,
                      float desiredDrive, float desiredTurn, float dtSec);
private:
  BalanceGains gains_;
  float integral_ = 0.0f;
};

struct DistanceReading {
  float cm = 0.0f;
  bool valid = false;
};
struct ObstacleReadings {
  DistanceReading front;
  DistanceReading left;
  DistanceReading right;
};
struct MotionIntent {
  float drive = 0.0f; // forward motion bias [-1,+1]
  float turn = 0.0f;  // positive = turn right
};

class ObstacleNavigator {
public:
  MotionIntent update(const ObstacleReadings &d);
  void reset();
private:
  int turnDirection_ = 0; // -1 left, +1 right, 0 none
};

// One physical ultrasonic sensor mounted on a PWM servo. Measurements are
// directional snapshots: they are NOT simultaneous readings.
enum class LookDirection : uint8_t { Front, Left, Right };

class ServoScanScheduler {
public:
  LookDirection target() const { return target_; }
  bool scanning() const { return scanning_; }
  void reset();
  void measurement(LookDirection direction, float cm, bool valid,
                   uint32_t nowMs);
  ObstacleReadings readings(uint32_t nowMs) const;
private:
  struct Snapshot {
    DistanceReading range;
    uint32_t measuredMs = 0;
  };
  Snapshot front_, left_, right_;
  LookDirection target_ = LookDirection::Front;
  bool scanning_ = false;
  bool haveStartedSweep_ = false;
  uint32_t sweepStartedMs_ = 0;
  static DistanceReading fresh(const Snapshot &snapshot, uint32_t nowMs,
                               uint32_t maxAgeMs);
};

// With X forward, Z up, forward tilt increases pitch; gyro +Y matches.
float pitchFromAccelXZ(float accelX, float accelZ);
float clampFloat(float value, float low, float high);
} // namespace gyroguard
