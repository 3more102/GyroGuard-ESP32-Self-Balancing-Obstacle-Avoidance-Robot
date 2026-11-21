#pragma once
#include <cstdint>

namespace gyroguard {
// Debounce and require a stable release before any new press can toggle arming.
// A button held down throughout startup never generates an arm event.
class ArmButtonGuard {
public:
  bool update(bool pressed, uint32_t nowMs);
private:
  bool initialized_ = false;
  bool rawPressed_ = false;
  bool stablePressed_ = false;
  bool released_ = false;
  uint32_t lastChangeMs_ = 0;
};

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

// Prevent motor arming until healthy IMU samples remain close to upright
// AND have low angular rate for a continuous interval.
class ArmingInterlock {
public:
  ArmingInterlock(float maxAbsPitchDeg, float maxAbsPitchRateDegPerSec,
                  uint32_t stableHoldMs);
  bool observe(float pitchDeg, float pitchRateDegPerSec, bool imuHealthy,
               uint32_t nowMs);
  void reset();
  bool ready() const { return ready_; }
private:
  float maxAbsPitchDeg_;
  float maxAbsPitchRateDegPerSec_;
  uint32_t stableHoldMs_;
  uint32_t stableSinceMs_ = 0;
  uint32_t lastSampleMs_ = 0;
  bool tracking_ = false;
  bool ready_ = false;
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
