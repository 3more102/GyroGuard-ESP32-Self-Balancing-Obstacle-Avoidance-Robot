#include <Arduino.h>
#include <Wire.h>
#include <cmath>
#include "Config.h"
#include "Mpu6050.h"
#include "MotorDriver.h"
#include "Ultrasonic.h"
#include <GyroCore.h>

#ifndef GYROGUARD_AUTONOMY
#define GYROGUARD_AUTONOMY 0
#endif
#ifndef GYROGUARD_MOTOR_ENABLE
#define GYROGUARD_MOTOR_ENABLE 0
#endif
#if GYROGUARD_MOTOR_ENABLE != 0 && GYROGUARD_MOTOR_ENABLE != 1
#error "GYROGUARD_MOTOR_ENABLE must be 0 or 1"
#endif
#ifndef GYROGUARD_ENCODERS
#define GYROGUARD_ENCODERS 0
#endif
#if GYROGUARD_ENCODERS
#include "WheelEncoders.h"
#include <WheelVelocity.h>
WheelEncoders wheelEncoders;
gyroguard::WheelVelocityEstimator wheelVelocity;
gyroguard::WheelRate latestWheelRate;
#endif

Mpu6050 imu;
MotorDriver motors;
UltrasonicScanner sonar;
gyroguard::BalanceController controller;
gyroguard::ObstacleNavigator navigator;
gyroguard::ArmButtonGuard armButton;
gyroguard::ArmingInterlock armingInterlock(
    cfg::ARM_LIMIT_DEG, cfg::ARM_MAX_RATE_DPS, cfg::ARM_STABLE_MS);

bool imuOkay = false;
bool armed = false;
float currentPitch = 999.0f;
float currentPitchRate = 0.0f;
float lastCommandLeft = 0.0f;
float lastCommandRight = 0.0f;
// Observed start-to-start controller periods, accumulated over each telemetry
// reporting window. These are NOT measured computation execution times.
uint32_t controlWindowMinUs = UINT32_MAX;
uint32_t controlWindowMaxUs = 0;
uint32_t controlWindowCycles = 0;
uint32_t controlDeadlineMisses = 0;
uint32_t lastControlUs = 0;
uint32_t lastPrintMs = 0;

void disarm(const char *reason) {
  const bool wasArmed = armed;
  armed = false;
  motors.disable();
  lastCommandLeft = 0.0f;
  lastCommandRight = 0.0f;
  controller.reset();
  navigator.reset();
  armingInterlock.reset();
  if (wasArmed) {
    Serial.print("DISARM: "); Serial.println(reason);
  }
}

void checkButton() {
  // First GPIO sample cannot arm; require a stable release + fresh press.
  // Called after the current 200 Hz IMU/deadline evaluation.
  if (!armButton.update(digitalRead(cfg::ARM_BUTTON) == LOW, millis())) return;
  if (armed) {
    disarm("button");
    return;
  }
#if GYROGUARD_MOTOR_ENABLE
  if (imuOkay && armingInterlock.ready() &&
      std::isfinite(currentPitch) &&
      fabsf(currentPitch) <= cfg::ARM_LIMIT_DEG) {
    controller.reset();
    navigator.reset();
    armed = true;
    Serial.println("ARMED (press button to disarm)");
  } else {
    Serial.println("ARM REFUSED: IMU fault or not steadily upright for 750 ms");
  }
#else
  Serial.println("ARM REFUSED: default sensor-only firmware locks motor output");
#endif
}

void setup() {
  Serial.begin(115200);
  pinMode(cfg::ARM_BUTTON, INPUT_PULLUP);
  motors.begin();
  motors.disable();
  Wire.begin(cfg::SDA, cfg::SCL);
  Wire.setClock(400000);
  Wire.setTimeOut(8);
  Serial.println("GyroGuard ESP32 | SAFETY: motors DISABLED");
  Serial.println("Release the arm button; press only after arm_ready=1.");
  Serial.printf("Motor output build lock: %s\n",
                GYROGUARD_MOTOR_ENABLE ? "UNLOCKED (BENCH ONLY)" : "LOCKED");
  Serial.println("Hold chassis still, upright, with wheels off the ground.");
  imuOkay = imu.begin() && imu.calibrateGyro();
  if (!imuOkay) Serial.println("MPU6050 initialization/calibration failed");
  sonar.begin();
#if GYROGUARD_ENCODERS
  wheelEncoders.begin();
  Serial.println("Encoder monitor ON; no motor velocity feedback");
#endif
  lastControlUs = micros();
  Serial.printf("Autonomous navigation: %s\n", GYROGUARD_AUTONOMY ? "ENABLED" : "DISABLED");
}

void loop() {
  sonar.tick(); // non-blocking echoes
  const uint32_t now = micros();
  const uint32_t elapsed = now - lastControlUs;
  if (elapsed >= cfg::CONTROL_PERIOD_US) {
    lastControlUs = now;
    if (elapsed < controlWindowMinUs) controlWindowMinUs = elapsed;
    if (elapsed > controlWindowMaxUs) controlWindowMaxUs = elapsed;
    ++controlWindowCycles;
#if GYROGUARD_ENCODERS
    uint32_t leftTicks = 0, rightTicks = 0;
    wheelEncoders.snapshot(leftTicks, rightTicks);
    latestWheelRate = wheelVelocity.update(leftTicks, rightTicks, now);
#endif
    if (elapsed > cfg::MAX_CONTROL_GAP_US) {
      ++controlDeadlineMisses;
      disarm("controller deadline missed");
    } else if (imuOkay) {
      ImuMeasurement m;
      const float dt = static_cast<float>(elapsed) / 1000000.0f;
      if (!imu.read(m, dt)) {
        imuOkay = false;
        disarm("IMU communication lost");
      } else {
        currentPitch = cfg::PITCH_SIGN * m.pitchDeg;
        const float rate = cfg::PITCH_SIGN * m.gyroYDegPerSec;
        currentPitchRate = rate;
        armingInterlock.observe(currentPitch, rate, imuOkay, millis());
        if (!std::isfinite(currentPitch) ||
            fabsf(currentPitch) > cfg::FALL_LIMIT_DEG) {
          disarm("tilt/fall threshold");
        } else if (armed) {
          gyroguard::MotionIntent intent;
#if GYROGUARD_AUTONOMY
          // Servo sweep has no forward-looking range; hold motion until it
          // returns to center and records a fresh front measurement.
          if (!sonar.scanning()) intent = navigator.update(sonar.readings());
          else navigator.reset();
#endif
          const auto power = controller.update(currentPitch, rate,
                                               intent.drive, intent.turn, dt);
          lastCommandLeft = cfg::MOTOR_SIGN * power.left;
          lastCommandRight = cfg::MOTOR_SIGN * power.right;
#if GYROGUARD_MOTOR_ENABLE
          motors.set(lastCommandLeft, lastCommandRight);
#else
          lastCommandLeft = 0.0f;
          lastCommandRight = 0.0f;
#endif
        }
      }
    }
  }
  // Check arming only after processing the latest IMU and control deadline.
  checkButton();
  if (millis() - lastPrintMs >= 200u) {
    lastPrintMs = millis();
    const auto d = sonar.readings();
    const char *facing = sonar.pointing() == gyroguard::LookDirection::Front ? "front" :
                         sonar.pointing() == gyroguard::LookDirection::Left ? "left" : "right";
    const uint32_t ctlMin = controlWindowCycles ? controlWindowMinUs : 0u;
    Serial.printf(
        "arm=%d imu=%d pitch=%.2f arm_ready=%d scan=%d look=%s "
        "front=%s%.1f left=%s%.1f right=%s%.1f "
        "t_ms=%lu ctl_n=%lu ctl_min_us=%lu ctl_max_us=%lu "
        "ctl_misses=%lu rate_dps=%.2f cmd_l=%.3f cmd_r=%.3f\n",
        armed, imuOkay, currentPitch, armingInterlock.ready(),
        sonar.scanning(), facing,
        d.front.valid ? "" : "?", d.front.cm,
        d.left.valid ? "" : "?", d.left.cm,
        d.right.valid ? "" : "?", d.right.cm,
        static_cast<unsigned long>(lastPrintMs),
        static_cast<unsigned long>(controlWindowCycles),
        static_cast<unsigned long>(ctlMin),
        static_cast<unsigned long>(controlWindowMaxUs),
        static_cast<unsigned long>(controlDeadlineMisses),
        currentPitchRate, lastCommandLeft, lastCommandRight);
    controlWindowCycles = 0;
    controlWindowMinUs = UINT32_MAX;
    controlWindowMaxUs = 0;
#if GYROGUARD_ENCODERS
    Serial.printf("enc_valid=%d left_tps=%.1f right_tps=%.1f (diagnostic)\n",
                  latestWheelRate.valid, latestWheelRate.leftTicksPerSec,
                  latestWheelRate.rightTicksPerSec);
#endif
  }
}
