#include <Arduino.h>
#include <Wire.h>
#include <cmath>
#include "Config.h"
#include "Mpu6050.h"
#include "MotorDriver.h"
#include "Ultrasonic.h"
#if GYROGUARD_ENCODERS
#include "WheelEncoders.h"
#include <EncoderTickRate.h>
#endif
#include <GyroCore.h>

#ifndef GYROGUARD_AUTONOMY
#define GYROGUARD_AUTONOMY 0
#endif

Mpu6050 imu;
MotorDriver motors;
UltrasonicScanner sonar;
#if GYROGUARD_ENCODERS
WheelEncoders encoders;
gyroguard::EncoderTickRateEstimator encoderEstimator;
gyroguard::EncoderTickRates encoderRate;
uint32_t lastEncoderSampleMs = 0;
#endif
gyroguard::BalanceController controller;
gyroguard::ObstacleNavigator navigator;
gyroguard::ArmingInterlock armingInterlock(
    cfg::ARM_LIMIT_DEG, cfg::ARM_MAX_RATE_DPS, cfg::ARM_STABLE_MS);

bool imuOkay = false;
bool armed = false;
float currentPitch = 999.0f;
uint32_t lastControlUs = 0;
uint32_t lastPrintMs = 0;
uint32_t lastButtonChangeMs = 0;
bool previousButtonSample = HIGH;
bool stableButton = HIGH;

void disarm(const char *reason) {
  const bool wasArmed = armed;
  armed = false;
  motors.disable();
  controller.reset();
  navigator.reset();
  armingInterlock.reset();
  if (wasArmed) {
    Serial.print("DISARM: "); Serial.println(reason);
  }
}

void checkButton() {
  const bool sample = digitalRead(cfg::ARM_BUTTON);
  const uint32_t now = millis();
  if (sample != previousButtonSample) lastButtonChangeMs = now;
  previousButtonSample = sample;
  if (sample != stableButton && now - lastButtonChangeMs >= 40) {
    stableButton = sample;
    if (stableButton == LOW) {
      if (armed) {
        disarm("button");
      } else if (imuOkay && armingInterlock.ready() &&
                 std::isfinite(currentPitch) &&
                 fabsf(currentPitch) <= cfg::ARM_LIMIT_DEG) {
        controller.reset();
        navigator.reset();
        armed = true;
        Serial.println("ARMED (press button to disarm)");
      } else {
        Serial.println("ARM REFUSED: IMU fault or not steadily upright for 750 ms");
      }
    }
  }
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
  Serial.println("Hold chassis still, upright, with wheels off the ground.");
  imuOkay = imu.begin() && imu.calibrateGyro();
  if (!imuOkay) Serial.println("MPU6050 initialization/calibration failed");
  sonar.begin();
#if GYROGUARD_ENCODERS
  encoders.begin();
  encoderEstimator.reset();
  lastEncoderSampleMs = millis();
  Serial.println("Wheel encoders: telemetry only; no velocity PID");
#endif
  lastControlUs = micros();
  Serial.printf("Autonomous navigation: %s\n", GYROGUARD_AUTONOMY ? "ENABLED" : "DISABLED");
}

void loop() {
  sonar.tick(); // non-blocking echoes
#if GYROGUARD_ENCODERS
  // Independent 10 Hz tick-rate diagnostics; never alter motor commands.
  const uint32_t encoderNowMs = millis();
  if (encoderNowMs - lastEncoderSampleMs >= 100u) {
    lastEncoderSampleMs = encoderNowMs;
    const auto ticks = encoders.snapshot();
    encoderRate = encoderEstimator.observe(ticks.left, ticks.right, encoderNowMs);
  }
#endif
  const uint32_t now = micros();
  const uint32_t elapsed = now - lastControlUs;
  if (elapsed >= cfg::CONTROL_PERIOD_US) {
    lastControlUs = now;
    if (elapsed > cfg::MAX_CONTROL_GAP_US) {
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
          motors.set(cfg::MOTOR_SIGN * power.left, cfg::MOTOR_SIGN * power.right);
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
    Serial.printf("arm=%d imu=%d pitch=%.2f arm_ready=%d scan=%d look=%s front=%s%.1f left=%s%.1f right=%s%.1f",
                  armed, imuOkay, currentPitch, armingInterlock.ready(),
                  sonar.scanning(), facing,
                  d.front.valid ? "" : "?", d.front.cm,
                  d.left.valid ? "" : "?", d.left.cm,
                  d.right.valid ? "" : "?", d.right.cm);
#if GYROGUARD_ENCODERS
    const auto ticks = encoders.snapshot();
    Serial.printf(" enc_left=%ld enc_right=%ld enc_l_tps=%.1f enc_r_tps=%.1f enc_valid=%d",
                  static_cast<long>(ticks.left), static_cast<long>(ticks.right),
                  encoderRate.left, encoderRate.right, encoderRate.valid);
#endif
    Serial.println();
  }
}
