#pragma once
#include <stdint.h>

// Target: classic ESP32-WROOM-32 DevKit (NOT ESP32-C3/S3).
// HC-SR04 ECHO must be divided/level-shifted to <= 3.3 V.
namespace cfg {
constexpr int SDA = 21, SCL = 22;
constexpr int MOTOR_L_IN1 = 18, MOTOR_L_IN2 = 19, MOTOR_L_PWM = 25;
constexpr int MOTOR_R_IN1 = 16, MOTOR_R_IN2 = 17, MOTOR_R_PWM = 26;
constexpr int MOTOR_STBY = 23;
constexpr int ARM_BUTTON = 27; // button to GND, internal pull-up
constexpr int SONAR_TRIG = 32, SONAR_ECHO = 34;
constexpr int SERVO_PWM = 33;  // external 5V servo supply; common GND
constexpr int SERVO_FRONT_DEG = 90;
constexpr int SERVO_LEFT_DEG  = 135; // swap left/right if bracket reversed
constexpr int SERVO_RIGHT_DEG = 45;
constexpr uint32_t SERVO_SETTLE_MS = 220; // measure only after movement
constexpr uint32_t SONAR_INTERVAL_US = 65000; // >60 ms between triggers
constexpr uint32_t SONAR_ECHO_TIMEOUT_US = 25000;
constexpr uint32_t CONTROL_PERIOD_US = 5000;  // 200 Hz
constexpr uint32_t MAX_CONTROL_GAP_US = 30000;
constexpr float FALL_LIMIT_DEG = 32.0f;
constexpr float ARM_LIMIT_DEG = 6.0f;
constexpr float ARM_MAX_RATE_DPS = 10.0f;   // reject arming while moving
constexpr uint32_t ARM_STABLE_MS = 750;    // sustained low-tilt, low-rate samples
constexpr uint32_t ARM_MAX_SAMPLE_AGE_MS = 25; // disallow stale-IMU arming
constexpr float MAX_MOTOR_FRACTION = 0.70f; // conservative startup cap
constexpr int LEFT_MOTOR_SIGN = 1;   // change to -1 only after wheel test
constexpr int RIGHT_MOTOR_SIGN = 1;  // change to -1 only after wheel test
constexpr float PITCH_SIGN = 1.0f;   // verify sensor axis before floor test
constexpr float MOTOR_SIGN = 1.0f;   // verify corrective direction while held
} // namespace cfg
