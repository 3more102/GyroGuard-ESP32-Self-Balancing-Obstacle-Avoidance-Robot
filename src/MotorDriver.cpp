#include "MotorDriver.h"
#include "Config.h"
#include <cmath>

#ifndef GYROGUARD_MOTOR_ENABLE
#define GYROGUARD_MOTOR_ENABLE 0
#endif
#if GYROGUARD_MOTOR_ENABLE != 0 && GYROGUARD_MOTOR_ENABLE != 1
#error "GYROGUARD_MOTOR_ENABLE must be 0 or 1"
#endif

void MotorDriver::begin() {
  pinMode(cfg::MOTOR_STBY, OUTPUT);
  digitalWrite(cfg::MOTOR_STBY, LOW);
  const int pins[] = {cfg::MOTOR_L_IN1, cfg::MOTOR_L_IN2,
                      cfg::MOTOR_R_IN1, cfg::MOTOR_R_IN2};
  for (int p : pins) { pinMode(p, OUTPUT); digitalWrite(p, LOW); }
  // PlatformIO espressif32 6.12.0 uses Arduino-ESP32 2.0.17.
  ledcSetup(0, 20000, 10);
  ledcSetup(1, 20000, 10);
  ledcAttachPin(cfg::MOTOR_L_PWM, 0);
  ledcAttachPin(cfg::MOTOR_R_PWM, 1);
  ledcWrite(0, 0);
  ledcWrite(1, 0);
}
void MotorDriver::writeMotor(float power, int sign, int in1, int in2, uint8_t channel) {
  if (!std::isfinite(power)) power = 0.0f;
  power = constrain(power * static_cast<float>(sign), -cfg::MAX_MOTOR_FRACTION,
                    cfg::MAX_MOTOR_FRACTION);
  if (power > 0.001f) {
    digitalWrite(in1, HIGH); digitalWrite(in2, LOW);
  } else if (power < -0.001f) {
    digitalWrite(in1, LOW); digitalWrite(in2, HIGH);
  } else {
    digitalWrite(in1, LOW); digitalWrite(in2, LOW);
  }
  ledcWrite(channel, static_cast<uint32_t>(fabsf(power) * 1023.0f));
}
void MotorDriver::set(float left, float right) {
#if GYROGUARD_MOTOR_ENABLE
  writeMotor(left, cfg::LEFT_MOTOR_SIGN, cfg::MOTOR_L_IN1, cfg::MOTOR_L_IN2, 0);
  writeMotor(right, cfg::RIGHT_MOTOR_SIGN, cfg::MOTOR_R_IN1, cfg::MOTOR_R_IN2, 1);
  digitalWrite(cfg::MOTOR_STBY, HIGH);
#else
  // Compile-time lock: even an accidental call cannot energize STBY.
  (void)left;
  (void)right;
  disable();
#endif
}
void MotorDriver::disable() {
  digitalWrite(cfg::MOTOR_STBY, LOW);
  ledcWrite(0, 0); ledcWrite(1, 0);
  digitalWrite(cfg::MOTOR_L_IN1, LOW); digitalWrite(cfg::MOTOR_L_IN2, LOW);
  digitalWrite(cfg::MOTOR_R_IN1, LOW); digitalWrite(cfg::MOTOR_R_IN2, LOW);
}
