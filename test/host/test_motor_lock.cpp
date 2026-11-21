#include <cassert>
#include <cstdint>
#include "Config.h"
#include "MotorDriver.h"

// Minimal GPIO/PWM hardware simulator; checks firmware commands, not electronics.
namespace {
int standbyLevel = LOW;
unsigned standbyHighWrites = 0;
uint32_t pwmDuty[2] = {0, 0};
}

void pinMode(int, int) {}
void digitalWrite(int pin, int level) {
  if (pin == cfg::MOTOR_STBY) {
    standbyLevel = level;
    if (level == HIGH) ++standbyHighWrites;
  }
}
double ledcSetup(uint8_t, double, uint8_t) { return 20000.0; }
void ledcAttachPin(int, uint8_t) {}
void ledcWrite(uint8_t channel, uint32_t duty) {
  assert(channel < 2);
  pwmDuty[channel] = duty;
}

int main() {
  MotorDriver motor;
  motor.begin();
  assert(standbyLevel == LOW);
  assert(pwmDuty[0] == 0 && pwmDuty[1] == 0);
  motor.set(0.5f, -0.5f);
#if GYROGUARD_MOTOR_ENABLE
  assert(standbyLevel == HIGH);
  assert(standbyHighWrites == 1);
  assert(pwmDuty[0] > 0 && pwmDuty[1] > 0);
#else
  assert(standbyLevel == LOW);
  assert(standbyHighWrites == 0);
  assert(pwmDuty[0] == 0 && pwmDuty[1] == 0);
#endif
  motor.disable();
  assert(standbyLevel == LOW);
  assert(pwmDuty[0] == 0 && pwmDuty[1] == 0);
}
