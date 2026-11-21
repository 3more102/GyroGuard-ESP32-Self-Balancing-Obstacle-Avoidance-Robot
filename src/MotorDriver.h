#pragma once
#include <Arduino.h>

class MotorDriver {
public:
  void begin();
  void set(float left, float right);
  void disable();
private:
  void writeMotor(float power, int sign, int in1, int in2, uint8_t channel);
};
