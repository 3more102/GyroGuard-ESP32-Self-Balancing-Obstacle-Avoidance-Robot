#pragma once
#include <Arduino.h>

struct ImuMeasurement {
  float pitchDeg = 0.0f;
  float gyroYDegPerSec = 0.0f;
};

class Mpu6050 {
public:
  bool begin();
  // Keep robot motionless; powered motors remain disabled during calibration.
  bool calibrateGyro(unsigned samples = 300);
  bool read(ImuMeasurement &out, float dtSec);
private:
  bool readRegisters(uint8_t reg, uint8_t *buf, size_t len);
  bool writeRegister(uint8_t reg, uint8_t value);
  bool readRaw(int16_t &ax, int16_t &ay, int16_t &az, int16_t &gy);
  float gyroYBias_ = 0.0f;
  float pitchDeg_ = 0.0f;
  bool filtered_ = false;
  static constexpr uint8_t kAddress = 0x68;
};
