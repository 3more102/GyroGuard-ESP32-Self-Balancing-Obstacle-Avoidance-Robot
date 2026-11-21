#include "Mpu6050.h"
#include <GyroCore.h>
#include <Wire.h>
#include <cmath>

bool Mpu6050::readRegisters(uint8_t reg, uint8_t *buf, size_t len) {
  Wire.beginTransmission(kAddress);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom(kAddress, static_cast<uint8_t>(len), static_cast<uint8_t>(true)) != len) return false;
  for (size_t i = 0; i < len; ++i) buf[i] = static_cast<uint8_t>(Wire.read());
  return true;
}
bool Mpu6050::writeRegister(uint8_t reg, uint8_t value) {
  Wire.beginTransmission(kAddress);
  Wire.write(reg);
  Wire.write(value);
  return Wire.endTransmission(true) == 0;
}
bool Mpu6050::begin() {
  uint8_t who = 0;
  if (!readRegisters(0x75, &who, 1) || who != 0x68) return false;
  // Wake sensor; 200 Hz data output, low-pass filter, +/-2g, +/-250 dps.
  if (!writeRegister(0x6B, 0x01)) return false;
  if (!writeRegister(0x1A, 0x03)) return false;
  if (!writeRegister(0x19, 0x04)) return false;
  if (!writeRegister(0x1C, 0x00)) return false;
  if (!writeRegister(0x1B, 0x00)) return false;
  delay(100);
  filtered_ = false;
  return true;
}
bool Mpu6050::readRaw(int16_t &ax, int16_t &az, int16_t &gy) {
  uint8_t b[14];
  if (!readRegisters(0x3B, b, sizeof(b))) return false;
  ax = static_cast<int16_t>((static_cast<uint16_t>(b[0]) << 8) | b[1]);
  az = static_cast<int16_t>((static_cast<uint16_t>(b[4]) << 8) | b[5]);
  gy = static_cast<int16_t>((static_cast<uint16_t>(b[10]) << 8) | b[11]);
  return true;
}
bool Mpu6050::calibrateGyro(unsigned samples) {
  if (samples < 10) return false;
  double sum = 0.0;
  for (unsigned i = 0; i < samples; ++i) {
    int16_t ax, az, gy;
    if (!readRaw(ax, az, gy)) return false;
    sum += static_cast<float>(gy) / 131.0f;
    delay(4);
  }
  gyroYBias_ = static_cast<float>(sum / samples);
  filtered_ = false;
  return true;
}
bool Mpu6050::read(ImuMeasurement &out, float dtSec) {
  int16_t ax, az, gy;
  if (!readRaw(ax, az, gy)) return false;
  const float measuredPitch = gyroguard::pitchFromAccelXZ(
      static_cast<float>(ax), static_cast<float>(az));
  const float gyroY = static_cast<float>(gy) / 131.0f - gyroYBias_;
  if (!std::isfinite(measuredPitch) || !std::isfinite(gyroY)) return false;
  if (!filtered_) {
    pitchDeg_ = measuredPitch;
    filtered_ = true;
  } else {
    // 0.5 s complementary filter time constant, 200Hz nominal tick.
    const float alpha = 0.5f / (0.5f + dtSec);
    pitchDeg_ = alpha * (pitchDeg_ + gyroY * dtSec) +
                (1.0f - alpha) * measuredPitch;
  }
  out.pitchDeg = pitchDeg_;
  out.gyroYDegPerSec = gyroY;
  return true;
}
