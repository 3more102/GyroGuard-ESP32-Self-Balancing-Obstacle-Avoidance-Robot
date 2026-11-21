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
bool Mpu6050::readRaw(int16_t &ax, int16_t &ay, int16_t &az,
                      int16_t &gx, int16_t &gy, int16_t &gz) {
  uint8_t b[14];
  if (!readRegisters(0x3B, b, sizeof(b))) return false;
  ax = static_cast<int16_t>((static_cast<uint16_t>(b[0]) << 8) | b[1]);
  ay = static_cast<int16_t>((static_cast<uint16_t>(b[2]) << 8) | b[3]);
  az = static_cast<int16_t>((static_cast<uint16_t>(b[4]) << 8) | b[5]);
  gx = static_cast<int16_t>((static_cast<uint16_t>(b[8]) << 8) | b[9]);
  gy = static_cast<int16_t>((static_cast<uint16_t>(b[10]) << 8) | b[11]);
  gz = static_cast<int16_t>((static_cast<uint16_t>(b[12]) << 8) | b[13]);
  return true;
}
bool Mpu6050::calibrateGyro(unsigned samples) {
  if (samples < 30) return false;
  gyroguard::GyroCalibrationGuard guard;
  for (unsigned i = 0; i < samples; ++i) {
    int16_t ax, ay, az, gx, gy, gz;
    if (!readRaw(ax, ay, az, gx, gy, gz)) return false;
    if (!guard.observe(static_cast<float>(ax) / 16384.0f,
                       static_cast<float>(ay) / 16384.0f,
                       static_cast<float>(az) / 16384.0f,
                       static_cast<float>(gx) / 131.0f,
                       static_cast<float>(gy) / 131.0f,
                       static_cast<float>(gz) / 131.0f)) return false;
    delay(4);
  }
  float bias = 0.0f;
  if (!guard.bias(bias)) return false;
  gyroYBias_ = bias;
  filtered_ = false;
  return true;
}
bool Mpu6050::read(ImuMeasurement &out, float dtSec) {
  int16_t ax, ay, az, gx, gy, gz;
  if (!readRaw(ax, ay, az, gx, gy, gz)) return false;
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
