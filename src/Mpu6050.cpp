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
bool Mpu6050::readRaw(int16_t &ax, int16_t &ay, int16_t &az, int16_t &gy) {
  uint8_t b[14];
  if (!readRegisters(0x3B, b, sizeof(b))) return false;
  ax = static_cast<int16_t>((static_cast<uint16_t>(b[0]) << 8) | b[1]);
  ay = static_cast<int16_t>((static_cast<uint16_t>(b[2]) << 8) | b[3]);
  az = static_cast<int16_t>((static_cast<uint16_t>(b[4]) << 8) | b[5]);
  gy = static_cast<int16_t>((static_cast<uint16_t>(b[10]) << 8) | b[11]);
  return true;
}
bool Mpu6050::calibrateGyro(unsigned samples) {
  if (samples < 10) return false;
  double mean = 0.0;
  double m2 = 0.0;
  float minPitch = 360.0f;
  float maxPitch = -360.0f;
  for (unsigned i = 0; i < samples; ++i) {
    int16_t ax, ay, az, gy;
    if (!readRaw(ax, ay, az, gy)) return false;
    const float rawRate = static_cast<float>(gy) / 131.0f;
    if (!gyroguard::imuSamplePlausible(ax / 16384.0f, ay / 16384.0f,
                                      az / 16384.0f, rawRate)) return false;
    const float pitch = gyroguard::pitchFromAccelXZ(ax, az);
    if (pitch < minPitch) minPitch = pitch;
    if (pitch > maxPitch) maxPitch = pitch;
    // Welford online variance avoids cancellation from squaring raw samples.
    const double delta = rawRate - mean;
    mean += delta / static_cast<double>(i + 1u);
    m2 += delta * (rawRate - mean);
    delay(4);
  }
  const float stdDev = static_cast<float>(std::sqrt(m2 / samples));
  if (!gyroguard::gyroCalibrationStationary(maxPitch - minPitch,
                                             static_cast<float>(mean), stdDev))
    return false;
  gyroYBias_ = static_cast<float>(mean);
  filtered_ = false;
  return true;
}
bool Mpu6050::read(ImuMeasurement &out, float dtSec) {
  int16_t ax, ay, az, gy;
  if (!readRaw(ax, ay, az, gy)) return false;
  // A disconnected/malfunctioning sensor can return a plausible I2C payload
  // containing impossible physical readings; fail closed on those samples.
  if (!gyroguard::imuSamplePlausible(ax / 16384.0f, ay / 16384.0f,
                                    az / 16384.0f,
                                    static_cast<float>(gy) / 131.0f))
    return false;
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
