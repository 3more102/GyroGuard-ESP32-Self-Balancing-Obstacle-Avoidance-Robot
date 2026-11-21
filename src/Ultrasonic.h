#pragma once
#include <Arduino.h>
#include <GyroCore.h>

// A single HC-SR04 on a servo. Echo captured with GPIO interrupt so balance
// controller is never blocked by pulseIn() or servo settling delays.
class UltrasonicScanner {
public:
  void begin();
  void tick();
  gyroguard::ObstacleReadings readings() const {
    return schedule_.readings(millis());
  }
  bool scanning() const { return schedule_.scanning(); }
  gyroguard::LookDirection pointing() const { return schedule_.target(); }
private:
  static void IRAM_ATTR echoInterrupt(void *arg);
  void moveTo(gyroguard::LookDirection direction);
  void finishPing(bool receivedEcho);
  gyroguard::ServoScanScheduler schedule_;
  gyroguard::LookDirection pointing_ = gyroguard::LookDirection::Front;
  uint32_t servoMovedMs_ = 0;
  uint32_t lastPingUs_ = 0;
  uint32_t startedPingUs_ = 0;
  bool everPinged_ = false;
  volatile bool waitingEcho_ = false;
  volatile uint32_t riseUs_ = 0;
  volatile uint32_t widthUs_ = 0;
  volatile bool sawRise_ = false;
  volatile bool ready_ = false;
};
