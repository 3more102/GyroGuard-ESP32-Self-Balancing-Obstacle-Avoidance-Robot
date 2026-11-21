#pragma once
#include <Arduino.h>
#include <cstdint>
#include "Config.h"

// Opt-in 2x quadrature monitoring: interrupts on A edges, B determines sign.
// Monitoring ONLY; no motor-speed PID, positioning or autonomous drive.
class WheelEncoders {
public:
  void begin() {
    // GPIO35/36 lack internal pull-ups: condition signals externally.
    pinMode(cfg::ENC_LEFT_A, INPUT);
    pinMode(cfg::ENC_RIGHT_A, INPUT);
    pinMode(cfg::ENC_LEFT_B, INPUT_PULLUP);
    pinMode(cfg::ENC_RIGHT_B, INPUT_PULLUP);
    attachInterruptArg(cfg::ENC_LEFT_A, onLeftA, this, CHANGE);
    attachInterruptArg(cfg::ENC_RIGHT_A, onRightA, this, CHANGE);
  }
  void snapshot(uint32_t &left, uint32_t &right) const {
    noInterrupts();
    left = leftTicks_;
    right = rightTicks_;
    interrupts();
  }
private:
  static void IRAM_ATTR onLeftA(void *arg) {
    auto *e = static_cast<WheelEncoders *>(arg);
    const int step = digitalRead(cfg::ENC_LEFT_A) ==
                     digitalRead(cfg::ENC_LEFT_B) ? 1 : -1;
    e->leftTicks_ += step * cfg::ENC_LEFT_SIGN; // modular unsigned tick count
  }
  static void IRAM_ATTR onRightA(void *arg) {
    auto *e = static_cast<WheelEncoders *>(arg);
    const int step = digitalRead(cfg::ENC_RIGHT_A) ==
                     digitalRead(cfg::ENC_RIGHT_B) ? 1 : -1;
    e->rightTicks_ += step * cfg::ENC_RIGHT_SIGN;
  }
  volatile uint32_t leftTicks_ = 0u;
  volatile uint32_t rightTicks_ = 0u;
};
