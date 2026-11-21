#pragma once
#include <Arduino.h>
#include <cstdint>

// Optional two-wheel quadrature diagnostic, not part of the balance feedback.
// Count only A-channel rising edges; B-channel phase determines direction.
// Each tick is one A rising edge, NOT four quadrature edges.
struct EncoderCounts {
  int32_t left = 0;
  int32_t right = 0;
};

class WheelEncoders {
public:
  void begin();
  EncoderCounts snapshot() const;
private:
  static void IRAM_ATTR onLeftA(void *context);
  static void IRAM_ATTR onRightA(void *context);
  volatile int32_t leftTicks_ = 0;
  volatile int32_t rightTicks_ = 0;
};
