#include "WheelEncoders.h"
#include "Config.h"

// GPIO35/39 are input only and lack internal pull-ups. All encoder outputs
// must be 3.3 V compatible; external pull-ups to 3.3 V may be required.
void WheelEncoders::begin() {
  pinMode(cfg::ENCODER_LEFT_A, INPUT);
  pinMode(cfg::ENCODER_LEFT_B, INPUT);
  pinMode(cfg::ENCODER_RIGHT_A, INPUT);
  pinMode(cfg::ENCODER_RIGHT_B, INPUT);
  noInterrupts();
  leftTicks_ = 0;
  rightTicks_ = 0;
  interrupts();
  attachInterruptArg(digitalPinToInterrupt(cfg::ENCODER_LEFT_A),
                     onLeftA, this, RISING);
  attachInterruptArg(digitalPinToInterrupt(cfg::ENCODER_RIGHT_A),
                     onRightA, this, RISING);
}

void IRAM_ATTR WheelEncoders::onLeftA(void *context) {
  auto *self = static_cast<WheelEncoders *>(context);
  const int direction = (digitalRead(cfg::ENCODER_LEFT_B) == HIGH) ? 1 : -1;
  self->leftTicks_ += cfg::ENCODER_LEFT_SIGN * direction;
}

void IRAM_ATTR WheelEncoders::onRightA(void *context) {
  auto *self = static_cast<WheelEncoders *>(context);
  const int direction = (digitalRead(cfg::ENCODER_RIGHT_B) == HIGH) ? 1 : -1;
  self->rightTicks_ += cfg::ENCODER_RIGHT_SIGN * direction;
}

EncoderCounts WheelEncoders::snapshot() const {
  // Short critical section takes an internally consistent pair of counts.
  noInterrupts();
  const EncoderCounts counts{leftTicks_, rightTicks_};
  interrupts();
  return counts;
}
