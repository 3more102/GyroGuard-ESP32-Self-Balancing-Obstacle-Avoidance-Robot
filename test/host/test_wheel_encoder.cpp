#include "WheelEncoders.h"
#include "Config.h"
#include <cassert>
#include <iostream>
#include <map>

namespace {
struct Hook { void (*handler)(void *) = nullptr; void *context = nullptr; };
std::map<int, Hook> hooks;
std::map<int, int> inputLevels;
int criticalDepth = 0;

void pulseA(int gpio) {
  const Hook h = hooks.at(gpio);
  assert(h.handler != nullptr);
  h.handler(h.context);
}
} // namespace

void pinMode(int, int) {}
int digitalPinToInterrupt(int pin) { return pin; }
int digitalRead(int pin) { return inputLevels[pin]; }
void noInterrupts() { ++criticalDepth; }
void interrupts() { --criticalDepth; assert(criticalDepth >= 0); }
void attachInterruptArg(int gpio, void (*handler)(void *), void *ctx, int mode) {
  assert(mode == RISING);
  hooks[gpio] = {handler, ctx};
}

int main() {
  WheelEncoders encoder;
  encoder.begin();
  assert(criticalDepth == 0);
  assert(hooks.size() == 2u);
  auto counts = encoder.snapshot();
  assert(counts.left == 0 && counts.right == 0);

  // Positive when phase B high, negative when low (per configured sign).
  inputLevels[cfg::ENCODER_LEFT_B] = HIGH;
  pulseA(cfg::ENCODER_LEFT_A);
  counts = encoder.snapshot();
  assert(counts.left == cfg::ENCODER_LEFT_SIGN && counts.right == 0);

  inputLevels[cfg::ENCODER_RIGHT_B] = LOW;
  pulseA(cfg::ENCODER_RIGHT_A);
  counts = encoder.snapshot();
  assert(counts.left == cfg::ENCODER_LEFT_SIGN);
  assert(counts.right == -cfg::ENCODER_RIGHT_SIGN);

  inputLevels[cfg::ENCODER_LEFT_B] = LOW;
  inputLevels[cfg::ENCODER_RIGHT_B] = HIGH;
  pulseA(cfg::ENCODER_LEFT_A);
  pulseA(cfg::ENCODER_RIGHT_A);
  counts = encoder.snapshot();
  assert(counts.left == 0 && counts.right == 0);
  assert(criticalDepth == 0);

  std::cout << "PASS: optional wheel encoder direction/count diagnostics\n";
}
