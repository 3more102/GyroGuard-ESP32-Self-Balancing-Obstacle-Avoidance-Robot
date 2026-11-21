#include <WheelVelocityPI.h>
#include <cassert>
#include <cmath>
#include <limits>
#include <iostream>
using namespace gyroguard;

int main() {
  VelocityPISettings gains;
  gains.kp = 0.01f;
  gains.ki = 0.5f;
  gains.maxOutput = 0.2f;
  gains.maxIntegral = 1.0f;
  gains.maxTargetTicksPerSec = 100.0f;
  gains.maxMeasuredTicksPerSec = 1000.0f;
  WheelVelocityPI pi(gains);
  const WheelRate zero{0.0f, 0.0f, true};
  assert(!pi.update(10.0f, 0.0f, zero, 0.1f).valid); // opt-in only
  pi.enable(true);
  auto a = pi.update(10.0f, -10.0f, zero, 0.1f);
  assert(a.valid && a.left > 0.0f && a.right < 0.0f);
  for (int i = 0; i < 20; ++i) {
    a = pi.update(100.0f, -100.0f, zero, 0.1f);
    assert(a.valid && a.left <= 0.2f && a.right >= -0.2f);
  }
  a = pi.update(0.0f, 0.0f, zero, 0.1f);
  assert(a.valid && std::fabs(a.left) < 0.0001f &&
         std::fabs(a.right) < 0.0001f); // no saturation windup
  a = pi.update(10.0f, 10.0f, {5.0f, 15.0f, true}, 0.1f);
  assert(a.valid && a.left > 0.0f && a.right < 0.0f);

  a = pi.update(0.0f, 0.0f, {0.0f, 0.0f, false}, 0.1f);
  assert(!a.valid && !pi.enabled());
  assert(!pi.update(10.0f, 10.0f, zero, 0.1f).valid);
  pi.enable(true);
  a = pi.update(std::numeric_limits<float>::quiet_NaN(), 0.0f, zero, 0.1f);
  assert(!a.valid && !pi.enabled());
  pi.enable(true);
  assert(!pi.update(101.0f, 0.0f, zero, 0.1f).valid);
  pi.enable(true);
  assert(!pi.update(0.0f, 0.0f, {2000.0f, 0.0f, true}, 0.1f).valid);
  pi.enable(true);
  assert(!pi.update(0.0f, 0.0f, zero, 0.3f).valid);
  pi.enable(true);
  assert(!pi.update(0.0f, 0.0f, zero, 0.0f).valid);
  pi.enable(true);
  pi.reset();
  assert(!pi.enabled());

  WheelVelocityPI notTuned; // default zero gains cannot move wheels
  notTuned.enable(true);
  assert(!notTuned.update(20.0f, 20.0f, zero, 0.1f).valid);
  std::cout << "PASS: velocity PI preview disabled-by-default, sign, saturation, fail-closed\n";
}
