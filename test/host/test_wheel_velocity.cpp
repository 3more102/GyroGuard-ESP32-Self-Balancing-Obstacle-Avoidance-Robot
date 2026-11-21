#include <WheelVelocity.h>
#include <cassert>
#include <cmath>
#include <cstdint>
#include <iostream>
using namespace gyroguard;

int main() {
  WheelVelocityEstimator estimator;
  auto r = estimator.update(0u, 0u, 100000u);
  assert(!r.valid);
  r = estimator.update(3u, 0u, 120000u);
  assert(!r.valid); // too soon to compute a tick rate
  r = estimator.update(10u, static_cast<uint32_t>(-5), 200000u);
  assert(r.valid);
  assert(std::fabs(r.leftTicksPerSec - 100.0f) < 0.001f);
  assert(std::fabs(r.rightTicksPerSec + 50.0f) < 0.001f);

  r = estimator.update(20u, static_cast<uint32_t>(-10), 300000u);
  assert(r.valid && r.leftTicksPerSec > 0 && r.rightTicksPerSec < 0);

  r = estimator.update(30u, 0u, 650000u);
  assert(!r.valid); // long gap invalidates snapshot
  r = estimator.update(40u, 4u, 750000u);
  assert(r.valid);

  estimator.reset();
  estimator.update(0u, 0u, 0u);
  r = estimator.update(6000u, 0u, 60000u);
  assert(!r.valid); // implausible interrupt burst

  estimator.reset();
  estimator.update(0xFFFFFFFDu, 0xFFFFFFFDu, 0xFFFFFF00u);
  r = estimator.update(3u, 3u, 0x0000EA60u);
  assert(r.valid); // both edge counters and micros() wrap
  assert(r.leftTicksPerSec > 0.0f && r.leftTicksPerSec < 150.0f);
  assert(r.rightTicksPerSec > 0.0f);

  std::cout << "PASS: wheel-rate diagnostics, gaps, noise and wraps\n";
}
