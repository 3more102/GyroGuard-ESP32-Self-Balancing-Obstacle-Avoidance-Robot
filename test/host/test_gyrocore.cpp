#include "GyroCore.h"
#include <cassert>
#include <cmath>
#include <iostream>
using namespace gyroguard;

int main() {
  const float tenDegrees = 10.0f * 3.14159265f / 180.0f;
  assert(std::fabs(pitchFromAccelXZ(-std::sin(tenDegrees), std::cos(tenDegrees)) - 10.0f) < 0.001f);
  assert(std::fabs(pitchFromAccelXZ(+std::sin(tenDegrees), std::cos(tenDegrees)) + 10.0f) < 0.001f);
  BalanceController c;
  auto u = c.update(0.0f, 0.0f, 0.0f, 0.0f, 0.005f);
  assert(u.left == 0.0f && u.right == 0.0f);
  u = c.update(8.0f, 0.0f, 0.0f, 0.0f, 0.005f);
  assert(u.left > 0 && u.right > 0); // forward tilt = forward correction
  u = c.update(-8.0f, 0.0f, 0.0f, 0.0f, 0.005f);
  assert(u.left < 0 && u.right < 0);
  u = c.update(0.0f, 0.0f, 0.0f, 1.0f, 0.005f);
  assert(u.left > 0 && u.right < 0); // pivot right
  u = c.update(10000.0f, 0.0f, 0.0f, 0.0f, 0.005f);
  assert(fabsf(u.left) <= 0.70f && fabsf(u.right) <= 0.70f);
  u = c.update(NAN, 0.0f, 0.0f, 0.0f, 0.005f);
  assert(u.left == 0 && u.right == 0); // invalid samples = zero output
  u = c.update(2.0f, 0.0f, 0.0f, 0.0f, 0.100f);
  assert(u.left == 0 && u.right == 0); // missed deadline
  c.reset();

  ObstacleNavigator nav;
  auto intent = nav.update({{0, false}, {100, true}, {100, true}});
  assert(intent.drive == 0 && intent.turn == 0); // fail closed
  intent = nav.update({{120, true}, {100, true}, {100, true}});
  assert(intent.drive > 0 && intent.turn == 0);
  intent = nav.update({{35, true}, {90, true}, {50, true}});
  assert(intent.drive == 0 && intent.turn < 0); // choose left
  intent = nav.update({{55, true}, {50, true}, {90, true}});
  assert(intent.turn < 0); // hysteresis, keep turning left
  intent = nav.update({{95, true}, {100, true}, {100, true}});
  assert(intent.drive > 0 && intent.turn == 0);
  intent = nav.update({{35, true}, {20, true}, {20, true}});
  assert(intent.drive == 0 && intent.turn == 0); // no clear turn route
  intent = nav.update({{35, true}, {100, false}, {100, true}});
  assert(intent.drive == 0 && intent.turn == 0); // unknown side sensor
  intent = nav.update({{120, true}, {15, true}, {90, true}});
  assert(intent.turn > 0); // steer away from left wall
  intent = nav.update({{120, true}, {90, true}, {15, true}});
  assert(intent.turn < 0); // steer away from right wall
  // One HC-SR04 follows a servo schedule: front -> left -> right -> front.
  ServoScanScheduler scan;
  assert(scan.target() == LookDirection::Front);
  assert(!scan.scanning());
  assert(!scan.readings(0).front.valid);
  scan.measurement(LookDirection::Front, 120.0f, true, 100);
  assert(!scan.scanning());
  assert(scan.readings(101).front.valid);
  scan.measurement(LookDirection::Front, 32.0f, true, 200);
  assert(scan.scanning());
  assert(scan.target() == LookDirection::Left);
  // Ignore a delayed echo from another orientation.
  scan.measurement(LookDirection::Right, 100.0f, true, 210);
  assert(scan.target() == LookDirection::Left);
  scan.measurement(LookDirection::Left, 85.0f, true, 460);
  assert(scan.target() == LookDirection::Right);
  scan.measurement(LookDirection::Right, 115.0f, true, 710);
  assert(scan.target() == LookDirection::Front);
  assert(scan.scanning()); // no driving while the servo points away from front
  scan.measurement(LookDirection::Front, 31.0f, true, 960);
  assert(!scan.scanning());
  auto snap = scan.readings(970);
  assert(snap.front.valid && snap.left.valid && snap.right.valid);
  ObstacleNavigator singleNav;
  auto steer = singleNav.update(snap);
  assert(steer.drive == 0 && steer.turn > 0); // prefer right with larger clearance
  // A close front echo inside the scan rate limit must not restart sweeping.
  scan.measurement(LookDirection::Front, 30.0f, true, 1000);
  assert(!scan.scanning());
  // Front ages out quickly; cached side samples remain for bounded time.
  snap = scan.readings(1400);
  assert(!snap.front.valid && snap.left.valid && snap.right.valid);
  assert(singleNav.update(snap).drive == 0);
  snap = scan.readings(3000);
  assert(!snap.left.valid && !snap.right.valid);
  scan.measurement(LookDirection::Front, 25.0f, true, 1800);
  assert(scan.scanning() && scan.target() == LookDirection::Left);
  scan.measurement(LookDirection::Left, 0.0f, false, 2050);
  scan.measurement(LookDirection::Right, 90.0f, true, 2300);
  scan.measurement(LookDirection::Front, 30.0f, true, 2550);
  snap = scan.readings(2560);
  assert(!snap.left.valid && snap.right.valid);
  singleNav.reset();
  assert(singleNav.update(snap).turn == 0); // missing side -> fail closed
  scan.measurement(LookDirection::Front, NAN, true, 2600);
  assert(!scan.readings(2601).front.valid);
  scan.reset();
  assert(!scan.scanning() && !scan.readings(3000).front.valid);
  // uint32_t millisecond wrap must not make a recent echo instantly stale.
  scan.measurement(LookDirection::Front, 150.0f, true, 0xFFFFFFF0u);
  assert(scan.readings(0x00000020u).front.valid);
  assert(!scan.readings(0x00000320u).front.valid);

  std::cout << "PASS: balance/navigation, MPU6050 pitch, servo-scan and stale-data checks\n";
}
