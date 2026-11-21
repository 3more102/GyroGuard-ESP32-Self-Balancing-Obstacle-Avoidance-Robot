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
  // Fault injection: corrupt navigation commands must fail closed.
  for (float bad : {NAN, INFINITY, -INFINITY}) {
    u = c.update(0.0f, 0.0f, bad, 0.0f, 0.005f);
    assert(u.left == 0.0f && u.right == 0.0f);
    u = c.update(0.0f, 0.0f, 0.0f, bad, 0.005f);
    assert(u.left == 0.0f && u.right == 0.0f);
  }
  u = c.update(0.0f, 0.0f, 0.0f, 0.0f, 0.005f);
  assert(u.left == 0.0f && u.right == 0.0f); // recovery after faults
  c.reset();

  // Startup button held LOW cannot create an arm event.
  ArmButtonGuard button;
  assert(!button.update(true, 0u));
  assert(!button.update(true, 1000u));
  assert(!button.update(false, 1001u));
  assert(!button.update(false, 1039u));
  assert(!button.update(false, 1041u)); // stable release
  assert(!button.update(true, 1050u));
  assert(!button.update(false, 1055u)); // contact bounce
  assert(!button.update(true, 1060u));
  assert(!button.update(true, 1099u));
  assert(button.update(true, 1100u)); // fresh stable press
  assert(!button.update(true, 1600u)); // held button is not a repeat
  assert(!button.update(false, 1601u));
  assert(!button.update(false, 1641u));
  assert(!button.update(true, 1650u));
  assert(button.update(true, 1690u)); // intentional new press

  ArmButtonGuard releasedAtBoot;
  assert(!releasedAtBoot.update(false, 0u));
  assert(!releasedAtBoot.update(false, 41u));
  assert(!releasedAtBoot.update(true, 45u));
  assert(releasedAtBoot.update(true, 85u));

  // Debounce timestamps remain correct across uint32_t rollover.
  ArmButtonGuard rolloverButton;
  assert(!rolloverButton.update(false, 0xFFFFFFE0u));
  assert(!rolloverButton.update(false, 0x00000011u));
  assert(!rolloverButton.update(true, 0x00000020u));
  assert(rolloverButton.update(true, 0x00000049u));

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
  // Corrupted but 'valid' ranges must not authorize autonomous motion.
  intent = nav.update({{INFINITY, true}, {100, true}, {100, true}});
  assert(intent.drive == 0.0f && intent.turn == 0.0f);
  intent = nav.update({{400, true}, {100, true}, {100, true}});
  assert(intent.drive == 0.0f && intent.turn == 0.0f);
  intent = nav.update({{35, true}, {NAN, true}, {90, true}});
  assert(intent.drive == 0.0f && intent.turn == 0.0f);
  intent = nav.update({{35, true}, {90, true}, {INFINITY, true}});
  assert(intent.drive == 0.0f && intent.turn == 0.0f);
  intent = nav.update({{35, true}, {90, true}, {60, true}});
  assert(intent.turn < 0.0f);
  // Losing a side sensor mid-turn must cancel the old turn command.
  intent = nav.update({{45, true}, {NAN, true}, {60, true}});
  assert(intent.drive == 0.0f && intent.turn == 0.0f);
  intent = nav.update({{120, true}, {90, true}, {90, true}});
  assert(intent.drive > 0.0f && intent.turn == 0.0f);
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

  // A 750ms hold requires continuous 200Hz *fresh* samples.
  ArmingInterlock interlock(6.0f, 12.0f, 750u);
  assert(!interlock.ready());
  for (uint32_t t = 1000u; t < 1750u; t += 5u) {
    assert(!interlock.observe(1.0f, 1.0f, true, t));
  }
  assert(interlock.observe(1.0f, 1.0f, true, 1750u));
  assert(interlock.ready());
  assert(!interlock.observe(1.0f, 13.0f, true, 1755u));
  assert(!interlock.ready()); // excessive rate resets readiness
  for (uint32_t t = 2000u; t < 2400u; t += 5u) {
    assert(!interlock.observe(1.0f, 1.0f, true, t));
  }
  assert(!interlock.observe(1.0f, 1.0f, true, 2425u));
  assert(!interlock.ready()); // >20ms missing samples restart the hold
  assert(!interlock.observe(NAN, 0.0f, true, 2430u));
  assert(!interlock.observe(0.0f, INFINITY, true, 2435u));
  assert(!interlock.observe(0.0f, 0.0f, false, 2440u));
  assert(!interlock.observe(7.0f, 0.0f, true, 2445u));
  interlock.reset();
  for (uint32_t i = 0u; i < 150u; ++i) {
    assert(!interlock.observe(0.0f, 0.0f, true,
                              0xFFFFFF00u + i * 5u));
  }
  assert(interlock.observe(0.0f, 0.0f, true,
                            0xFFFFFF00u + 150u * 5u));
  assert(interlock.ready()); // uint32_t millis() wrap
  interlock.reset();
  assert(!interlock.ready());

  std::cout << "PASS: balance/navigation, MPU6050 pitch, arming interlock, servo-scan and stale-data checks\n";
}
