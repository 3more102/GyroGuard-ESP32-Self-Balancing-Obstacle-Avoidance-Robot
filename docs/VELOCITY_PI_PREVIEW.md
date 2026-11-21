# V0.6 — Wheel velocity PI preview (not enabled on robot)

**Status: host-tested controller component only. No hardware calibration, wheel-speed control, or autonomous actuation is claimed.** The implementation is in lib/GyroCore/src/WheelVelocityPI.h; it is deliberately NOT called from src/main.cpp. The firmware's balance loop and motor commands are unchanged. GYROGUARD_AUTONOMY=0 remains the default.

## Why a separate outer loop?

The existing inner MPU6050 pitch stabilizer runs at 200 Hz. The optional wheel encoders estimate **ticks per second**, not metres per second. An outer loop could later compare requested and measured wheel speeds at around 10 Hz and produce limited motion corrections while the inner pitch controller keeps the robot upright. That cascade must be designed and physically tested before linking these components; directly applying PI preview outputs to motor PWM would bypass the stabilizing controller and is not supported.

## Implemented experimental component

- Two independent PI preview channels for left and right wheel rate.
- Defaults to **zero PI gains** and **disabled**; there is no motor-actuation path from this module.
- Rejects non-finite or absent encoder rates, excessive targets/rates, invalid configuration, or update intervals outside 20–250 ms.
- A rejected update clears integrators and **latches disabled** until intentionally re-enabled.
- Caps preview output and uses conditional integration to avoid integral windup while saturated.
- Has C++ host tests for disabled state, command signs, saturation, anti-windup, NaN, range limits, stale/invalid data, and reset.

The numerical limits in the example struct are *software envelopes*, not values measured or approved for any particular motors, wheels or batteries. The unit-test PI gains are synthetic test inputs, **not recommended tuning values**.

## Encoder calibration: required before integration

1. Keep the **motor power physically disconnected**. Wire the two encoders according to [PINOUT.md](PINOUT.md), with 3.3 V-safe signals and external pull-ups where required.
2. Build and upload the optional encoder-monitor PlatformIO environment. In a safe cradle, rotate the **left wheel forward exactly one full output-shaft revolution by hand** and examine encoder direction and tick rate during motion. Do the same for right. The current firmware prints rates, not accumulated counters; therefore measuring exact ticks per revolution requires a future raw-count telemetry addition or a separate instrument. **Do not infer CPR from rate alone.**
3. Verify positive forward tick rate and opposite sign for reverse by hand-turning each wheel. Change encoder sign settings in Config.h only after measurement.
4. Record actual output-shaft ticks per wheel revolution, wheel radius, gear ratio, encoder signal quality and 3.3 V interface in the bench record. With this project's 2× A-edge counting scheme, do not assume a motor-shaft PPR specification equals output-wheel ticks per revolution.
5. Independently validate motor direction, IMU sign, physical cutoff, stability, balance gains, power distribution and timing before floor tests. Only after these pass should the preview be considered for a bounded, supervised experimental outer loop.

## Software-only check

Run: bash test/host/run_tests.sh

The tests validate code behavior on a host machine. They do not validate encoder electrical wiring, balancing, collision avoidance or a closed-loop robot. Keep autonomous movement disabled until physical safety gates pass.
