# GyroGuard ESP32 — Self-Balancing Robot + Obstacle Avoidance

**Status: V0.4 software prototype; NO hardware validation, not ready for autonomous deployment.**

ESP32-WROOM-32 prototype with an MPU6050 IMU, TB6612FNG dual motor driver, two DC geared motors and **one HC-SR04 ultrasonic sensor mounted on a servo**. The design separates the inner balance loop from a conservative navigation policy. Source includes host-runnable controller tests and wiring/commissioning documentation.

## System architecture

![GyroGuard ESP32 V0.2 system block diagram](docs/assets/gyroguard-system-block-diagram.svg)

The diagram shows the **MPU6050 → ESP32 stabilization loop**, the **single HC-SR04 sensor mounted on a scanning servo**, and the **TB6612FNG driving the left/right motors**, together with manual arming and power distribution. It is a design schematic, not evidence of hardware validation. The controller uses a complementary pitch filter and a 200 Hz angle controller; obstacle avoidance is a conservative, rule-based prototype and is **disabled by default**.

## Implemented

- 200 Hz polled balance loop with MPU6050 gyro offset calibration, complementary pitch filter and angle PD control with optional I term.
- Manual arming interlock requires **750 ms of continuous healthy IMU readings** within ±6° pitch and ±12°/s pitch rate; invalid samples, excess movement and disarming clear readiness. Telemetry reports `arm_ready`. Gaps over 20 ms between IMU readings reset the 750 ms timer; the arm button is checked after the current control/deadline evaluation. This is an additional software safeguard, not a hardware emergency stop.
- Two-direction TB6612FNG motor control at 20 kHz PWM, startup-off `STBY`, physical arm/disarm button, tilt/IMU/deadline cutoffs.
- One interrupt-timed HC-SR04 and PWM servo scanning front/left/right only when a near-front obstacle appears. The 200 Hz balance loop is never blocked waiting for servo movement or ECHO; stale readings invalidate navigation.
- Conservative rule-based obstacle navigation: stop/turn toward the open side near an obstacle; slow in intermediate range; steer away from close side walls. **Default disabled**, as autonomous motion needs closed-loop wheel velocity feedback.
- Host tests for controller sign, saturation, sensor failures, servo-scan sequencing, fresh/stale directional snapshots, obstacle turns and hysteresis. V0.3 adds rejection of physically implausible/corrupt sonar ranges and cancels an active turn when side sensing becomes invalid.
- GitHub Actions CI runs C++ host safety tests, telemetry tests, firmware source syntax checks and the actual PlatformIO ESP32 build. The build publishes firmware artifacts when successful.
- Offline serial telemetry reporting (`tools/bench_report.py`) summarizes available pitch/range samples, IMU fault flags and disarm events for supervised bench commissioning.

This is **not TinyML or an AI-trained model**. TinyML is a future optional phase, once real obstacle and motion data have been gathered and a baseline has been validated.

## Requirements

- Classic ESP32-WROOM-32 development board (PlatformIO `esp32dev`)
- MPU6050 (AD0=GND), TB6612FNG, 2 DC geared motors, 1 HC-SR04 + 1 hobby servo (e.g. SG90), arm button
- Suitable battery/regulator, mechanical chassis, common ground, 5V-to-3.3V ECHO level shifting, electrical protection

See [pinout and electrical warnings](docs/PINOUT.md), then [commissioning checklist](docs/COMMISSIONING.md).

## Build and test

1. Install VS Code + PlatformIO, open this directory.
2. Upload: `pio run -t upload` (set correct serial port as needed).
3. View telemetry: `pio device monitor -b 115200`.
4. Run C++/Python host regression tests (Linux/macOS with g++, Python 3): `bash test/host/run_tests.sh`.
5. After supervised, wheels-off-ground tests, capture serial diagnostics: `pio device monitor -b 115200 > bench.log` (stop with Ctrl+C). Summarize the log with `python3 tools/bench_report.py bench.log --json`. Use `--strict` to exit with code 2 for missing telemetry, IMU fault samples or recorded disarms. **A report with no recorded faults does not establish safe hardware operation.**

The project pins the **official PlatformIO Espressif32 6.12.0** (Arduino-ESP32 2.0.x) because its `ledcSetup/ledcAttachPin` API is different from Arduino-ESP32 3.x. Do not silently bump the framework without migrating PWM calls.

## Control flow

`MPU6050 -> pitch complementary filter -> 200 Hz angle controller -> TB6612 -> wheels`

`HC-SR04 mounted on servo -> front/left/right scan scheduler -> obstacle policy -> bounded drive/turn intent -> balance controller`

By default `GYROGUARD_AUTONOMY=0` in `platformio.ini`. `GYROGUARD_AUTONOMY=1` enables experimental navigation **only after safe hardware validation**. Motor sign, mount orientation, controller gains and motor PWM cap are in `include/Config.h` and `lib/GyroCore/src/GyroCore.h`.

## Continuous integration and fault hardening

- **Host tests:** `.github/workflows/host-tests.yml` runs C++ controller, navigation, servo scheduler, and Arduino-interface syntax checks.
- **ESP32 compile check:** `.github/workflows/firmware-build.yml` compiles classic ESP32 firmware with PlatformIO Core 6.1.18 and pinned `espressif32 @ 6.12.0`; it does **not** flash or test physical hardware.
- Navigation control now refuses non-finite drive and turn intent rather than propagating invalid values through the balance controller. The sonar ISR latches the first completed ECHO pulse to reject later spurious edges before the main loop consumes it.
- **Important:** Hardware power cutoff, motor polarity, stable upright control and encoder-based velocity regulation still require verification before autonomous driving.

## V0.4 manual-arm safety improvement

- Before pressing the arm button, hold the chassis still and upright until telemetry shows `arm_ready=1`.
- A fall, IMU fault, excessive pitch rate or missed deadline clears the arming readiness timer. An early button press is refused; release and press again once ready.
- This is a **software-only interlock**. It does not replace an independent motor power cutoff, cradle tests, verified controller signs or motor encoders.

## Roadmap

1. **Bench validation:** hardware wiring, PWM polarity, IMU pitch direction, sensor range tests, fail-stop behavior.
2. **Balance tuning:** fit Kp/Kd to measured geometry and motor behavior; verify timing jitter.
3. **Add motor encoders + velocity PID:** reliable upright position hold, commanded forward speed and safer turns.
4. **Nav validation:** timed test course, side obstacle robustness, fault injection, collision metrics.
5. **Optional TinyML:** collect labeled sonar/odometry data, train and evaluate offline, quantize/measure inference latency vs 200 Hz budget.

## References

- Espressif [ESP32 datasheet](https://documentation.espressif.com/esp32_datasheet_en.html) and [GPIO rules](https://docs.espressif.com/projects/esp-idf/en/release-v5.5/esp32/api-reference/peripherals/gpio.html).
- PlatformIO [Espressif32 release 6.12.0](https://github.com/platformio/platform-espressif32/releases/tag/v6.12.0).
- Espressif [Arduino PWM API migration](https://docs.espressif.com/projects/arduino-esp32/en/latest/migration_guides/2.x_to_3.0.html).

## Single-servo sonar scan design (V0.2)

- Keep HC-SR04 facing **front at 90°** for frequent forward distance readings.
- If valid front distance is below 50 cm, suspend autonomous motion intent and swing the servo to **left 135°**, then **right 45°**, then back to **front 90°**.
- Wait for the servo to settle at each angle and trigger the HC-SR04 at least 65 ms apart. Sonar echoes are captured by GPIO interrupts; no blocking `pulseIn`.
- Only allow navigation decisions again after the center reading returns. Choose the side with more clearance if both side readings are valid and fresh; missing/stale forward readings stop navigation intent.
- **Important:** Servo movement induces dynamic vibrations and a single transducer cannot see all directions simultaneously. This is a rule-based prototype, not an AI model or validated autonomous system.
## Optional wheel-encoder diagnostics (experimental)

The `esp32dev-encoder-monitor` PlatformIO environment adds **monitoring only**, not closed-loop velocity PID or autonomous movement. It is off in the default `esp32dev` build. The monitor counts rising and falling transitions of each encoder's A channel while sampling B for direction (2× decoding). The firmware reports `enc_valid`, `left_tps`, and `right_tps` every 200 ms. Rates are **ticks per second**, not RPM.

```bash
pio run -e esp32dev-encoder-monitor
pio run -e esp32dev-encoder-monitor -t upload
pio device monitor -b 115200
```

Use GPIO35/A + GPIO13/B for left, and GPIO36/A + GPIO14/B for right. Signals must be 3.3 V safe; GPIO35/36 are input-only and have **no built-in pull-ups**, so add external 3.3 V pull-ups if your encoder needs them. With motor supply disconnected, spin each wheel by hand to confirm its channel and sign before enabling motor power. Reverse `ENC_LEFT_SIGN` or `ENC_RIGHT_SIGN` only after testing. **Zero ticks with `enc_valid=1` does not prove that an encoder is connected.** This software cannot hold position or control velocity until the encoder electrical interface, counts-per-wheel-revolution and an outer-loop controller have been validated on physical hardware.
