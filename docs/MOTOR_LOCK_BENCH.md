# V0.5 motor-output lock and staged bench commissioning

**Hardware status: untested.** The normal PlatformIO build is a **sensor-only**
commissioning image. It sets `GYROGUARD_MOTOR_ENABLE=0` and
`GYROGUARD_AUTONOMY=0`. TB6612 STBY starts LOW; arming is refused, and
`MotorDriver::set()` is compiled to call `disable()` rather than write any
nonzero PWM or set STBY HIGH. This is a software safeguard, **not** an
independent emergency stop or a certification of power safety.

## Stage A: sensor-only (default, motor supply disconnected)

1. Physically disconnect battery/motor VM from the TB6612 before wiring.
   Verify the ESP32 has a properly regulated supply, servo has a suitable
   external regulated 5 V supply, grounds are common, and HC-SR04 ECHO is
   shifted to 3.3 V or below. Never apply 12 V to ESP32 GPIO/3V3/5V.
2. Use `pio run -e esp32dev` then `pio run -e esp32dev -t upload`.
   Watch the 115200 baud serial output: it must print
   `Motor outputs: LOCKED (compile-time)` and status `motor_en=0`.
3. Check stationary MPU6050 calibration, pitch sign, sensor distance
   validity and left/front/right scan direction (one HC-SR04 on servo).
   Pressing the arm button must print an explicit motor-lock refusal.
4. With **VM still disconnected**, verify GPIO23/STBY stays LOW and both
   PWM outputs remain inactive using a suitable meter/scope. This is
   a required **physical** check; unit tests do not establish it.
5. Save a serial log and run `python3 tools/bench_report.py bench.log --strict`.
   A passing parser result is only a software diagnostic.

## Stage B: restrained motor polarity testing (explicit opt-in)

**Prerequisites:** stable, measured rails, secure wheels-off-ground cradle,
correct motor voltage/current limits, inline fuse/protection and a **physical,
independent power cutoff** on the motor supply. Do not proceed on a desk or
with free-spinning exposed mechanisms.

1. In `platformio.ini`, change ONLY
   `-D GYROGUARD_MOTOR_ENABLE=0` to
   `-D GYROGUARD_MOTOR_ENABLE=1`. Keep
   `GYROGUARD_AUTONOMY=0`. Check the firmware revision and build flags.
2. Rebuild/reflash: `pio run -e esp32dev -t clean`, then
   `pio run -e esp32dev -t upload`. Confirm
   `Motor outputs: UNLOCKED (compile-time)` and `motor_en=1` over UART.
3. Start with a restrained chassis, independently cut motor power before
   approaching the wheels, ensure the button is released and hold the
   chassis upright while confirming `arm_ready=1`. Press the button once
   to arm. Check the corrective wheel direction with only brief,
   supervised tests. Any inverted correction means **cut motor power**.
4. Test manual disarm, loss-of-IMU disarm, controller timing fault,
   tilt cutoff, rail sag and cutoff switch before tuning balancing.
5. Restore `GYROGUARD_MOTOR_ENABLE=0` and rebuild for routine
   sensor diagnostics.

No closed-loop wheel velocity PID or floor-level autonomous motion is
verified. Optional encoder readings are diagnostic only. Do not enable
autonomy until both the balance system and independent safety measures
have been physically validated and documented in
[BENCH_VALIDATION_RECORD.md](BENCH_VALIDATION_RECORD.md).

## Reproducible software checks

```sh
bash test/host/run_tests.sh
pio run -e esp32dev
pio run -e esp32dev-encoder-monitor
```

Host tests compile a locked and deliberately unlocked motor path with mock
GPIO/PWM functions. The normal ESP32 CI build validates the locked image.
Neither test proves electrical safety, motor polarity or successful
balancing on a real robot.
