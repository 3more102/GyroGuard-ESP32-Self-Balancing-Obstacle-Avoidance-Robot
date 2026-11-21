# GyroGuard V0.5 — staged bench commissioning

**Status:** hardware unverified. This is a procedure, not a successful test report. Use a mechanically restrained chassis and a physical motor-power disconnect. A software lock is **not** an emergency stop.

## A. Electrical inspection — power OFF

- [ ] Verify the exact ESP32 board and all wiring against [PINOUT.md](PINOUT.md).
- [ ] Inspect motor wiring for shorts, measure rail polarity and check that all grounds share a reference.
- [ ] Feed the TB6612 VCC logic supply from a compatible 3.3 V rail; VM must match the **specific motor rating** and remain isolated for stages B/C until explicitly required.
- [ ] Supply the SG90-class servo from a suitably rated regulated 5 V rail; do **not** draw its motor current from an ESP32 GPIO or its 3.3 V pin.
- [ ] Confirm the 5 V HC-SR04 ECHO has a divider/level shifter before GPIO34.
- [ ] Establish an **independent physical power disconnect** for the motor VM supply.
- [ ] Secure the wheels clear of the work surface and keep hands out of the servo and drivetrain.

## B. Sensor-only build — motor VM DISCONNECTED

Default build flags in `platformio.ini`:

```ini
-D GYROGUARD_AUTONOMY=0
-D GYROGUARD_MOTOR_ENABLE=0
```

1. Build and flash with `pio run -e esp32dev -t upload`.
2. Start `pio device monitor -b 115200`. Confirm the banner says `Motor output: LOCKED (sensor-only commissioning)`.
3. Wait for IMU initialization/calibration while the chassis is stationary. Record `imu=1` and the pitch estimate. Tilting the supported chassis forward should **increase** the reported pitch for the configured axes. Confirm the physical sign, do not assume it.
4. Hold the IMU still/upright long enough for `arm_ready=1`. Press the arm button: the firmware should print `ARM REFUSED: motor output locked (GYROGUARD_MOTOR_ENABLE=0)` and `arm` must remain zero.
5. Verify the **single** sonar remains forward normally. Approach it with a target, observe `front` ranges and the left → right → front sweep below the configured 50 cm threshold. Check labels match actual physical directions.
6. Disconnect/obstruct the ECHO input and check invalid sonar reports. Do not expect the sonar to identify soft, angled or out-of-range objects reliably.
7. Review boot stability, sensor sampling, servo clearance and power-rail voltage under servo motion. Keep the motor VM physically disconnected during **all** B tests.

**Pass gate to C:** correctly oriented pitch, stable IMU, sensible tagged sonar readings, acceptable servo travel and **no motor arming** in the locked firmware. Record observations; do not claim success merely because the firmware builds.

## C. Manual motor polarity — restrained wheels OFF the floor

**Only after B passes**, leave `GYROGUARD_AUTONOMY=0`, change `GYROGUARD_MOTOR_ENABLE=1` locally, rebuild and reflash. Prepare a physical VM cutoff before restoring the motor supply. The software arming checks still require 750 ms upright stability and a deliberate button press.

- [ ] With an emergency disconnect reachable, confirm the left and right motor direction wiring at brief low-demand intervals and verify the sign settings in `include/Config.h`.
- [ ] Confirm that a small **forward** chassis tilt produces both wheels' **forward** corrective rotation. If the response is reversed, **disconnect VM** before editing signs or wiring.
- [ ] Confirm the arm button disarms and `STBY` drops LOW; inspect the physical action, not just a serial message.
- [ ] Confirm disarm on tilt/fall and on disconnect of the IMU (under restrained, supervised conditions).
- [ ] Record peak current, any controller-deadline failures and supply brownouts. Do not infer safety from `arm_ready` alone.

**Pass gate to D:** correct polarity and confirmed disarm hardware behavior. If not observed, revert to sensor-only firmware.

## D. Supported balance trials — autonomy still OFF

- Start with mechanically bounded short sessions and measured, recorded pitch response; the example PID gains in source are placeholders.
- Tune proportional and derivative gains incrementally. Validate motor power headroom and behavior at both tilt polarities.
- Add independent motor-power cutoff, calibrated encoder feedback and a closed-loop wheel-velocity layer **before** interpreting sustained upright/position holding as a navigation-ready robot.
- Do not perform unrestricted floor trials or set `GYROGUARD_AUTONOMY=1` based solely on software tests.

## E. Log evidence

Capture a diagnostic log:

```bash
pio device monitor -b 115200 > bench.log
# Stop monitor with Ctrl+C.
python3 tools/bench_report.py bench.log --json
python3 tools/bench_report.py bench.log --strict
```

Record test date, board, power rails, motor model/rated voltage, IMU mounting, servo geometry, measurements, any faults, corrective changes and video if available. Host tests and CI validate software compilation/selected logic only; physical balance, current limits and collision safety remain **unverified** until measured.

**12 V caution:** Do not connect a 12 V battery directly to ESP32 3.3 V/5 V pins. Even if the TB6612 motor-supply range accepts a given voltage, the motor's **own rated voltage/current** and thermal limits still control whether it is suitable. The baseline project describes a 2S Li-ion pack and regulated logic/servo power.
