# ESP32 GyroGuard hardware commissioning

**Software prototype only — not verified on hardware.** Use a cradle with wheels off the floor. Disconnect motor power during wiring.

**Default image is sensor-only:** `GYROGUARD_MOTOR_ENABLE=0` in `platformio.ini` prevents software arming and motor output; `GYROGUARD_AUTONOMY=0` keeps navigation disabled. Servo motion and sensor telemetry still run. Install an independent, accessible hardware cutoff on motor power before any deliberate motor-unlocked build; do not treat the firmware lock as an emergency stop. Do not change the motor flag until pitch axes, sonar direction, logic power and ECHO level shifting are checked with motor VM disconnected.

1. Confirm classic ESP32-WROOM-32 dev board, MPU6050, TB6612FNG, **one HC-SR04 on a servo** (e.g. SG90), 2 motors, button.
2. Verify voltage shift on the **single sonar ECHO** and a sufficiently rated **external regulated 5 V servo supply**, all grounds common. Never feed 5 V ECHO into the ESP32.
3. Place the servo at neutral with the sensor facing forward. Check clearance at 45° (right), 90° (front), 135° (left). Limit servo travel for your bracket; avoid mechanical binding.
4. Run `pio run -t upload`; start `pio device monitor -b 115200`. The default firmware remains motor-locked and the MPU6050 calibrates while stationary.
5. Confirm imu=1, pitch sign (leaning forward positive), and serial sonar look= positions. A near obstacle in front should trigger a left/right sweep and a return to front; check the measured sides match physical directions.
6. With motor power isolated, deliberately unplug ECHO or move an object out of range; the sonar must show invalid ? readings, and navigation command should not move forward.
7. With motor power still disconnected, verify the GPIO27 button **cannot arm** default locked firmware, including a button held down at boot. A stable release followed by a fresh 40 ms press is required for any arming attempt. Only after the electrical checks, an independent power cutoff and a mechanically restrained wheels-up setup, deliberately compile `GYROGUARD_MOTOR_ENABLE=1` for the supervised polarity test. Hold the chassis within ±6° pitch and ±12°/s pitch rate for at least 750 ms (`arm_ready=1`), release the button for at least 40 ms, then press for at least 40 ms. IMU sampling gaps above 20 ms restart the readiness timer. Confirm corrective wheel directions with minimal power, and verify `PITCH_SIGN`, `MOTOR_SIGN`, and wheel signs before further tests.
8. Check disarming on button press, lost IMU data, missed controller deadline, or tilt >32°. Add an independent hardware power cutoff before unrestricted testing.
9. Tune balancing **before enabling autonomy**. PID gains are placeholders; optional encoder telemetry is **diagnostic only** and no closed-loop wheel velocity or odometry controller is implemented. During servo sweep, motion intent is set to zero. This does not mean the robot is guaranteed to remain in place.
10. Autonomous mode is intentionally compiled out (GYROGUARD_AUTONOMY=0). Only enable after confirming stable closed-loop balance, wiring, motor polarity, turn sign and clearance under controlled conditions.

### Timing / limitations

- Inner balance loop target: 200 Hz, best-effort polling. Servo LEDC PWM runs independently and tick() never waits for echo via pulseIn().
- The single sonar is triggered at least 65 ms apart. Servo angle changes allow at least 220 ms settling before a new ping. Real servo speed, vibrations, mechanical backlash and reflected sonar geometry must be measured.
- Normally sonar stays centered, measuring forward. On an obstacle under 50 cm, it temporarily scans left -> right -> front and temporarily suspends navigation. A new sweep is rate-limited to 1.5 seconds.
- Sonar is neither a full collision sensor nor a substitute for odometry; side readings are snapshots. Failures may cause collisions. No certified real-time deadline guarantees.


### V0.5 software verification and bench diagnostics

Before supplying motor power, run `bash test/host/run_tests.sh` to check the host C++ controller and sonar behavior, source syntax, and Python telemetry parser. The GitHub Actions workflow also requests a real ESP32 PlatformIO firmware build; its success must be checked in the Actions tab, rather than inferred from a successful commit.

During supported wheels-off-ground tests, record serial output (115200 baud):

```bash
pio device monitor -b 115200 > bench.log
python3 tools/bench_report.py bench.log
python3 tools/bench_report.py bench.log --strict --json
```

Stop the serial monitor with Ctrl+C before reading the file. The report counts samples, motor-arming samples, arm-ready samples (when reported), scan samples, IMU faults, valid directional sonar samples, maximum absolute reported pitch and disarm reasons. New V0.5 firmware also reports shortest and longest observed controller start-to-start period within each 200 ms log window (`ctl_min_us`, `ctl_max_us`), control cycles (`ctl_n`), cumulative missed controller deadlines (`ctl_misses`), pitch rate (`rate_dps`) and last commanded normalized motor efforts (`cmd_l`, `cmd_r`). These are diagnostics, not direct velocity or electrical-current measurements. With `--strict`, missing telemetry, any recorded IMU fault, disarm, or a reported missed control deadline causes exit code 2. Invalid sonar echoes alone do not indicate a hardware fault (e.g., no echo due to out-of-range targets).

**Do not treat a clean bench report as proof of reliable balancing or autonomous avoidance.** Record on-hardware results for pitch/motor-sign polarity, ECHO level shifting, watchdog/disarm behavior, response to an object at measured distances and servo physical travel. Keep `GYROGUARD_AUTONOMY=0` until upright stability and navigation behavior have been validated under containment. Wheel encoders/velocity feedback are still not implemented.
