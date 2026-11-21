# ESP32 GyroGuard hardware commissioning

**Software prototype only — not verified on hardware.** Use a cradle with wheels off the floor. Disconnect motor power during wiring. **V0.5 builds with `GYROGUARD_MOTOR_ENABLE=0` by default** and therefore refuses arming. Read [BENCH_PHASES.md](BENCH_PHASES.md) for staged checks. Enable the motor flag only for restrained wheels-off-ground polarity testing, with an independent power cutoff.

1. Confirm classic ESP32-WROOM-32 dev board, MPU6050, TB6612FNG, **one HC-SR04 on a servo** (e.g. SG90), 2 motors, button.
2. Verify voltage shift on the **single sonar ECHO** and a sufficiently rated **external regulated 5 V servo supply**, all grounds common. Never feed 5 V ECHO into the ESP32.
3. Place the servo at neutral with the sensor facing forward. Check clearance at 45° (right), 90° (front), 135° (left). Limit servo travel for your bracket; avoid mechanical binding.
4. Run pio run -t upload; start pio device monitor -b 115200. On boot all motors remain disabled and the MPU6050 calibrates while stationary.
5. Confirm imu=1, pitch sign (leaning forward positive), and serial sonar look= positions. A near obstacle in front should trigger a left/right sweep and a return to front; check the measured sides match physical directions.
6. With motor power isolated, deliberately unplug ECHO or move an object out of range; the sonar must show invalid ? readings, and navigation command should not move forward.
7. **Only after setting `GYROGUARD_MOTOR_ENABLE=1`, rebuilding, and restoring motor power to a restrained cradle:** with chassis in a mechanical cradle and servo clear of hands, hold the robot within ±6° of upright with pitch rate within ±12°/s continuously for at least 750 ms (check `arm_ready=1` in telemetry). Then press GPIO27 button to arm. If the button was pressed too early, release and press it again after readiness is reached. Sampling gaps over 20 ms interrupt the 750 ms stability hold. Confirm that a positive forward pitch makes both wheels drive forward. Set PITCH_SIGN, MOTOR_SIGN, and each wheel sign accordingly.
8. Check disarming on button press, lost IMU data, missed controller deadline, or tilt >32°. Add an independent hardware power cutoff before unrestricted testing.
9. Tune balancing **before enabling autonomy**. PID gains are placeholders and no wheel encoders/odometry are implemented. During servo sweep, motion intent is set to zero. This does not mean the robot is guaranteed to remain in place.
10. Autonomous mode is intentionally compiled out (GYROGUARD_AUTONOMY=0). Only enable after confirming stable closed-loop balance, wiring, motor polarity, turn sign and clearance under controlled conditions.

### Timing / limitations

- Inner balance loop target: 200 Hz, best-effort polling. Servo LEDC PWM runs independently and tick() never waits for echo via pulseIn().
- The single sonar is triggered at least 65 ms apart. Servo angle changes allow at least 220 ms settling before a new ping. Real servo speed, vibrations, mechanical backlash and reflected sonar geometry must be measured.
- Normally sonar stays centered, measuring forward. On an obstacle under 50 cm, it temporarily scans left -> right -> front and temporarily suspends navigation. A new sweep is rate-limited to 1.5 seconds.
- Sonar is neither a full collision sensor nor a substitute for odometry; side readings are snapshots. Failures may cause collisions. No certified real-time deadline guarantees.


### V0.3 software verification and bench diagnostics

Before supplying motor power, run `bash test/host/run_tests.sh` to check the host C++ controller and sonar behavior, source syntax, and Python telemetry parser. The GitHub Actions workflow also requests a real ESP32 PlatformIO firmware build; its success must be checked in the Actions tab, rather than inferred from a successful commit.

During supported wheels-off-ground tests, record serial output (115200 baud):

```bash
pio device monitor -b 115200 > bench.log
python3 tools/bench_report.py bench.log
python3 tools/bench_report.py bench.log --strict --json
```

Stop the serial monitor with Ctrl+C before reading the file. The report counts samples, motor-arming samples, arm-ready samples (when reported), scan samples, IMU faults, valid directional sonar samples, maximum absolute reported pitch and disarm reasons. With `--strict`, missing telemetry, any recorded IMU fault, or a disarm causes exit code 2. Invalid sonar echoes alone do not indicate a hardware fault (e.g., no echo due to out-of-range targets).

**Do not treat a clean bench report as proof of reliable balancing or autonomous avoidance.** Record on-hardware results for pitch/motor-sign polarity, ECHO level shifting, watchdog/disarm behavior, response to an object at measured distances and servo physical travel. Keep `GYROGUARD_AUTONOMY=0` until upright stability and navigation behavior have been validated under containment. Wheel encoders/velocity feedback are still not implemented.
