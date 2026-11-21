# GyroGuard V0.6 — Motor-lock and staged bench checks

> **UNVERIFIED HARDWARE.** The software prototype does not establish safe balancing or obstacle avoidance. Do not enable autonomous movement. This document is a procedure, not a completed test record.

## 1. Electrical inspection — power disconnected

- Verify ESP32-WROOM-32, MPU6050, TB6612FNG, two geared motors, one HC-SR04 on an SG90-class servo, and button against [PINOUT.md](PINOUT.md).
- Disconnect **TB6612 VM** and independently check motor voltage/current specifications. Do not connect a 12 V battery directly to the ESP32. Follow [POWER_WIRING.md](POWER_WIRING.md).
- Confirm TB6612 VCC uses a compatible logic rail; servo has its own suitably rated regulated 5 V supply; common ground is present.
- Level shift HC-SR04 5 V ECHO to ESP32-compatible voltage before GPIO34.
- Add a reachable independent physical cutoff for motor VM. Keep wheels off the floor and moving parts clear.

## 2. Sensor-only firmware — VM disconnected

The default `platformio.ini` settings are:

```ini
-D GYROGUARD_AUTONOMY=0
-D GYROGUARD_MOTOR_ENABLE=0
```

```bash
pio run -e esp32dev
pio run -e esp32dev -t upload
pio device monitor -b 115200
```

- Confirm serial banner `Motor outputs: LOCKED (sensor-only)`, and each telemetry line reports `motor_lock=1`, `arm=0`, `cmd_l=0`, `cmd_r=0`.
- On pressing GPIO27 after steady IMU readiness, observe `ARM REFUSED: motor outputs are compile-locked`; do not mistake firmware reporting for verification of a physical motor cutoff.
- Read `imu=1`; carefully tilt the supported chassis in both directions and verify the measured pitch sign and `rate_dps` against real motion.
- With the sonar normally facing 90° forward, bring a target near it to request left 135°, right 45°, then return to center. Inspect bracket clearance and verify directional ranges; protect the servo from binding.
- Observe that unplugged/out-of-range ECHO reports invalid values, not invented distances. Log power-rail stability and `ctl_misses` while the servo moves.
- Capture data at 115200 baud and review it with `python3 tools/bench_report.py bench.log --strict`.

**Do not proceed** if any sensor direction, voltage, software arm refusal or physical motor isolation is inconsistent.

## 3. Restrained motor direction test — only after step 2

1. **Disconnect motor VM**, edit `platformio.ini` from `GYROGUARD_MOTOR_ENABLE=0` to `GYROGUARD_MOTOR_ENABLE=1`, rebuild and reflash. Do not enable `GYROGUARD_AUTONOMY`.
2. Make sure the independent VM cutoff is accessible and that the chassis is mechanically restrained with wheels off the floor.
3. Restore motor VM only for brief supervised tests; verify the intended sign of a small forward pitch yields a forward wheel correction. If either wheel responds incorrectly, disconnect VM before modifying signs.
4. Verify manual disarm, tilt disarm, missed-deadline handling and MPU6050-loss handling without relying on the software button as the sole emergency stop.
5. Record measured rail voltages, stall/current estimates for the actual motors, timing and observed motor behavior in [BENCH_VALIDATION_RECORD.md](BENCH_VALIDATION_RECORD.md).

**Software safety is not hardware safety.** All wheel polarity, PID stability, physical cutoff and obstacle-clearance tests remain unverified until measured on the actual robot.
