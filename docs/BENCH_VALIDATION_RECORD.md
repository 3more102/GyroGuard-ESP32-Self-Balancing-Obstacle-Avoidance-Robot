# GyroGuard V0.5 — physical bench validation record

**Status: NOT TESTED.** Fill from measured results only; passing C++/CI tests does not count as validation on the robot.

## Hardware configuration

| Field | Record actual value / photo or log reference |
|---|---|
| Date, firmware branch + SHA | TBD |
| Exact ESP32 DevKit model | TBD |
| Battery chemistry/cell count; measured min/max voltage | TBD |
| H-bridge board and motor part numbers | TBD |
| Measured running and stall currents per motor | TBD |
| 5 V regulator(s), output and current rating | TBD |
| MPU6050 breakout, axis orientation | TBD |
| Servo model, mechanical travel | TBD |
| Physical cutoff and fuse/protection | TBD |

## Safety gates (all must be physically tested before floor movement)

| Test | Method | Result / measured data | Evidence |
|---|---|---|---|
| 1. Motor power disconnected at boot | Boot; monitor GPIO23/STBY and wheels | NOT TESTED | |
| 2. Rail checks | Meter: VM, 5V, 3V3 at idle and load | NOT TESTED | |
| 3. HC-SR04 ECHO shift | Meter/scope confirm GPIO34 stays <=3.3 V | NOT TESTED | |
| 4. IMU stationary startup | Boot upright without movement; inspect imu= and pitch | NOT TESTED | |
| 5. Positive pitch -> forward correction | Restrained wheels-off-ground test | NOT TESTED | |
| 6. Left/right motor direction | Restrained test, record sign changes | NOT TESTED | |
| 7. Stable arming threshold | Wait >=750 ms upright; verify refusal before ready | NOT TESTED | |
| 8. Button disarm | Arm with cradle, press again | NOT TESTED | |
| 9. Physical motor cutoff | Demonstrate motor supply removal independent of MCU | NOT TESTED | |
| 10. Tilt >32 degrees | Restrained test, expect immediate software disarm | NOT TESTED | |
| 11. IMU disconnect / timing fault | Fault-injection in safe cradle; confirm STBY low | NOT TESTED | |
| 12. One-servo sonar sweep | Target front <50 cm; verify left/right/front order | NOT TESTED | |
| 13. 5 V servo transient | Observe MCU reboots, rail sag during motion | NOT TESTED | |
| 14. Sustained balance | Tethered/cradled tuning, record pitch/drift/oscillation | NOT TESTED | |
| 15. Velocity feedback | Measure wheel encoder data and closed-loop speed control | NOT IMPLEMENTED | |

## Bench commands

```sh
pio run
bash test/host/run_tests.sh
pio run -t upload
pio device monitor -b 115200
# For supervised serial capture use a terminal logging tool.
python3 tools/bench_report.py bench.log --strict --json
```

Host tests and a firmware build do not verify motor polarity, safe battery wiring, collision avoidance or sustained upright control. Keep `GYROGUARD_AUTONOMY=0` while any essential safety gate is unverified. Record any disarm reason, yaw/turn sign, motor driver temperature and control loop timing. A 'PASS' means a real result with attached evidence, **not** an expectation.

## Stop criteria

Stop and remove motor power on a controller reset, unexplained motor motion, inverted corrective direction, hot driver, voltage out of range, servo rail collapse, failed manual/physical cutoff, repeated IMU faults, or unstable upright tuning.

See [POWER_WIRING.md](POWER_WIRING.md), [PINOUT.md](PINOUT.md) and [COMMISSIONING.md](COMMISSIONING.md).
