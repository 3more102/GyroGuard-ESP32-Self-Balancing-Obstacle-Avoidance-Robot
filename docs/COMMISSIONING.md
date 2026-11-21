# ESP32 GyroGuard hardware commissioning

**Software prototype only — not verified on hardware.** Use a cradle with wheels off the floor. Disconnect motor power during wiring.

1. Confirm classic ESP32-WROOM-32 dev board, MPU6050, TB6612FNG, **one HC-SR04 on a servo** (e.g. SG90), 2 motors, button.
2. Verify voltage shift on the **single sonar ECHO** and a sufficiently rated **external regulated 5 V servo supply**, all grounds common. Never feed 5 V ECHO into the ESP32.
3. Place the servo at neutral with the sensor facing forward. Check clearance at 45° (right), 90° (front), 135° (left). Limit servo travel for your bracket; avoid mechanical binding.
4. Run `pio run -e esp32dev` before uploading, then `pio run -t upload` and `pio device monitor -b 115200`. On boot the motors remain disabled during MPU6050 gyro calibration. The software refuses startup samples with angular rate >15 deg/s, mean gyro offset >5 deg/s, or gyro sample standard deviation >2 deg/s. These are conservative preliminary limits, not proof that the robot is motionless. If initialization fails, secure the robot upright and motionless, inspect the wiring, then restart; never bypass failed calibration to arm the motors.
5. Confirm imu=1, pitch sign (leaning forward positive), and serial sonar look= positions. A near obstacle in front should trigger a left/right sweep and a return to front; check the measured sides match physical directions.
6. With motor power isolated, deliberately unplug ECHO or move an object out of range; the sonar must show invalid ? readings, and navigation command should not move forward.
7. With chassis in a mechanical cradle and servo clear of hands, arm via GPIO27 button and confirm that a positive forward pitch makes both wheels drive forward. Set PITCH_SIGN, MOTOR_SIGN, and each wheel sign accordingly.
8. Check disarming on button press, lost IMU data, missed controller deadline, or tilt >32°. Add an independent hardware power cutoff before unrestricted testing.
9. Tune balancing **before enabling autonomy**. PID gains are placeholders and no wheel encoders/odometry are implemented. During servo sweep, motion intent is set to zero. This does not mean the robot is guaranteed to remain in place.
10. Autonomous mode is intentionally compiled out (GYROGUARD_AUTONOMY=0). Only enable after confirming stable closed-loop balance, wiring, motor polarity, turn sign and clearance under controlled conditions.

### Timing / limitations

- Inner balance loop target: 200 Hz, best-effort polling. Servo LEDC PWM runs independently and tick() never waits for echo via pulseIn().
- The single sonar is triggered at least 65 ms apart. Servo angle changes allow at least 220 ms settling before a new ping. Real servo speed, vibrations, mechanical backlash and reflected sonar geometry must be measured.
- Normally sonar stays centered, measuring forward. On an obstacle under 50 cm, it temporarily scans left -> right -> front and temporarily suspends navigation. A new sweep is rate-limited to 1.5 seconds.
- Sonar is neither a full collision sensor nor a substitute for odometry; side readings are snapshots. Failures may cause collisions. No certified real-time deadline guarantees.
