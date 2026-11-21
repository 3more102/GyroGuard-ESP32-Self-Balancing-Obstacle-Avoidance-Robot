# GyroGuard ESP32 — Power, wiring and first hardware checks

**Status: engineering checklist, not a validated schematic or a tested robot.** This guide complements [PINOUT.md](PINOUT.md) and [COMMISSIONING.md](COMMISSIONING.md). Disconnect all power while modifying wiring. Keep the robot restrained, with wheels clear of the floor, and install an **independent motor-power cutoff** before actuating motors.

## Can a 12 V battery power this robot?

**Only indirectly, and only after checking the exact motor and battery specifications.**

- The **ESP32-WROOM-32 module** itself requires **3.0–3.6 V** at its 3.3 V supply rail. **Never connect 12 V to an ESP32 3V3 GPIO or module supply.** A particular development board may expose a USB/5V/VIN input with its own regulator; verify that specific board schematic before applying power. Use a properly sized buck regulator appropriate to the board input, not a raw 12 V connection.
- The **TB6612FNG motor supply (VM)** has an operating range of **2.5–13.5 V**, while the **logic supply (VCC)** has an operating range of **2.7–5.5 V**. In this project VCC is **3.3 V**. **12 V may be usable at VM only** if the battery's *maximum charged voltage*, the *motors' rated voltage*, *motor stall current*, cooling, and the chosen driver board all remain within specification. A motor-driver voltage limit is **not** permission to feed that voltage into the ESP32.
- The Toshiba TB6612FNG datasheet rates output current up to **1.0 A per channel in its operating range when VM ≥ 4.5 V** (and lists separately defined short peak/absolute ratings). Measure or obtain the motor's stall current and ensure proper driver margin. Do not size the driver using an absolute-maximum or brief peak value as a continuous rating.
- The standard **HC-SR04** requires **5 V** and its **ECHO must be level-shifted to 3.3 V logic** before reaching ESP32 GPIO34. Power the SG90-class servo from a **regulated 5 V rail sized for its transient load**; do not draw servo power from ESP32 3V3.

**No specific battery is selected in this repository:** select one only after verifying motor rated voltage, stall current, peak servo current, regulator ratings and the actual ESP32 development board input requirements.

## Power-distribution concept

```text
Battery (voltage validated for motors + electronics)
  |
  +-- fuse / independent emergency motor cutoff --> TB6612 VM --> motors
  |
  +-- suitably rated regulator --> ESP32 DevKit approved power input
  |                                 +-- regulated 3V3 --> TB6612 VCC, MPU6050*
  |
  +-- regulated 5V with servo current margin --> SG90-class servo
                                          +----> HC-SR04 VCC

ALL LOW-VOLTAGE GROUNDS MUST SHARE A COMMON REFERENCE.
TB6612 STBY remains low until firmware deliberately enables the driver.
*Check the exact MPU6050 breakout's voltage requirements and onboard regulator.
```

**USB warning:** When flashing via USB while external supplies are attached, check the specific board's USB/external-power arrangement to prevent back-feeding. Motor and servo surges can reset the ESP32 even when nominal supply voltages are correct; measure rail dips with suitable instrumentation.

## Current firmware signal map (classic ESP32-WROOM-32)

| Device | GPIO connection | Electrical note |
|---|---|---|
| MPU6050 I2C | SDA 21, SCL 22 | 3.3 V logic; breakout-specific supply |
| TB6612 motor A | AIN1 18, AIN2 19, PWMA 25 | Two outputs to left motor |
| TB6612 motor B | BIN1 16, BIN2 17, PWMB 26 | Two outputs to right motor |
| TB6612 standby | STBY 23 | Keep disabled until controlled bench test |
| Single HC-SR04 | TRIG 32, ECHO 34 | ECHO 5 V level shift required; GPIO34 is input only |
| Servo carrying HC-SR04 | signal 33 | Separate regulated 5 V supply, common ground |
| Manual arm/disarm button | GPIO27 to GND | Internal pull-up |
| Optional encoders (not velocity PID) | left A35/B13; right A36/B14 | All signals 3.3 V max; GPIO35/36 require external pull-ups for open-drain sources |

See [PINOUT.md](PINOUT.md) and `include/Config.h` for the canonical current pin assignment. Confirm every pin against the firmware version flashed onto the board.

## Stepwise electrical checks — before balancing

1. **With all power off:** check no short exists between battery positive and ground; verify rail polarity and that the independent motor cutoff actually opens the path to TB6612 VM. Check wiring against the exact *breakout board* labels, not just IC pin numbers.
2. **Logic first, motor VM disconnected:** power the ESP32 from an approved supply; measure the 3.3 V rail, confirm serial boot at 115200 baud, `imu=1` and the expected MPU6050 pitch sign.
3. **Sensor/servo next:** power the servo from its regulated 5 V rail with a common ground; verify only one sonar transducer is installed; check 90° front, 135° left and 45° right physically match the mechanism. Check the ECHO voltage **at GPIO34** under operation, not only at the sensor.
4. **Driver logic with motor supply still isolated:** verify TB6612 VCC measures 3.3 V, STBY is inactive after boot and motor PWM/control pins are not accidentally energized by unintended wiring. Do not rely on software-only shutdown as the emergency stop.
5. **Only on a restrained rig:** after verifying motor rated voltage/current and the applicable firmware's actuation safety flags, energize VM using a current-limited supply and independent cutoff. Validate each wheel's direction at the lowest practical energy. Confirm that forward pitch commands **forward corrective wheel rotation** before trying closed-loop balancing.
6. **Fault injection under supervision:** test physical cutoff, manual disarm, excessive tilt, IMU fault, and missed-deadline shutdown with hands and cables away from rotating wheels. Record power rail behavior and serial logs.
7. **No floor trial / autonomous navigation** until balance stability, controller polarity, sonar angles, braking response and encoder-based speed feedback are independently verified. Keep `GYROGUARD_AUTONOMY=0` while commissioning.

## References (manufacturer/distributor)

- Espressif, [ESP32-WROOM-32 Datasheet](https://documentation.espressif.com/esp32-wroom-32_datasheet_en.html) — 3.0–3.6 V module operating supply.
- Toshiba, [TB6612FNG Datasheet](https://toshiba.semicon-storage.com/info/TB6612FNG_datasheet_en_20141001.pdf?did=10660&prodName=TB6612FNG) — VM/VCC operating range, output-current limits and motor bridge behavior.
- SparkFun, [HC-SR04 (5 V) product documentation](https://www.sparkfun.com/ultrasonic-distance-sensor-hc-sr04.html) — standard ultrasonic module supply.

**Bench log = evidence of observed behavior, not proof of safe balancing.** Never interpret software compilation, static tests or simulated wiring as physical validation.
