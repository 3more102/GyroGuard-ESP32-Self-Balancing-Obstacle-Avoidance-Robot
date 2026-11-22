# ESP32-WROOM-32 DevKit wiring (one sonar + servo)

**Start with the [verified power and battery wiring guide](POWER.md) before connecting a 2S or 12 V supply.**

| Device | Signal | ESP32 GPIO |
|---|---|---:|
| MPU6050 | SDA / SCL | 21 / 22 |
| TB6612FNG left motor | AIN1 / AIN2 / PWMA | 18 / 19 / 25 |
| TB6612FNG right motor | BIN1 / BIN2 / PWMB | 16 / 17 / 26 |
| TB6612FNG | STBY | 23 |
| **One HC-SR04** | TRIG / ECHO | **32 / 34** |
| **One servo** (SG90-class) | PWM signal | **33** |
| Arm/disarm button | GPIO to GND | 27 |

**Power & hardware safety**

- HC-SR04 usually requires 5 V. **ECHO is a 5 V output**: use a 5 V to 3.3 V logic-level shifter or divider. Example: ECHO -> 1 kΩ -> GPIO34; GPIO34 -> 2 kΩ -> GND. GPIO34 is input-only.
- Power the servo from a **separate adequately rated regulated 5 V supply** (not ESP32 3.3 V); connect the servo ground, sonar ground, motor-driver ground, and ESP32 ground together. Servo motors may draw large transient currents causing ESP32 brownouts.
- Servo horn carries the **single ultrasonic sensor**; angle 90° looks front, 135° left, 45° right. Verify your mount: swap left/right angles in Config.h if opposite. Do not exceed your physical servo travel.
- TB6612 VCC = 3.3 V logic; VM = suitable separately regulated motor supply. Motors to AO1/AO2, BO1/BO2. Verify motor and driver current limits, add decoupling near motor driver and protections. Never power motors from the ESP32 GPIO pins.
- MPU6050 AD0=GND (I2C 0x68); use 3.3 V supply **only if your specific breakout supports it**. X axis must face forward, Z up for pitch in this firmware.
- GPIO33 provides 50 Hz servo pulses from LEDC channel 2; channels 0 and 1 control motors at 20 kHz. LEDC driver calls target Arduino-ESP32 2.x.
- The servo scans sequentially, **not all three directions at once**. Side samples can be stale between sweeps. Never assume complete/instant obstacle coverage; wheel encoders and a physical emergency stop are recommended for safe autonomous testing.

## Optional wheel encoders (monitoring build only)

| Encoder | A input (CHANGE interrupt) | B input (direction) |
|---|---:|---:|
| Left | GPIO35 | GPIO13 |
| Right | GPIO36 | GPIO14 |

Both A and B outputs must stay within the ESP32's **0–3.3 V logic range**; level-shift 5 V signals. GPIO35 and GPIO36 are input-only with no internal pull-up or pull-down support: if the sensor output is open-collector/open-drain, fit external pull-ups to 3.3 V. GPIO13/14 may conflict with use of JTAG. These pins are unused by the default build. Run `pio run -e esp32dev-encoder-monitor` only after wiring checks. This is not a closed-loop velocity controller.
