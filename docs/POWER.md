# GyroGuard ESP32 — battery, power rails, and safe wiring

**Hardware checklist, not a tested schematic.** The actual motor model, battery pack, buck modules, ESP32 DevKit revision and servo current requirements must be checked before power-up. Disconnect the battery when rewiring.

## Can the robot use a 12 V supply?

**Not as an ESP32 supply.** The ESP32-WROOM-32 module itself requires **3.0–3.6 V at VDD33** (3.3 V nominal); the permitted input to a DevKit's **5V/VIN** pin is board-dependent. Never connect 12 V to the ESP32 3V3 pin, GPIO, servo, or sensor.

The Toshiba TB6612FNG has **separate pins**:

| Connection | Electrical role | Verified range / requirement |
|---|---|---|
| TB6612 **VM** | Motor H-bridge rail | **2.5–13.5 V operating**, subject to motor/driver current and thermal limits |
| TB6612 **VCC** | Logic supply | **2.7–5.5 V operating**; use this project's **3.3 V logic rail** |
| ESP32 **3V3/VDD33** | Controller logic | **3.0–3.6 V at the WROOM module**, nominal 3.3 V |
| Servo | Actuator supply | Follow your *specific servo* data sheet; never feed it from ESP32 3V3 |
| HC-SR04 | Ultrasonic transducer supply | Follow the specific module data sheet; common HC-SR04 modules use 5 V |

A *regulated* 12 V source may go **only to TB6612 VM** if **both motors are rated for that voltage**, the PCB/module accepts it, and startup/stall current, thermal dissipation and switching spikes remain within the actual driver limits. **Do not infer motor compatibility from H-bridge VM compatibility.** A 3-cell Li-ion pack is 12.6 V fully charged and leaves little headroom to the 13.5 V operating limit. Prefer a measured, compatible motor supply with margin, rather than blindly connecting a 12 V adapter or battery.

Toshiba specifies **1.0 A/channel operating maximum when VM is at least 4.5 V**, and **1.2 A/channel absolute maximum rating**; a separate short pulse rating is not a safe continuous-motor specification. If motor stall/startup current exceeds the board/IC limits, select a higher-current driver instead of relying on PWM caps to protect hardware.

## Candidate two-cell battery architecture (only for matching motor ratings)

A two-cell series lithium-ion pack is **7.4 V nominal, 8.4 V fully charged**. This is an example architecture, not a statement that any unknown motor will tolerate 8.4 V.

```text
     Protected 2S battery (7.4 V nominal, 8.4 V full)
                  |
       battery-rated fuse + main OFF switch
                  |
           +------+-------------------------+
           |                                |
           v                                v
    TB6612 VM motor rail          Regulated 5 V buck rail
           |                                |
       Motor A/B                 +----------+-------------+
                                |                        |
                          ESP32 DevKit                 5 V servo
                          5 V/VIN input*               HC-SR04 VCC
                                |
                        onboard 3.3 V rail*
                          |          |
                      MPU6050     TB6612 VCC

    COMMON GND: battery -, TB6612 GND/PGND, ESP32 GND,
                MPU6050 GND, servo GND, HC-SR04 GND

    ESP32 GPIO32 -> sonar TRIG
    Sonar ECHO 5 V -> 5V-to-3.3V level-shift -> ESP32 GPIO34
    ESP32 GPIO33 -> servo SIGNAL
    ESP32 GPIO23 -> TB6612 STBY (logic, not motor power)
```

`*` Confirm your **exact DevKit's** 5V/VIN input and onboard 3.3 V regulator current rating before using that path. Alternatively use separately documented, correctly regulated rails. A dedicated 5 V regulator for the servo can reduce motor-induced brownouts; always join grounds for a valid signal reference. Ensure the motor rail is appropriately fused and wiring/contacts withstand stall current. A physical motor-power kill switch is required for supervised mechanical tests.

**Do not use the TB6612 H-bridge as a battery voltage regulator.** Its VM rail is passed to the switching motor bridge; motor power is not a regulated 3.3 V or 5 V output.

## Meter checks before plugging in the ESP32

1. With all power off, verify ground continuity, no rail short, correct motor wiring, and correct orientation/polarity of battery, regulator and driver.
2. Power the buck converter **without the ESP32/servo connected**. Measure the regulator output and verify the DevKit's expected input voltage.
3. Measure the motor VM rail at the TB6612 module and **logic VCC = 3.3 V** separately; never interchange them.
4. Connect the controller without motor power; verify 3.3 V near the module under load. Check the servo supply for large voltage drops during movement.
5. Measure the **level-shifted** HC-SR04 ECHO at GPIO34: it must not exceed ESP32 input limits. The existing [pinout](PINOUT.md) shows a 1 kΩ / 2 kΩ divider for nominal 5 V ECHO.
6. Check the main cutoff kills **motor power independently of firmware**. Test motors only in a restrained cradle with wheels off the floor and autonomy disabled.
7. Do not connect a lithium-ion pack unless it has suitable protection, wiring, charger and an enclosure rated for the intended current.

## Sources

- [Toshiba TB6612FNG manufacturer datasheet, electrical ratings and operating range](https://toshiba.semicon-storage.com/info/TB6612FNG_datasheet_en_20141001.pdf?did=10660&prodName=TB6612FNG) (see page 3).
- [Espressif ESP32-WROOM-32 datasheet, electrical characteristics](https://documentation.espressif.com/esp32-wroom-32_datasheet_en.html) (section 5).
- [GyroGuard pinout](PINOUT.md) and [supervised commissioning](COMMISSIONING.md).

**Status:** Documentation only. No battery or physical wiring has been validated.
