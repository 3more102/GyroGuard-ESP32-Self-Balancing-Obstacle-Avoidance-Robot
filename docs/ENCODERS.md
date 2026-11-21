# Optional wheel-encoder commissioning (V0.5 development)

**Status:** software-only encoder telemetry. No hardware validation; no velocity PID or encoder-dependent safety shutdown. `GYROGUARD_AUTONOMY=0` remains mandatory during testing.

## Wiring (classic ESP32-WROOM-32)

| Device | A channel (rising-edge interrupt) | B channel (direction phase) |
|---|---:|---:|
| Left wheel quadrature encoder | GPIO35 | GPIO14 |
| Right wheel quadrature encoder | GPIO39 | GPIO13 |

- All four signals MUST be within ESP32 3.3 V logic levels. Level-shift any 5 V encoder outputs. Connect encoder grounds to ESP32 ground.
- GPIO35/GPIO39 are input-only and have no internal pull-ups; open-collector encoder outputs require appropriately sized *external* pull-ups to **3.3 V**. GPIO14/13 are also configured as plain inputs, so provide external pull-ups when the encoder outputs require them.
- The firmware captures **one A rising edge** per quadrature cycle and reads B for count direction. This is not x4 quadrature decoding. `ENCODER_LEFT_SIGN` / `ENCODER_RIGHT_SIGN` may need reversing for your sensor orientation.
- With the default `GYROGUARD_ENCODERS=0`, the new encoder GPIOs are not initialized. Set this build flag to `1` in `platformio.ini` only after hardware checks; leave `GYROGUARD_AUTONOMY=0`.

## Wheels-off-ground test

1. Disconnect motor VM while installing the encoders. Confirm each encoder's power and logic levels with a multimeter or scope. Secure the robot in a cradle.
2. Enable `GYROGUARD_ENCODERS=1`, build/upload, and open 115200 baud serial telemetry. The output adds `enc_left`, `enc_right`, `enc_l_tps`, `enc_r_tps` and `enc_valid`.
3. Rotate **only the left wheel** one physical wheel revolution by hand. Verify left counts change and right counts do not. Repeat separately for the right. Adjust signs in `include/Config.h` if forward rotation decreases counts.
4. Count 5-10 physical wheel revolutions in both directions and calculate **A-rising ticks per wheel revolution**, including gearbox reduction. Rates are *ticks per second*, not RPM; `RPM = ticks_per_second * 60 / ticks_per_wheel_revolution`.
5. With both wheels stationary, tick-rate should approach zero. Repeat with loose signal wires/noisy conditions safely (motors off) to catch false ticks. A stale or implausibly large computed rate yields `enc_valid=0`.
6. **Important:** `enc_valid=1` only means the data passed the estimator's numerical/time checks. A disconnected or stuck encoder can report zero ticks and still yield `enc_valid=1`. Do not use this as a safety interlock.

Do not turn on autonomous obstacle avoidance or closed-loop velocity control before separate supervised validation of IMU/motor directions, stable upright balance, encoder signs, motor cutoff and wheel behavior. Encoder telemetry does not alter the 200 Hz balance loop.

Reference: [Espressif ESP32 GPIO peripheral documentation](https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-reference/peripherals/gpio.html).
