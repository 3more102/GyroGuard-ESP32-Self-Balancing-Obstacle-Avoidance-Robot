#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")/../.."
mkdir -p build
c++ -std=c++17 -Wall -Wextra -Werror -pedantic -fsanitize=address,undefined \
  -fno-omit-frame-pointer -Ilib/GyroCore/src \
  lib/GyroCore/src/GyroCore.cpp test/host/test_gyrocore.cpp \
  -o build/gyroguard_host_tests
./build/gyroguard_host_tests
c++ -std=c++17 -Wall -Wextra -Werror -pedantic -fsanitize=address,undefined \
  -fno-omit-frame-pointer -Ilib/GyroCore/src \
  test/host/test_wheel_velocity.cpp -o build/gyroguard_wheel_velocity_tests
./build/gyroguard_wheel_velocity_tests

# Source-level compilation with minimal host Arduino/Wire declarations.
# This validates C++ syntax, NOT ESP32 SDK compatibility or device behavior.
for source in src/main.cpp src/MotorDriver.cpp src/Mpu6050.cpp src/Ultrasonic.cpp; do
  c++ -std=c++17 -Wall -Wextra -Werror -fsyntax-only \
      -Itest/host/stubs -Iinclude -Isrc -Ilib/GyroCore/src "$source"
done
# Compile the optional encoder-enabled branch against minimal Arduino stubs.
c++ -std=c++17 -Wall -Wextra -Werror -fsyntax-only \
    -DGYROGUARD_ENCODERS=1 \
    -Itest/host/stubs -Iinclude -Isrc -Ilib/GyroCore/src src/main.cpp
python3 -m unittest discover -s test/host -p "test_*.py" -v
echo "PASS: ESP32 source syntax check with host interface stubs"
