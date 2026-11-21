#pragma once
#include <cstdint>
#include <cstddef>
struct FakeWire {
 void begin(int, int);
 void setClock(uint32_t);
 void setTimeOut(uint16_t);
 void beginTransmission(uint8_t);
 size_t write(uint8_t);
 uint8_t endTransmission(bool);
 uint8_t requestFrom(uint8_t, uint8_t, uint8_t);
 int read();
};
extern FakeWire Wire;
