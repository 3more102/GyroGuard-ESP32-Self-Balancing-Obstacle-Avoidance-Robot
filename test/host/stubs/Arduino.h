#pragma once
#include <cstddef>
#include <cstdint>
#include <cmath>
#define IRAM_ATTR
#define HIGH 1
#define LOW 0
#define OUTPUT 1
#define INPUT 0
#define INPUT_PULLUP 2
#define CHANGE 3
uint32_t micros();
uint32_t millis();
void pinMode(int, int);
int digitalRead(int);
void digitalWrite(int, int);
void attachInterruptArg(int, void(*)(void*), void*, int);
void noInterrupts();
void interrupts();
void delay(unsigned long);
void delayMicroseconds(unsigned int);
double ledcSetup(uint8_t, double, uint8_t);
void ledcAttachPin(int, uint8_t);
void ledcWrite(uint8_t, uint32_t);
template<class T> T constrain(T x, T low, T high) { return x < low ? low : (x > high ? high : x); }
struct FakeSerial {
 void begin(unsigned long);
 void print(const char*);
 void println(const char*);
 template<class...Args> void printf(const char*, Args...);
};
extern FakeSerial Serial;
