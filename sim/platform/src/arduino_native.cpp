/**
 * @file arduino_native.cpp
 * @brief Native implementation of the Arduino shim (virtual clock + pin stubs).
 */

#include "Arduino.h"

#include <cstdio>
#include <cstring>

namespace {
unsigned long long g_sim_time_us = 0;
HardwareSerial g_serial;

// Very small pin state mirror, useful when debugging the shim itself.
uint8_t g_pin_state[256] = {0};
}  // namespace

HardwareSerial Serial;

// ------------------------------------------------------------------ time base
extern "C" void sim_time_set_us(unsigned long long t_us) { g_sim_time_us = t_us; }
extern "C" unsigned long long sim_time_get_us(void) { return g_sim_time_us; }
extern "C" void sim_time_advance_us(unsigned long dt_us) { g_sim_time_us += dt_us; }

unsigned long micros() { return static_cast<unsigned long>(g_sim_time_us & 0xFFFFFFFFULL); }
unsigned long millis() { return static_cast<unsigned long>((g_sim_time_us / 1000ULL) & 0xFFFFFFFFULL); }

// Delays do not block in simulation: they simply advance the virtual clock, so
// SimpleFOC's start-up delays cost no wall-clock time but keep timing coherent.
void delay(unsigned long ms) { g_sim_time_us += static_cast<unsigned long long>(ms) * 1000ULL; }
void delayMicroseconds(unsigned int us) { g_sim_time_us += us; }
void yield() {}

// ------------------------------------------------------------------ gpio/adc
void pinMode(uint8_t pin, uint8_t mode) { g_pin_state[pin] = mode; }

void digitalWrite(uint8_t pin, uint8_t val) {
    (void)pin;
    (void)val;
}

int digitalRead(uint8_t pin) {
    (void)pin;
    return LOW;
}

int analogRead(uint8_t pin) {
    (void)pin;
    return 0;
}

void analogWrite(uint8_t pin, int val) {
    (void)pin;
    (void)val;
}

void analogReadResolution(uint8_t bits) { (void)bits; }
void analogWriteResolution(uint8_t bits) { (void)bits; }

void attachInterrupt(uint8_t pin, void (*isr)(void), int mode) {
    (void)pin;
    (void)isr;
    (void)mode;
}

void detachInterrupt(uint8_t pin) { (void)pin; }

long map(long x, long in_min, long in_max, long out_min, long out_max) {
    if (in_max == in_min) return out_min;
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
