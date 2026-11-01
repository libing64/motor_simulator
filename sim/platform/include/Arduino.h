#ifndef SFOC_NATIVE_ARDUINO_H
#define SFOC_NATIVE_ARDUINO_H
/**
 * @file Arduino.h
 * @brief Minimal Arduino core shim used to build SimpleFOC natively on a PC.
 *
 * Only the subset of the Arduino API required by the SimpleFOC core is
 * provided.  Timing functions (`micros`, `millis`, `delay`, ...) are backed by
 * a *virtual* clock (see sim_time.h) so that simulation time can be advanced
 * deterministically by the testbench instead of using wall-clock time.
 */

#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cmath>
#include <cstdarg>
#include <string>

#include "sim_time.h"

// ---------------------------------------------------------------- basic types
typedef bool boolean;
typedef uint8_t byte;

#define HIGH 0x1
#define LOW 0x0

#define INPUT 0x0
#define OUTPUT 0x1
#define INPUT_PULLUP 0x2
#define INPUT_PULLDOWN 0x3

#define NOT_AN_INTERRUPT -1
#define LED_BUILTIN 13

// C++11 is plenty for the shim
#ifndef NULL
#define NULL 0
#endif

using std::abs;
using std::fabs;

#ifndef constrain
#define constrain(amt, low, high) ((amt) < (low) ? (low) : ((amt) > (high) ? (high) : (amt)))
#endif
#ifndef _BV
#define _BV(bit) (1 << (bit))
#endif
#ifndef bitRead
#define bitRead(value, bit) (((value) >> (bit)) & 0x01)
#endif
#ifndef bitSet
#define bitSet(value, bit) ((value) |= (1UL << (bit)))
#endif
#ifndef bitClear
#define bitClear(value, bit) ((value) &= ~(1UL << (bit)))
#endif
#ifndef bitWrite
#define bitWrite(value, bit, bitvalue) ((bitvalue) ? bitSet(value, bit) : bitClear(value, bit))
#endif

// ---------------------------------------------------------------- math macros
#define PI 3.1415926535897932384626433832795f
#define HALF_PI 1.5707963267948966192313216916398f
#define TWO_PI 6.283185307179586476925286766559f
#define DEG_TO_RAD 0.017453292519943295769236907684886f
#define RAD_TO_DEG 57.295779513082320876798154814105f
#define EULER 2.718281828459045235360287471352f
#define radians(deg) ((deg) * DEG_TO_RAD)
#define degrees(rad) ((rad) * RAD_TO_DEG)
#define sq(x) ((x) * (x))

template <typename T>
inline T min_of(T a, T b) { return a < b ? a : b; }

// Arduino exposes min()/max() as macros.  SimpleFOC relies on mixed-argument
// friendly semantics, so macros (rather than std::min) are used here.  All
// standard headers used by the shim are included above this point.
#ifndef min
#define min(a, b) ((a) < (b) ? (a) : (b))
#endif
#ifndef max
#define max(a, b) ((a) > (b) ? (a) : (b))
#endif
#ifndef round
#define round(x) ((x) >= 0 ? (long)((x) + 0.5) : (long)((x)-0.5))
#endif

long map(long x, long in_min, long in_max, long out_min, long out_max);

// ---------------------------------------------------------------- flash strings
class __FlashStringHelper;
#define F(string_literal) (reinterpret_cast<const __FlashStringHelper*>(string_literal))

//! Minimal stand-in for Arduino's String (only what SimpleFOC's debug code uses).
class String {
public:
    String() {}
    String(const char* s) : s_(s ? s : "") {}
    String(const String&) = default;
    String& operator=(const String&) = default;
    const char* c_str() const { return s_.c_str(); }
    size_t length() const { return s_.size(); }
    bool operator==(const char* o) const { return s_ == (o ? o : ""); }

private:
    std::string s_;
};

//! Arduino's string-concatenation helper type (derives from String).
class StringSumHelper : public String {
public:
    StringSumHelper() {}
    StringSumHelper(const char* s) : String(s) {}
};

// ---------------------------------------------------------------- Print
class Print {
public:
    virtual ~Print() {}
    virtual size_t write(uint8_t c) { (void)c; return 0; }
    virtual size_t write(const uint8_t* buf, size_t n) {
        size_t i = 0;
        for (; i < n; i++) write(buf[i]);
        return i;
    }
    size_t write(const char* str) { return str ? write(reinterpret_cast<const uint8_t*>(str), std::strlen(str)) : 0; }

    size_t print(const char* s) { return write(s); }
    size_t print(char c) { return write(static_cast<uint8_t>(c)); }
    size_t print(const __FlashStringHelper* s) { return write(reinterpret_cast<const char*>(s)); }
    size_t print(const String& s) { return print(s.c_str()); }
    size_t print(unsigned char v, int base = 10) { return print_num(static_cast<unsigned long>(v), base); }
    size_t print(int v, int base = 10) { return print_signed(static_cast<long>(v), base); }
    size_t print(unsigned int v, int base = 10) { return print_num(static_cast<unsigned long>(v), base); }
    size_t print(long v, int base = 10) { return print_signed(v, base); }
    size_t print(unsigned long v, int base = 10) { return print_num(v, base); }
    size_t print(double v, int digits = 2) { return print_float(static_cast<float>(v), digits); }
    size_t print(float v, int digits = 2) { return print_float(v, digits); }

    size_t println() { return write("\r\n"); }
    size_t println(const char* s) { return print(s) + println(); }
    size_t println(char c) { return print(c) + println(); }
    size_t println(const __FlashStringHelper* s) { return print(s) + println(); }
    size_t println(const String& s) { return print(s.c_str()) + println(); }
    size_t println(int v, int base = 10) { return print(v, base) + println(); }
    size_t println(unsigned int v, int base = 10) { return print(v, base) + println(); }
    size_t println(long v, int base = 10) { return print(v, base) + println(); }
    size_t println(unsigned long v, int base = 10) { return print(v, base) + println(); }
    size_t println(double v, int digits = 2) { return print(v, digits) + println(); }
    size_t println(float v, int digits = 2) { return print(v, digits) + println(); }

    size_t printf(const char* fmt, ...) {
        char buf[256];
        va_list ap;
        va_start(ap, fmt);
        int n = std::vsnprintf(buf, sizeof(buf), fmt, ap);
        va_end(ap);
        if (n < 0) return 0;
        if (n > (int)sizeof(buf) - 1) n = sizeof(buf) - 1;
        write(reinterpret_cast<const uint8_t*>(buf), static_cast<size_t>(n));
        return static_cast<size_t>(n);
    }

protected:
    size_t print_float(float v, int digits) {
        char buf[64];
        int n = std::snprintf(buf, sizeof(buf), "%.*f", digits, static_cast<double>(v));
        return n > 0 ? write(reinterpret_cast<const uint8_t*>(buf), static_cast<size_t>(n)) : 0;
    }
    size_t print_num(unsigned long v, int base) {
        char buf[64];
        int n = std::snprintf(buf, sizeof(buf), base == 16 ? "%lX" : "%lu", v);
        return n > 0 ? write(reinterpret_cast<const uint8_t*>(buf), static_cast<size_t>(n)) : 0;
    }
    size_t print_signed(long v, int base) {
        char buf[64];
        int n = std::snprintf(buf, sizeof(buf), base == 16 ? "%lX" : "%ld", v);
        return n > 0 ? write(reinterpret_cast<const uint8_t*>(buf), static_cast<size_t>(n)) : 0;
    }
};

// ---------------------------------------------------------------- Serial
class HardwareSerial : public Print {
public:
    void begin(unsigned long) {}
    void end() {}
    int available() { return 0; }
    int read() { return -1; }
    int peek() { return -1; }
    void flush() {}
    operator bool() const { return true; }
};

extern HardwareSerial Serial;

// ---------------------------------------------------------------- time
unsigned long millis();
unsigned long micros();
void delay(unsigned long ms);
void delayMicroseconds(unsigned int us);
void yield();

// ---------------------------------------------------------------- GPIO / ADC
void pinMode(uint8_t pin, uint8_t mode);
void digitalWrite(uint8_t pin, uint8_t val);
int digitalRead(uint8_t pin);
int analogRead(uint8_t pin);
void analogWrite(uint8_t pin, int val);
void analogReadResolution(uint8_t bits);
void analogWriteResolution(uint8_t bits);
void attachInterrupt(uint8_t pin, void (*isr)(void), int mode);
void detachInterrupt(uint8_t pin);
#define digitalPinToInterrupt(p) (p)
#define CHANGE 1
#define FALLING 2
#define RISING 3

#endif  // SFOC_NATIVE_ARDUINO_H
