// Arduino.h falso para testar RFID.cpp no PC
#pragma once
#include <stdint.h>
#include <string.h>
#include <stdio.h>
#include <stddef.h>
typedef uint8_t byte; typedef bool boolean;
#define HEX 16
#define DEC 10
#define INPUT 0
#define OUTPUT 1
#define INPUT_PULLUP 2
#define A0 14
#define A1 15
#define A2 16
#define A3 17
#define A4 18
#define A5 19
#define A6 20
#define A7 21
#define _SFR_IO8(x) (*(volatile uint8_t*)0)
#define PROGMEM
class __FlashStringHelper;
#define F(s) (reinterpret_cast<const __FlashStringHelper*>(s))
extern uint64_t sim_us;
inline uint32_t millis(){ return (uint32_t)(sim_us/1000); }
inline uint32_t micros(){ return (uint32_t)sim_us; }
inline void delay(uint32_t ms){ sim_us += (uint64_t)ms*1000; }
class Print { public:
  virtual size_t write(uint8_t c){ putchar(c); return 1; }
  size_t print(const __FlashStringHelper*s){ return print((const char*)s); }
  size_t print(const char*s){ size_t n=0; while(*s) n+=write(*s++); return n; }
  size_t print(char c){ return write(c); }
  size_t print(unsigned v,int b=DEC){ char t[20]; snprintf(t,20,b==HEX?"%X":"%u",v); return print(t); }
  size_t print(int v,int b=DEC){ return print((unsigned)v,b); }
  size_t print(uint8_t v,int b=DEC){ return print((unsigned)v,b); }
  size_t print(unsigned long v,int b=DEC){ return print((unsigned)v,b); }
  size_t println(){ return write('\n'); }
  template<class T> size_t println(T v){ size_t n=print(v); return n+println(); }
  template<class T> size_t println(T v,int b){ size_t n=print(v,b); return n+println(); }
};
class Stream : public Print {};
extern Print Serial;
