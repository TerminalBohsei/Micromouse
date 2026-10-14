#pragma once
// Minimal Arduino stand-in so the REAL firmware sources (Navigator, Floor,
// Stepper) compile and run on a PC inside the simulator.
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <stdarg.h>

#define HIGH 1
#define LOW 0
#define INPUT 0
#define OUTPUT 1
#define INPUT_PULLUP 2

unsigned long millis();
unsigned long micros();
void delay(unsigned long ms);
void pinMode(int pin, int mode);
void digitalWrite(int pin, int v);
int  digitalRead(int pin);
int  analogRead(int pin);

struct SerialShim {
  bool verbose = false;
  void begin(int) {}
  int  available() { return 0; }
  int  read() { return -1; }
  void println(const char* s) { if (verbose) puts(s); }
  __attribute__((format(printf, 2, 3))) void printf(const char* f, ...) {
    if (!verbose) return;
    va_list a; va_start(a, f); vprintf(f, a); va_end(a);
  }
};
extern SerialShim Serial;
