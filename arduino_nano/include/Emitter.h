#pragma once
#include <Arduino.h>

class Emitter {
 private:
  const int pin = A0;

 public:
  void read();
  void init();
  void reset();
  int raw = 0;
};
