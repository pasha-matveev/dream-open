#pragma once
#include <Arduino.h>

class Emitter
{
private:
  int pin = A0;
public:
  void read();
  void init();
  void reset();
  int raw = 0;
  bool val = 0;
  bool first = 1;
  unsigned long long first_tm = 0;
  unsigned long long last_tm = 0;
};
