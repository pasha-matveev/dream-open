#pragma once
#include <Arduino.h>

class Emitter
{
private:
  const int pin = A0;
public:
  void read();
  void init();
  void reset();
  int raw = 0;
  bool val = false;
  bool first = true;
  unsigned long long first_tm = 0;
  unsigned long long last_tm = 0;
};
