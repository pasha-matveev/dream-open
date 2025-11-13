#pragma once
#include <Arduino.h>

#include "Const.h"

class Kicker {
 private:
  unsigned long long charge_tm = 0;
  unsigned long long kick_tm = 0;
  int min_kick_delay = 0;
  int min_delay = 500;
  int max_delay = ROBOT ? 21000 : 2400;
  int kick_pin = 7;
  int charge_pin = 4;

 public:
  void init();
  void start();
  bool started();
  void kick(int power = -1);
  void charge(bool flag = true);
  bool is_charged();
};
