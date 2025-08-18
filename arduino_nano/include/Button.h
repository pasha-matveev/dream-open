#pragma once
#include <Arduino.h>

class Button
{
private:
  int pin = 8;
  bool flag = 0;
  bool but = 0;

public:
  void init();
  bool state();
};
