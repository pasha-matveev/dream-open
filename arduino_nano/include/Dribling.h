#pragma once
#include <Servo.h>

class Dribling
{
private:
  Servo ESC1;
  Servo ESC2;

  const int ESC1_pin = 6;
  const int ESC2_pin = 5;

  double current_speed = 0, desired_speed = 0;

  unsigned long long lst_tm = 0;
  double change_k = 20.0 / 5000000.0;

public:
  void init();
  void set_speed(int);
  void run();
};
