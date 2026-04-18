#pragma once
#include <Servo.h>

#include "Const.h"

class Dribling {
 private:
  Servo ESC_left;
  Servo ESC_right;

  const int left_pin = 6;
  const int right_pin = 5;

  float left_k = ROBOT ? 0.00624654 : 0.0240184;
  float left_b = ROBOT ? 1450.01331 : 1443.63237;
  float right_k = ROBOT ? 0.0246352 : 0;
  float right_b = ROBOT ? 1441.63139 : 0;

  int desired_rpm = 0;

 public:
  void init();
  void set_rmp(int);
  void run();
};
