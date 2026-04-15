#pragma once
#include <Servo.h>

class Dribling {
 private:
  Servo ESC_left;
  Servo ESC_right;

  const int left_pin = 6;
  const int right_pin = 5;

  float left_k = 0.00624654;
  float left_b = 1450.01331;
  float right_k = 0.0246352;
  float right_b = 1441.63139;

  int desired_rpm = 0;

 public:
  void init();
  void set_rmp(int);
  void run();
};
