#pragma once
#include <Arduino.h>
#include <Servo.h>

class Dribling
{
private:
  Servo *ESC1 = new Servo;
  Servo *ESC2 = new Servo;
  double current_speed = 0, desired_speed = 0;

  unsigned long long lst_tm = 0;
  double change_k = 20.0 / 5000000.0;

public:
  void init();
  void set_speed(int);
  void run();
};
