#pragma once
#include <Arduino.h>

#include "Button.h"
#include "Dribling.h"
#include "Emitter.h"
#include "Gyro.h"
#include "Kicker.h"
#include "Motors.h"

#define ROBOT 0  // 0 - black robot, 1 - green robot

class Robot {
 private:
  float rel_direction, rel_rotation;
  bool first_motors_stop = true;

 public:
  Gyro gyro;
  Motors motors;
  Kicker kicker;
  Dribling dribling;
  Emitter emitter;
  Button button;

  void init();
  void read();
  void run();
  void stop();
  float direction = 0;
  float speed = 0;
  float rotation = 0;
  float rotation_limit = -1;
  int dribling_speed = 0;
  int kicker_force = 0;

  const bool init_gyro = false, init_motors = true, init_kicker = true,
             init_dribling = true, init_emitter = true, init_button = true;
};
