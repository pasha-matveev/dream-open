#include <Arduino.h>
#include "Gyro.h"
#include "Motors.h"
#include "Kicker.h"
#include "Dribling.h"
#include "Emitter.h"
#include "Button.h"

class Robot
{
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

  bool init_gyro = true, init_motors = true, init_kicker = true, init_dribling = true, init_emitter = true, init_button = true;
};

void Robot::init()
{
  if (init_motors)
    motors.init();
  if (init_gyro)
    gyro.init();
  if (init_dribling)
    dribling.init();
  if (init_emitter)
    emitter.init();
  if (init_kicker)
    kicker.init();
  if (init_button)
    button.init();
}

void Robot::read()
{
  if (init_gyro)
    gyro.read();
  if (init_emitter)
    emitter.read();
}

void Robot::run()
{
  rel_direction = gyro.relative(direction);
  rel_rotation = gyro.relative(rotation);
  motors.run(rel_direction, speed, rel_rotation, rotation_limit);
  if (init_dribling)
    dribling.run();
  first_motors_stop = true;
}

void Robot::stop()
{
  kicker.force = 0;
  if (first_motors_stop)
  {
    motors.run(0, 0, 0);
    first_motors_stop = false;
  }
  else
  {
    motors.stop();
  }
  if (init_dribling)
  {
    dribling.set_speed(0);
    dribling.run();
  }
}