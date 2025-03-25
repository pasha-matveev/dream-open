#include <Arduino.h>
#include "Gyro.h"
#include "Motors.h"
#include "Kicker.h"
#include "Dribling.h"
#include "Emitter.h"

class Robot
{
private:
  float rel_direction, rel_rotation;

public:
  Gyro *gyro = new Gyro;
  Motors *motors = new Motors;
  Kicker *kicker = new Kicker;
  Dribling *dribling = new Dribling;
  Emitter *emitter = new Emitter;

  void init();
  void read();
  void run();
  void stop();
  float direction = 0;
  float speed = 0;
  float rotation = 0;
  int dribling_speed = 0;
  int kicker_force = 0;

  bool init_gyro = true, init_motors = true, init_kicker = true, init_dribling = true, init_emitter = true;
};

void Robot::init()
{
  if (init_gyro)
    gyro->init();
  if (init_motors)
    motors->init();
  if (init_kicker)
    kicker->init();
  if (init_dribling)
    dribling->init();
  if (init_emitter)
    emitter->init();
}

void Robot::read()
{
  if (init_gyro)
    gyro->read();
  if (init_emitter)
    emitter->read();
}

void Robot::run()
{
  rel_direction = gyro->relative(direction);
  rel_rotation = gyro->relative(rotation);
  motors->run(rel_direction, speed, rel_rotation);
  if (init_dribling)
    dribling->run();
}

void Robot::stop()
{
  motors->stop();
  if (init_dribling)
  {
    dribling->set_speed(0);
    dribling->run();
  }
}