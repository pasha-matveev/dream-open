#include "Robot.h"

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