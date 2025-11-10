#include "strategy/strategy.h"

bool Strategy::turn(Robot& robot, double target_angle, bool curved_rotation) {
  target_angle -= 0.01;  // Бьём немного правее
  double delta = target_angle - robot.field_angle;
  if (curved_rotation) {
    Vec vel{robot.field_angle};
    if (delta > 0) {
      vel = vel.turn_right();
      vel = vel.rotate(0.1);
    } else {
      vel = vel.turn_left();
      vel = vel.rotate(-0.1);
    }
    // Длина (17) и скорость поворота (15) подобраны
    vel = vel.resize(17);
    robot.vel = vel;
    robot.rotation_limit = 15;
  } else {
    robot.vel = {0, 0};
    robot.rotation_limit = 15;
  }
  robot.rotation = delta;
  accelerated_dribbling(robot);
  if (abs(delta) <= 0.015) {
    // поворот закончен
    return true;
  }
  return false;
}