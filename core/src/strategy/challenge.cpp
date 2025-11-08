#include "strategy/strategy.h"
#include "utils/millis.h"

void Strategy::run_challenge(Robot& robot, Object& ball, Object& goal) {
  double dir = M_PI / 2;
  robot.dribling = 60;
  Vec vel{robot.field_angle};
  vel = vel.turn_right();
  // vel = vel.rotate(0.1);
  vel = vel.resize(15);
  robot.vel = vel;
  robot.rotation_limit = 15;
  robot.rotation = dir;
}