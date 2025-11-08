#include "strategy/strategy.h"
#include "utils/millis.h"

void Strategy::run_challenge(Robot& robot, Object& ball, Object& goal) {
  robot.dribling = max_dribling;
  double dir = M_PI / 2;
  Vec vel{robot.field_angle};
  vel = vel.turn_right();
  vel = vel.rotate(0.1);
  vel = vel.resize(17);
  robot.vel = vel;
  robot.rotation_limit = 15;
  robot.rotation = dir;
}