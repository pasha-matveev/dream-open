#include "strategy/strategy.h"
#include "utils/config.h"
#include "utils/millis.h"

void Strategy::run_test_circle(Robot& robot) {
  robot.dribling = config.strategy.max_dribling;
  double dir = M_PI / 2;
  Vec vel{robot.field_angle};
  vel = vel.turn_right();
  vel = vel.rotate(0.1);
  vel = vel.resize(19);
  robot.vel = vel;
  robot.rotation_limit = 15;
  robot.rotation = dir;
}

void Strategy::run_test_dribling(Robot& robot) {
  robot.dribling = config.strategy.max_dribling;
}

void Strategy::run_test(Robot& robot, Object& goal) {
  if (millis() - last_ball_visible <= 50) {
    dubins_hit(robot, goal);
  }
}