#include "strategy/strategy.h"
#include "utils/config.h"
#include "utils/millis.h"

void Strategy::run_test_circle(Robot& robot) {
  robot.dribling = config.strategy.dribbling.value_r;
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
  robot.dribling = config.strategy.dribbling.value_r;
}

void Strategy::run_test(Robot& robot, Object& goal) {
  robot.dribling = config.strategy.dribbling.value_l;
  long long delta = millis() - last_hit;
  robot.kicker_force = 0;
  if (delta >= 10100) {
    last_hit = millis();
  } else if (delta >= 10000) {
    robot.kicker_force = 100;
  }
}