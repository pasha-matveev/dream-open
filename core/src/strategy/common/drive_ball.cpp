#include "strategy/motion.h"
#include "config/config.h"
#include "config/strategy.h"
#include "utils/mapper.h"

void drive_ball(Robot& robot, const Vec& ball) {
  // double dist = (ball - robot.position).len();
  drive_target(robot, ball, 120, 0, true);
  robot.dribbling = config->strategy->dribbling->value_l;
  robot.rotation_limit = 40;
}
