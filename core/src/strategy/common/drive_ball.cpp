#include "strategy/motion.h"
#include "utils/config.h"

void drive_ball(Robot& robot, const Vec& ball) {
  // double dist = (ball - robot.position).len();
  drive_target(robot, ball, 120, 0, true);
  robot.dribling = config.strategy.dribbling.value_l;
  robot.rotation_limit = 40;
}
