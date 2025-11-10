#include "strategy/strategy.h"

void Strategy::drive_ball(Robot& robot, const Vec& ball) {
  double k;
  double dist = (ball - robot.position).len();
  if (dist < 16) {
    k = 1;
  } else if (dist < 40) {
    k = 2.5;
  } else {
    k = 4;
  }
  drive_target(robot, ball, k);
  robot.dribling = base_dribling;
  robot.rotation_limit = 40;
}