#include "strategy/strategy.h"

double Strategy::compute_ricochet(Robot& robot, bool left) {
  Vec ball_position = robot.ball_hole_position();
  Vec target{91, 230};
  double a;
  if (left) {
    a = ball_position.x;
  } else {
    a = 182 - ball_position.x;
  }
  double b = 217.5 - ball_position.y;
  double c = b / 2;
  double alpha = atan(a / c);
  double s = (left) ? 1 : -1;
  double res = alpha * s - robot.field_angle;
  return res;
}