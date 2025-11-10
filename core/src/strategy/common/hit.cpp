#include "strategy/strategy.h"

void Strategy::hit(Robot& robot, Object& goal, int power, int forward_timeout,
                   bool curved_rotation, int kick_timeout, double precision) {
  double target_angle;
  if (goal.visible) {
    target_angle = normalize_angle(goal.relative_angle);
  } else {
    Vec center{91, 237};
    Vec simple_route = center - robot.position;
    target_angle =
        normalize_angle(simple_route.field_angle() - robot.field_angle);
  }
  kick_dir(robot, target_angle, power, forward_timeout, curved_rotation,
           kick_timeout, precision);
}