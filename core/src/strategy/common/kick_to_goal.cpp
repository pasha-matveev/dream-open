#include "strategy/strategy.h"

void Strategy::kick_to_goal(Robot& robot, Object& goal, KickParams params) {
  double target_angle;
  if (goal.visible) {
    target_angle = normalize_angle(goal.relative_angle);
  } else {
    Vec center{91, 237};
    Vec simple_route = center - robot.position;
    target_angle =
        normalize_angle(simple_route.field_angle() - robot.field_angle);
  }
  params.relative_dir = target_angle;
  kick(robot, params);
}