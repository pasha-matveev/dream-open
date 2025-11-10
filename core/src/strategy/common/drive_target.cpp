#include "strategy/strategy.h"

void Strategy::drive_target(Robot& robot, const Vec& target, double k) {
  Vec vel = target - robot.position;
  robot.rotation = normalize_angle(vel.field_angle() - robot.field_angle);
  vel *= k;
  vel = vel.resize(min(vel.len(), 120.0));
  robot.vel = vel;
}