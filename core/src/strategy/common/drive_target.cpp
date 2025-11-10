#include "strategy/strategy.h"

bool Strategy::drive_target(Robot& robot, const Vec& target, double k,
                            double max_speed) {
  Vec vel = target - robot.position;
  robot.rotation = normalize_angle(vel.field_angle() - robot.field_angle);
  bool finished = vel.len() <= 2.0;
  vel *= k;
  vel = vel.resize(min(vel.len(), 120.0));
  robot.vel = vel;
  return finished;
}