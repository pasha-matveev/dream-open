#include "strategy/strategy.h"

bool Strategy::drive_target(Robot& robot, const Vec& target, double k,
                            double max_speed, double min_speed) {
  Vec vel = target - robot.position;
  robot.rotation = normalize_angle(vel.field_angle() - robot.field_angle);
  bool finished = vel.len() <= 2.0;
  vel *= k;
  vel = vel.resize(clamp(vel.len(), min_speed, max_speed));
  robot.vel = vel;
  return finished;
}