#include "strategy/strategy.h"
#include "utils/config.h"

bool Strategy::drive_target(Robot& robot, const Vec& target, double k,
                            double max_speed, double min_speed) {
  Vec vel = target - robot.position;
  double d_safe = vel.len();
  double v_safe = sqrt(2.0 * config.strategy.motion.max_linear_accel * d_safe);
  bool finished = vel.len() <= 2.0;
  if (finished) {
    vel = {0, 0};
  } else {
    vel = vel.resize(clamp(v_safe, min_speed, max_speed));
  }
  robot.vel = vel;
  robot.rotation = normalize_angle(vel.field_angle() - robot.field_angle);
  return finished;
}