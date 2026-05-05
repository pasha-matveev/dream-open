#include <spdlog/spdlog.h>

#include "strategy/motion.h"
#include "utils/config.h"

bool drive_target(Robot& robot, const Vec& target, double max_speed,
                  double min_speed, bool is_ball) {
  Vec vel = target - robot.position;
  double d_safe = vel.len();
  double v_safe = sqrt(2.0 * config.strategy.motion.max_linear_accel * d_safe);
  if (is_ball) {
    double mapper_vel = config.strategy.control.speed.map(d_safe);
    if (mapper_vel > v_safe) {
      spdlog::error("Mapper speed higher than estimated safe speed: {} / {}",
                    mapper_vel, v_safe);
    }
    v_safe = min(v_safe, mapper_vel);
  }
  // TODO: в конфигурации
  bool finished = (target - robot.position).len() <= 2;
  if (finished) {
    vel = {0, 0};
  } else {
    vel = vel.resize(clamp(v_safe, min_speed, max_speed));
  }
  robot.vel = vel;
  robot.rotation = normalize_angle(vel.field_angle() - robot.field_angle);
  return finished;
}
