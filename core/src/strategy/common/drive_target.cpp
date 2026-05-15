#include "config/config.h"
#include "config/strategy.h"
#include "config/strategy/control.h"
#include "strategy/motion.h"
#include "utils/mapper.h"

bool drive_target(Robot& robot, const Vec& target, double max_speed,
                  double min_speed, bool is_ball, const Mapper* speed_map) {
  Vec vel = target - robot.position;
  double d_safe = vel.len();
  double v_safe =
      sqrt(2.0 * config->strategy->motion->max_linear_accel * d_safe);
  if (is_ball) {
    // speed_map задаёт свою кривую скорости (напр. fast_direct); по умолчанию
    // используется control->speed.
    const Mapper& m =
        speed_map ? *speed_map : *config->strategy->control->speed;
    v_safe = min(v_safe, m.map(d_safe));
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
