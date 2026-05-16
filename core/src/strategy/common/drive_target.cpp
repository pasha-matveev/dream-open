#include "config/config.h"
#include "config/strategy.h"
#include "config/strategy/control.h"
#include "strategy/motion.h"
#include "utils/mapper.h"

bool drive_target(Robot& robot, const Vec& target, const DriveParams& params) {
  Vec vel = target - robot.position;
  double d_safe = vel.len();
  // brake_safe — физический предел торможения до остановки в цели; при false
  // стартуем с max_speed и кривую задаёт только маппер (таран).
  double v_safe =
      params.brake_safe
          ? sqrt(2.0 * config->strategy->motion->max_linear_accel * d_safe)
          : params.max_speed;
  if (params.is_ball) {
    // speed_map задаёт свою кривую скорости (напр. fast_direct); по умолчанию
    // используется control->speed.
    const Mapper& m = params.speed_map ? *params.speed_map
                                       : *config->strategy->control->speed;
    v_safe = min(v_safe, m.map(d_safe));
  }
  // TODO: в конфигурации
  bool finished = (target - robot.position).len() <= 2;
  if (finished) {
    vel = {0, 0};
  } else {
    vel = vel.resize(clamp(v_safe, params.min_speed, params.max_speed));
  }
  robot.vel = vel;
  robot.rotation = normalize_angle(vel.field_angle() - robot.field_angle);
  return finished;
}
