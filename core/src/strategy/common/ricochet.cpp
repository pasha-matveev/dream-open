#include "field_dims.h"
#include "robot.h"
#include "strategy/kick.h"
#include "strategy/motion.h"
#include "utils/geo/vec.h"

double compute_ricochet_field_angle(const Vec& ball_pos, const Vec& target,
                                    bool left) {
  Vec mirror;
  if (left) {
    // Отражение через левую стенку x=0.
    mirror = {-target.x, target.y};
  } else {
    // Отражение через правую стенку x=field_dims::kWidth.
    mirror = {2.0 * field_dims::kWidth - target.x, target.y};
  }
  Vec dir = mirror - ball_pos;
  return dir.field_angle();
}

void execute_ricochet_kick(Robot& robot, KickController& kick, bool left,
                           const Vec& target, bool recompute_each_tick,
                           bool curved_rotation,
                           const Mapper* dribbling_slowdown,
                           double& cached_field_angle, bool& angle_computed) {
  double field_angle;
  if (recompute_each_tick) {
    // Пересчёт каждый тик от текущего ball_hole_position — угол сходится к
    // верному прицелу по мере доворота корпуса.
    field_angle =
        compute_ricochet_field_angle(robot.ball_hole_position(), target, left);
  } else {
    // Фиксируем угол при первом вызове (как kickoff).
    if (!angle_computed) {
      cached_field_angle = compute_ricochet_field_angle(
          robot.ball_hole_position(), target, left);
      angle_computed = true;
    }
    field_angle = cached_field_angle;
  }
  double relative_dir = normalize_angle(field_angle - robot.field_angle);
  KickParams kp{.relative_dir = relative_dir};
  kp.curved_rotation = curved_rotation;
  kp.dribbling_slowdown = dribbling_slowdown;
  kick.execute(robot, kp);
}
