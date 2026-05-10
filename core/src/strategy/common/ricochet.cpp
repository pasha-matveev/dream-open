#include "field_dims.h"
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
