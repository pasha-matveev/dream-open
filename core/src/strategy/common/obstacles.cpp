#include "strategy/obstacles.h"

#include <limits>

#include "config/config.h"
#include "config/lidar.h"
#include "robot.h"
#include "strategy/zones.h"
#include "utils/geo/polygon.h"

std::optional<Vec> nearest_obstacle(Robot& robot, const Vec& ref_point,
                                    double min_y) {
  if (!robot.lidar) return std::nullopt;
  static Polygon full_markup = make_full_field_markup();

  const auto& piter_cfg = *config->lidar->piter;

  std::optional<Vec> best = std::nullopt;
  double best_dist = std::numeric_limits<double>::infinity();
  for (const auto& obs : robot.lidar->obstacles_data) {
    // Препятствия в зоне вратаря — скорее всего наш вратарь, не враг.
    if (obs.y < min_y) continue;
    double self_dist = (obs - robot.position).len();
    if (self_dist < piter_cfg.self_exclusion_radius) continue;
    if (!full_markup.inside(obs) &&
        full_markup.dist(obs) > piter_cfg.markup_tolerance) {
      continue;
    }
    double ref_dist = (obs - ref_point).len();
    if (ref_dist < best_dist) {
      best_dist = ref_dist;
      best = obs;
    }
  }
  return best;
}
