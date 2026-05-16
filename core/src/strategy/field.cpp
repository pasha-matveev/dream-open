#include "strategy/field.h"

#include <spdlog/spdlog.h>

#include <cassert>

#include "config/config.h"
#include "config/strategy.h"
#include "strategy/segment.h"

Field::Field(const vector<Vec>& points_)
    : Polygon(points_), brake_modes_(points_.size(), BrakeMode::Normal) {}

Field::Field(const vector<Vec>& points_, std::vector<BrakeMode> brake_modes)
    : Polygon(points_), brake_modes_(std::move(brake_modes)) {
  assert(points.size() == brake_modes_.size());
}

double Field::decel_k_for(BrakeMode mode) {
  return mode == BrakeMode::Low ? config->strategy->motion->decel_k_low
                                : config->strategy->motion->decel_k;
}

void Field::apply_seg(int i, Robot& robot, double push_k,
                      double push_v_min) const {
  if (brake_modes_[i] == BrakeMode::Off) return;
  int j = (i + 1) % points.size();
  Segment seg{points[i], points[j]};
  seg.apply(robot, push_k, push_v_min, decel_k_for(brake_modes_[i]));
}

void Field::apply(Robot& robot, double push_k, double push_v_min) const {
  if (robot.vel.len() == 0) {
    // Не нужно ограничивать скорость
    return;
  }
  if (inside(robot.position)) {
    // 1. Ищем ближайшую точку пересечения вектора скорости и сторон
    // многоугольника
    // 2. Ограничиваем по соответствующему отрезку

    Vec long_vel = robot.vel.resize(10000);
    auto [np, idx] = find_intersection(robot.position, long_vel);
    if (idx != -1) {
      apply_seg(idx, robot, push_k, push_v_min);
    } else {
      spdlog::error("Not found any segment by intersection");
    }

    for (int i = 0; i < points.size(); ++i) {
      if (brake_modes_[i] == BrakeMode::Off) continue;
      int j = (i + 1) % points.size();
      Segment seg(points[i], points[j]);
      if (seg.is_proection(robot.position) &&
          seg.correct_side(robot.position)) {
        seg.apply(robot, push_k, push_v_min, decel_k_for(brake_modes_[i]));
      }
    }
  } else {
    // 1. Ищем 2 ближайших отрезка (2, чтобы не выезжать на углах)
    // 2. Ограничиваем по ним
    int idx1 = -1, idx2 = -1;
    double dist1 = DBL_MAX, dist2 = DBL_MAX;

    for (int i = 0; i < points.size(); ++i) {
      int j = (i + 1) % points.size();
      Segment seg(points[i], points[j]);
      double dist = seg.dist(robot.position);
      if (dist < dist1) {
        idx2 = idx1;
        dist2 = dist1;
        idx1 = i;
        dist1 = dist;
      } else if (dist < dist2) {
        idx2 = i;
        dist2 = dist;
      }
    }
    if (idx1 != -1) {
      apply_seg(idx1, robot, push_k, push_v_min);
    } else {
      spdlog::error("Not found first closest segment: {} {}", robot.position.x,
                    robot.position.y);
    }
    if (idx2 != -1) {
      apply_seg(idx2, robot, push_k, push_v_min);
    } else {
      spdlog::error("Not found second closest segment: {} {}", robot.position.x,
                    robot.position.y);
    }
  }
}