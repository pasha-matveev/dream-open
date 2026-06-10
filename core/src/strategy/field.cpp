#include "strategy/field.h"

#include <spdlog/spdlog.h>

#include <cassert>

#include "config/brake_profile.h"
#include "config/config.h"
#include "config/strategy.h"
#include "strategy/segment.h"

const BrakeProfile profile_for(BrakeType type) {
  assert(type != BrakeType::Off);  // подразумевается, что Off пропускается
  switch (type) {
    case BrakeType::Wall:
      return *config->strategy->motion->wall;
    case BrakeType::VirtualLow:
      return *config->strategy->motion->virtual_low;
    case BrakeType::VirtualNormal:
      return *config->strategy->motion->virtual_normal;
    case BrakeType::Off:
      assert(false);
  }
}

Field::Field(const vector<Vec>& points_, std::vector<BrakeType> brake_types)
    : Polygon(points_), brake_types_(std::move(brake_types)) {
  assert(points.size() == brake_types_.size());
}

void Field::apply_seg(int i, Robot& robot, double push_k,
                      double push_v_min) const {
  if (brake_types_[i] == BrakeType::Off) return;
  int j = (i + 1) % points.size();
  Segment seg{points[i], points[j]};
  const auto& p = profile_for(brake_types_[i]);
  seg.apply(robot, push_k, push_v_min, p.decel_k, p.wall_limit);
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
      if (brake_types_[i] == BrakeType::Off) continue;
      int j = (i + 1) % points.size();
      Segment seg(points[i], points[j]);
      if (seg.is_proection(robot.position) &&
          seg.correct_side(robot.position)) {
        const auto& p = profile_for(brake_types_[i]);
        seg.apply(robot, push_k, push_v_min, p.decel_k, p.wall_limit);
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