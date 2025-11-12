#include "strategy/field.h"

#include <spdlog/spdlog.h>

#include "strategy/segment.h"

Field::Field(const vector<Vec>& points_) : Polygon(points_) {}

void Field::apply_seg(int i, Robot& robot) const {
  int j = (i + 1) % points.size();
  Segment seg{points[i], points[j]};
  seg.apply(robot);
}

void Field::apply(Robot& robot) const {
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
      apply_seg(idx, robot);
    } else {
      spdlog::error("Not found any segment by intersection");
    }

    for (int i = 0; i < points.size(); ++i) {
      int j = (i + 1) % points.size();
      Segment seg(points[i], points[j]);
      if (seg.is_proection(robot.position) &&
          seg.correct_side(robot.position)) {
        seg.apply(robot);
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
      apply_seg(idx1, robot);
    } else {
      spdlog::error("Not found first closest segment: {} {}", robot.position.x,
                    robot.position.y);
    }
    if (idx2 != -1) {
      apply_seg(idx2, robot);
    } else {
      spdlog::error("Not found second closest segment: {} {}", robot.position.x,
                    robot.position.y);
    }
  }
}