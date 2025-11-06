#include "strategy/field.h"

#include "strategy/segment.h"

Field::Field(const vector<Vec>& points_) : points(points_) {}

bool Field::inside(Robot& robot) const {
  double ang = 0;
  for (int i = 0; i < points.size(); ++i) {
    int j = (i + 1) % points.size();
    Vec a = points[i] - robot.position;
    Vec b = points[j] - robot.position;
    double angle = normalize_angle(b.field_angle() - a.field_angle());
    ang += angle;
  }
  ang = abs(ang);
  if (ang > M_PI) {
    return true;
  }
  return false;
}

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
  if (inside(robot)) {
    // 1. Ищем ближайшую точку пересечения вектора скорости и сторон
    // многоугольника
    // 2. Ограничиваем по соответствующему отрезку

    Vec np;
    int idx = -1;
    Vec long_vel = robot.vel.resize(10000);
    Segment vel = {robot.position, robot.position + long_vel};

    for (int i = 0; i < points.size(); ++i) {
      int j = (i + 1) % points.size();
      Segment seg(points[i], points[j]);

      if (!seg.intersects_vel(vel)) continue;
      Vec p = seg.intersect_point(vel);
      if (idx == -1 ||
          (p - robot.position).len2() < (np - robot.position).len2()) {
        idx = i;
        np = p;
      }
    }
    assert(idx != -1);
    apply_seg(idx, robot);
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
    assert(idx1 != -1);
    assert(idx2 != -1);
    apply_seg(idx1, robot);
    apply_seg(idx2, robot);
  }
}