#include "strategy/field.h"

#include "strategy/segment.h"

Field::Field(const vector<Vec>& points_) : points(points_) {}

void Field::apply(Robot& robot) const {
  for (int i = 0; i < points.size(); ++i) {
    int j = (i + 1) % points.size();
    Segment seg{points[i], points[j]};
    seg.apply(robot);
  }
}