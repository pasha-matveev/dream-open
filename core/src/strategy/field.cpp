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
  cout << "ang " << ang << endl;
  if (ang > M_PI) {
    return true;
  }
  return false;
}

void Field::apply(Robot& robot) const {
  // if (inside(robot)) {
  //   return;
  // }
  for (int i = 0; i < points.size(); ++i) {
    int j = (i + 1) % points.size();
    Segment seg{points[i], points[j]};
    seg.apply(robot);
  }
}