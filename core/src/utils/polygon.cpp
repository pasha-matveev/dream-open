#include "utils/polygon.h"

Polygon::Polygon(const vector<Vec>& points_) : points(points_) {}

bool Polygon::inside(const Vec& p) const {
  double ang = 0;
  for (int i = 0; i < points.size(); ++i) {
    int j = (i + 1) % points.size();
    Vec a = points[i] - p;
    Vec b = points[j] - p;
    double angle = normalize_angle(b.field_angle() - a.field_angle());
    ang += angle;
  }
  ang = abs(ang);
  if (ang > M_PI) {
    return true;
  }
  return false;
}