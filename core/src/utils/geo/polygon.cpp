#include "utils/geo/polygon.h"

#include "strategy/segment.h"

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

bool Polygon::hyst_inside(const Vec& p) {
  double d = dist(p);
  double signed_d = inside(p) ? d : -d;
  return hyst.compute(-signed_d);
}

double Polygon::dist(const Vec& p) const {
  double ans = -1;
  bool has_ans = false;
  for (int i = 0; i < points.size(); ++i) {
    int j = (i + 1) % points.size();
    Segment seg(points[i], points[j]);
    double d = seg.dist(p);
    assert(d >= 0);
    if (!has_ans || d <= ans) {
      ans = d;
      has_ans = true;
    }
  }
  return ans;
}

pair<Vec, int> Polygon::find_intersection(const Vec& a, const Vec& dir) const {
  Vec np;
  int idx = -1;

  Segment s(a, a + dir);

  for (int i = 0; i < points.size(); ++i) {
    int j = (i + 1) % points.size();
    Segment seg(points[i], points[j]);

    if (!(seg * s)) continue;
    Vec p = seg.intersect_point(s);
    if (idx == -1 || (p - a).len2() < (np - a).len2()) {
      idx = i;
      np = p;
    }
  }
  return {np, idx};
}