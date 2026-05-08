#pragma once

#include <vector>

#include "utils/geo/vec.h"
#include "utils/switch.h"

using namespace std;

class Polygon {
 protected:
  vector<Vec> points;
  // Гистерезис для hyst_inside: внутри [-2, 2] см от границы держим прошлое
  // значение, чтобы не флипать на шуме трекинга.
  Switch hyst{0.0, 2.0};

 public:
  Polygon(const vector<Vec>& points_);
  bool inside(const Vec& p) const;
  bool hyst_inside(const Vec& p);
  double dist(const Vec& p) const;
  pair<Vec, int> find_intersection(const Vec& a, const Vec& dir) const;

  friend class Visualization;
};