#pragma once

#include <vector>

#include "utils/geo/vec.h"

using namespace std;

class Polygon {
 protected:
  vector<Vec> points;

 public:
  Polygon(const vector<Vec>& points_);
  bool inside(const Vec& p) const;
  double dist(const Vec& p) const;
  pair<Vec, int> find_intersection(const Vec& a, const Vec& dir) const;

  friend class Visualization;
};