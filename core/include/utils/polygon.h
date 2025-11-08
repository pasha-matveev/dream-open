#pragma once

#include <vector>

#include "utils/vec.h"

using namespace std;

class Polygon {
 protected:
  vector<Vec> points;

 public:
  Polygon(const vector<Vec>& points_);
  bool inside(const Vec& p) const;
};