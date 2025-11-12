#pragma once

#include "utils/geo/vec.h"

class Circle {
 private:
  double tangent_angle(const Vec& p) const;
  double tangent_len(const Vec& p) const;

 public:
  Vec center;
  double r;

  bool inside(const Vec& p) const;
  double dist(const Vec& p) const;
  Vec tangent_left(const Vec& p) const;
  Vec tangent_right(const Vec& p) const;
};