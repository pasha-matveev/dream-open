#pragma once

#include "utils/geo/vec.h"

class Robot;

class Line {
 public:
  double a, b, c;
  Line(const Vec& s, const Vec& f);
  friend Vec operator*(const Line& a, const Line& b);
};

class Segment {
 public:
  Vec a, b;
  Segment(const Vec& a_, const Vec& b_);

  bool is_proection(const Vec& p) const;
  bool correct_side(const Vec& p) const;
  double normal_dist(const Vec& p) const;
  double dist(const Vec& p) const;
  void apply(Robot& robot) const;
  bool has_point(const Vec& p) const;
  bool intersects_vel_deprecated(const Segment& vel) const;
  Vec intersect_point(const Segment& vel) const;
  Vec nearest_point(const Vec& p) const;

  friend bool operator*(const Segment& a, const Segment& b);
};