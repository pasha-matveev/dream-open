#pragma once

#include "utils/vec.h"

class Robot;

class Line {
 private:
  static constexpr double EPS = 1e-8;

 public:
  double a, b, c;
  Line(const Vec& s, const Vec& f);
  friend Vec operator*(const Line& a, const Line& b);
};

class Segment {
 private:
  static constexpr double EPS = 1e-8;

  bool is_proection(const Vec& p) const;

 public:
  Vec a, b;
  Segment(const Vec& a_, const Vec& b_);

  double normal_dist(const Vec& p) const;
  double dist(const Vec& p) const;
  void apply(Robot& robot) const;
  bool has_point(const Vec& p) const;
  bool intersects_vel(const Segment& vel) const;
  Vec intersect_point(const Segment& vel) const;
  Vec nearest_point(const Vec& p) const;
};