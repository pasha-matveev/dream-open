#pragma once

#include "robot.h"
#include "utils/vec.h"

class Segment {
 private:
  Vec a, b;
  bool is_inside(Robot&);
  double dist(const Vec&);

 public:
  Segment(const Vec&, const Vec&);
  bool proection(const Vec&);
  double total_dist(const Vec&);
  void apply(Robot&);
};