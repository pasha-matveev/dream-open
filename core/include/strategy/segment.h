#pragma once

#include "robot.h"
#include "utils/vec.h"

class Segment {
 private:
  Vec a, b;
  bool is_inside(Robot&);
  double dist(Vec&);

 public:
  Segment(const Vec&, const Vec&);
  void apply(Robot&);
};