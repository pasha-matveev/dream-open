#pragma once

#include <vector>

#include "utils/vec.h"

class Object {
 public:
  int h_min, s_min, v_min;
  int h_max, s_max, v_max;
  Vec center;
  double angle;
  double dist;

  Object() = default;
  Object(const std::vector<int> &, const std::vector<int> &);
  virtual ~Object();
  float get_pixels_dist();
};