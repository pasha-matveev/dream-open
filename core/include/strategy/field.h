#pragma once

#include <SFML/Graphics.hpp>
#include <vector>

#include "robot.h"
#include "utils/geo/polygon.h"
#include "utils/geo/vec.h"

using namespace std;

// храним по часовой стрелке
class Field : public Polygon {
 private:
  void apply_seg(int i, Robot& robot) const;

 public:
  Field(const vector<Vec>& points_);
  void apply(Robot& robot) const;

  friend class Visualization;
};