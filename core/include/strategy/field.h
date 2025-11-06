#pragma once

#include <SFML/Graphics.hpp>
#include <vector>

#include "robot.h"
#include "utils/vec.h"

using namespace std;

// храним по часовой стрелке
class Field {
 private:
  vector<Vec> points;
  void apply_seg(int i, Robot& robot) const;

 public:
  bool inside(Robot& robot) const;
  Field(const vector<Vec>&);
  void apply(Robot& robot) const;

  friend class Visualization;
};