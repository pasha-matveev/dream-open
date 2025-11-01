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

 public:
  bool inside(Robot&) const;
  Field(const vector<Vec>&);
  void apply(Robot&) const;

  friend class Visualization;
};