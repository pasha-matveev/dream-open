#pragma once

#include <vector>

#include "robot.h"
#include "utils/vec.h"

using namespace std;

// храним по часовой стрелке
class Field {
 private:
  vector<Vec> points;

 public:
  void apply(Robot& robot);
};