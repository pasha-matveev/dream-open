#pragma once

#include <SFML/Graphics.hpp>
#include <vector>

#include "robot.h"
#include "utils/geo/polygon.h"
#include "utils/geo/vec.h"

using namespace std;

// Центр вражеских ворот в пользовательской системе координат.
// По соглашению координаты вводятся так, что вражеские ворота всегда здесь.
inline const Vec ENEMY_GOAL_CENTER{91.0, 237.0};

// храним по часовой стрелке
class Field : public Polygon {
 private:
  void apply_seg(int i, Robot& robot) const;

 public:
  Field(const vector<Vec>& points_);
  void apply(Robot& robot) const;
};