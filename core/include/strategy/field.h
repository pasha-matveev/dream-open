#pragma once

#include <SFML/Graphics.hpp>
#include <vector>

#include "field_dims.h"
#include "robot.h"
#include "utils/geo/polygon.h"
#include "utils/geo/vec.h"

using namespace std;

// Центр вражеских ворот в пользовательской системе координат.
// По соглашению координаты вводятся так, что вражеские ворота всегда здесь.
// y=237 — точка прицеливания внутри ворот (не сам lineup ворот).
inline const Vec ENEMY_GOAL_CENTER{field_dims::kWidth / 2, 237.0};
constexpr int FIELD_WIDTH = static_cast<int>(field_dims::kWidth);
constexpr int FIELD_HEIGHT = static_cast<int>(field_dims::kHeight);

// храним по часовой стрелке
class Field : public Polygon {
 private:
  void apply_seg(int i, Robot& robot) const;

 public:
  Field(const vector<Vec>& points_);
  void apply(Robot& robot) const;
};