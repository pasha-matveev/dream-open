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
  // Параллельный массив: brake_enabled_[i] управляет применением
  // Segment::apply на ребре points[i]→points[(i+1)%N]. false означает, что
  // ребро остаётся частью геометрии (inside/find_intersection/dispatch
  // работают как раньше), но Field::apply_seg для него no-op — не клампит
  // скорость робота. Используется для зонных границ keeper'а, у которых нет
  // физической стенки.
  std::vector<bool> brake_enabled_;

  void apply_seg(int i, Robot& robot, double push_k, double push_v_min) const;

 public:
  Field(const vector<Vec>& points_);
  Field(const vector<Vec>& points_, std::vector<bool> brake_enabled);
  void apply(Robot& robot, double push_k, double push_v_min) const;
};