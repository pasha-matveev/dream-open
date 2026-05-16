#pragma once

#include <SFML/Graphics.hpp>
#include <vector>

#include "field_dims.h"
#include "robot.h"
#include "strategy/brake_mode.h"
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
  // Параллельный массив: brake_modes_[i] управляет применением
  // Segment::apply на ребре points[i]→points[(i+1)%N].
  //   Normal — обычное торможение (decel_k).
  //   Off    — Field::apply_seg для ребра no-op: оно остаётся частью
  //            геометрии (inside/find_intersection работают как раньше),
  //            но скорость робота не клампится. Виртуальные границы зоны
  //            keeper'а, у которых нет физической стенки.
  //   Low    — ослабленное торможение (decel_k_low вместо decel_k). Верхняя
  //            граница выреза зоны вратаря у нападающего: не мешает погоне
  //            за врагом, но push-out по-прежнему не пускает внутрь зоны.
  std::vector<BrakeMode> brake_modes_;

  // decel_k для ребра с данным режимом: Low → decel_k_low, иначе decel_k.
  static double decel_k_for(BrakeMode mode);

  void apply_seg(int i, Robot& robot, double push_k, double push_v_min) const;

 public:
  Field(const vector<Vec>& points_);
  Field(const vector<Vec>& points_, std::vector<BrakeMode> brake_modes);
  void apply(Robot& robot, double push_k, double push_v_min) const;
};