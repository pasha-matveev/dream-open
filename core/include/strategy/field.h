#pragma once

#include <SFML/Graphics.hpp>
#include <vector>

#include "field_dims.h"
#include "robot.h"
#include "strategy/brake_type.h"
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
  // Параллельный массив: brake_types_[i] — тип границы ребра
  // points[i]→points[(i+1)%N], управляет применением Segment::apply.
  //   Off           — Field::apply_seg для ребра no-op: оно остаётся частью
  //                   геометрии (inside/find_intersection работают как
  //                   раньше), но скорость робота не клампится.
  //   Wall          — физический бортик поля; свой профиль (wall_limit,
  //                   decel_k).
  //   VirtualNormal — виртуальная граница, обычное торможение; свой профиль.
  //   VirtualLow    — виртуальная граница, ослабленное торможение; свой
  //                   профиль (push-out по-прежнему не пускает внутрь зоны).
  // Профили (wall_limit + decel_k) берутся из config.strategy.motion.brake.
  std::vector<BrakeType> brake_types_;

  void apply_seg(int i, Robot& robot, double push_k, double push_v_min) const;

 public:
  Field(const vector<Vec>& points_, std::vector<BrakeType> brake_types);
  void apply(Robot& robot, double push_k, double push_v_min) const;
};