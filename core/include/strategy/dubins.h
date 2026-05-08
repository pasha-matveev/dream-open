#pragma once

#include "utils/geo/vec.h"

class Robot;
class Object;
class Field;
class KickController;
class BallTracker;

class DubinsController {
 public:
  DubinsController() = default;

  // Двухфазная инициализация: после конструктора нужно вызвать init() с
  // зависимостями. Это разделяет создание и связывание, чтобы порядок
  // объявления полей в Strategy не был ловушкой.
  void init(KickController* kick, BallTracker* ball) {
    kick_ = kick;
    ball_ = ball;
  }

  bool dubins_hit(Robot& robot, Object& goal, Field& field, int power,
                  bool control);

  // Проверяет, достаточно ли точно робот развёрнут к точке прицеливания
  // (направление от мяча к воротам), чтобы безопасно стрелять из dubins-ветки.
  // Используется как защита от удара в свои ворота, если мяч был захвачен
  // на ранней (TANGENT) фазе dubins, до того как вращение довернуло корпус.
  bool is_aligned_for_kick(Robot& robot, Object& goal);

  void on_tick_end() {
    last_dubins_ = cur_dubins_;
    cur_dubins_ = false;
  }
  bool was_active_last_tick() const { return last_dubins_; }

 private:
  // Направление прицеливания: от позиции мяча к воротам. По возможности
  // используется относительный угол, видимый камерой (если ворота видны и
  // мяч близко); иначе геометрический вектор к центру ворот.
  Vec compute_goal_direction(Robot& robot, Object& goal);

  KickController* kick_ = nullptr;
  BallTracker* ball_ = nullptr;
  bool last_dubins_ = false;
  bool cur_dubins_ = false;
  int drive_ms_ = -1;
  Vec drive_ball_position_;
};
