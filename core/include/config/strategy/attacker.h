#pragma once

#include <rapidjson/fwd.h>

#include <memory>

struct Mapper;

namespace cfg {

struct Attacker {
  Attacker(const rapidjson::Value& doc);
  ~Attacker();
  Attacker(const Attacker&) = delete;
  Attacker& operator=(const Attacker&) = delete;

  bool dubins_enabled;
  double special_height;

  struct SpecialPos {
    double x, y;
  };
  std::unique_ptr<SpecialPos> special_pos;

  struct Ff {
    bool enabled;
    double v_min;
    long long stale_ms;
    double gain;
  };
  std::unique_ptr<Ff> ff;

  struct SpinShot {
    // Должно быть >= 2 см:
    double ready_dist;             // см, "на позиции" (DRIVE → PRE_SHOT_TURN)
    double ready_angle;            // рад, "ровно повёрнут"
    double pre_shot_angle_offset;  // рад, поворот в сторону стены перед ударом
    double sweep_angle;            // рад, угол поворота в SPIN
    double rotation_limit;         // angle/s, кэп угловой скорости в SPIN

    std::unique_ptr<Mapper> dribbling;  // скорость дриблинга
    double kicker_angle;                // рад, 0 = без киккера
    int kicker_force;                   // сила удара
    int spin_timeout_ms;                // 0 = только по углу
    double drive_max_speed;             // см/с, потолок скорости в фазе DRIVE
  };
  std::unique_ptr<SpinShot> spin_shot;

  struct Kurwa {
    double probability;  // [0, 1], вероятность выбрать kurwa вместо spin
    struct Pos {
      double x, y;
    };
    std::unique_ptr<Pos> pos_1;  // x, y в системе special_pos (y от ворот противника)
    std::unique_ptr<Pos> pos_2;
    double turn_1_angle;     // рад, базис field_angle=0, знак мирррится по стороне
    double turn_2_angle;     // рад
    double ready_dist;       // см, "на позиции"
    double ready_angle;      // рад, "ровно повёрнут"
    double drive_max_speed;  // см/с, потолок скорости в DRIVE-фазах
    int dribbling;           // скорость дриблинга в активных фазах
  };
  std::unique_ptr<Kurwa> kurwa;
};

}  // namespace cfg
