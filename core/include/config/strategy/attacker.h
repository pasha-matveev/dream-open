#pragma once

#include <rapidjson/fwd.h>

#include <memory>

#include "utils/geo/vec.h"

struct Mapper;
struct Switch;

namespace cfg {

struct Attacker {
  Attacker(const rapidjson::Value& doc);
  ~Attacker();
  Attacker(const Attacker&) = delete;
  Attacker& operator=(const Attacker&) = delete;

  bool dubins_enabled;
  double special_height;

  // Быстрый прямой подъезд к мячу, когда враг рядом с мячом.
  bool fast_direct_enabled;             // вкл/выкл быстрый таран врага у мяча
  std::unique_ptr<Mapper> fast_direct;  // дистанция → скорость
  std::unique_ptr<Switch> enemy_near_ball;  // гистерезис близости врага к мячу
  double enemy_min_y;        // см, препятствия ниже игнорируются (зона вратаря)
  long long enemy_latch_ms;  // мс, латч режима при пропаже цели лидаром
  int fast_direct_dribbling;    // скорость дриблера в быстром подъезде
  bool fast_direct_brake_safe;  // тормозить ли у мяча в быстром подъезде
                                // (false — таран без торможения)

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
    std::unique_ptr<Vec>
        pos_1;  // x, y в системе special_pos (y от ворот противника)
    std::unique_ptr<Vec> pos_2;
    std::unique_ptr<Vec> target;
    double turn_1_angle;  // рад, базис field_angle=0, знак мирррится по стороне
    double turn_2_angle;  // рад
    double ready_dist;    // см, "на позиции"
    double ready_angle;   // рад, "ровно повёрнут"
    double drive_max_speed;  // см/с, потолок скорости в DRIVE-фазах
    int dribbling;           // скорость дриблинга в активных фазах
  };
  std::unique_ptr<Kurwa> kurwa;

  // icarus: при захвате мяча враг был вплотную и корпус смотрит в верхнюю
  // половину поля — вместо удара вперёд бьём рикошетом от ближнего бортика.
  struct Icarus {
    // Выключатель режима. false — icarus никогда не активируется.
    bool enabled;
    // y, ниже которого icarus включается всегда — независимо от enemy_near и
    // facing_up. Координата робота в момент захвата мяча.
    double always_below_y;
    // Тип поворота при ударе → KickParams.curved_rotation.
    bool curved_rotation;
    // true — пересчитываем угол рикошета каждый тик от текущего
    // ball_hole_position; false — фиксируем при первом ударе.
    bool recompute_angle_each_tick;
    // Точки-таргеты (зеркалятся через боковую стенку в
    // compute_ricochet_field_angle).
    struct Target {
      double x;
      double y;
    };
    Target target_left;
    Target target_right;
    // Мапер плавного замедления дриблинга перед ударом (прошедшие мс →
    // значение дриблинга).
    std::unique_ptr<Mapper> dribbling_slowdown;
  };
  std::unique_ptr<Icarus> icarus;

  // hobubu: захватив мяч низко без врага рядом, вместо icarus едем вверх вдоль
  // ближнего бортика (прижимая мяч к борту) и завершаем атаку через kurwa.
  struct Hobubu {
    bool enabled;            // мастер-выключатель режима
    double probability;      // [0, 1], шанс выбрать hobubu вместо icarus
    double wall_dist;        // см, удалённость прямой проезда от стенки
    double face_wall_angle;  // рад, ориентация к стенке (знак мирррится по стороне)
    double drive_max_speed;  // см/с, потолок accel_drive в фазах подъезда
    double ready_dist;       // см, "на позиции"
    double ready_angle;      // рад, "ровно повёрнут" в FACE_WALL
    int dribbling;           // скорость дриблинга в активных фазах
  };
  std::unique_ptr<Hobubu> hobubu;
};

}  // namespace cfg
