#pragma once

#include <rapidjson/fwd.h>

#include <memory>
#include <optional>

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
    // Должно быть > 2 см: drive_target сам останавливает робота на 2 см от
    // цели, и при ready_dist < 2 переход DRIVE → PRE_SHOT_TURN не произойдёт.
    double ready_dist;             // см, "на позиции" (DRIVE → PRE_SHOT_TURN)
    double ready_angle;            // рад, "ровно повёрнут"
    double pre_shot_angle_offset;  // рад, добавка к wall_angle для финальной ориентации
    double sweep_angle;            // рад, угол поворота в SPIN
    double rotation_limit;         // rad/s, кэп угловой скорости в SPIN
    // -1 в JSON = nullopt (default value_r). Любое >=0 = явное значение.
    std::optional<int> dribbling_during_spin;
    int kicker_ms;                 // 0 = без киккера
    int spin_timeout_ms;           // 0 = только по углу
  };
  std::unique_ptr<SpinShot> spin_shot;
};

}  // namespace cfg
