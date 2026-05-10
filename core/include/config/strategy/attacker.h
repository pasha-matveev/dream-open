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
    // Должно быть >= 2 см:
    double ready_dist;             // см, "на позиции" (DRIVE → PRE_SHOT_TURN)
    double ready_angle;            // рад, "ровно повёрнут"
    double pre_shot_angle_offset;  // рад, поворот в сторону стены перед ударом
    double sweep_angle;            // рад, угол поворота в SPIN
    double rotation_limit;         // angle/s, кэп угловой скорости в SPIN

    std::optional<int>
        dribbling_during_spin;  // -1 в JSON = nullopt (default value_r). Любое
                                // >=0 = явное значение.
    double kicker_angle;        // рад, 0 = без киккера
    int kicker_force;           // сила удара
    int spin_timeout_ms;        // 0 = только по углу
    double drive_max_speed;     // см/с, потолок скорости в фазе DRIVE
  };
  std::unique_ptr<SpinShot> spin_shot;
};

}  // namespace cfg
