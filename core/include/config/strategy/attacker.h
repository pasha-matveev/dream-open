#pragma once

#include <rapidjson/fwd.h>

#include <memory>

namespace cfg {

struct Attacker {
  Attacker(const rapidjson::Value& doc);
  ~Attacker();
  Attacker(const Attacker&) = delete;
  Attacker& operator=(const Attacker&) = delete;

  double border;
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
};

}  // namespace cfg
