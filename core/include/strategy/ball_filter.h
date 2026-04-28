#pragma once

#include "utils/geo/vec.h"

class BallFilter {
 private:
  Vec pos{0, 0};
  Vec vel{0, 0};
  long long last_update_ms = -1;
  bool initialized = false;

 public:
  BallFilter() = default;

  void predict(double dt);
  void update(const Vec& measurement, long long now_ms);
  void lost(long long now_ms);

  Vec position() const;
  Vec velocity() const { return vel; }
  bool is_initialized() const { return initialized; }
};
