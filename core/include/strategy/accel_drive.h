#pragma once

#include "utils/geo/vec.h"

class Robot;

struct AccelDriveParams {
  Vec target;
  double max_speed = 120;
  double min_speed = 0;
  double prec = 2;
};

// Плавный разгон (предполагается для использования с захваченным мячом)
class AccelDriveController {
 public:
  bool execute(Robot& robot, const AccelDriveParams& params);
  void mark_for_reset() { reset_pending_ = true; }
  void apply_reset_if_pending() {
    if (reset_pending_) {
      current_speed_cap_ = 0;
      last_tick_ms_ = -1;
    }
  }

 private:
  double current_speed_cap_ = 0;
  long long last_tick_ms_ = -1;
  bool reset_pending_ = false;
};
