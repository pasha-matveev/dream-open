#pragma once

class Robot;

struct TurnParams {
  double target_field_angle;
  bool curved_rotation = true;
  bool accelerated_dribbling = true;
};

class TurnController {
 public:
  bool execute(Robot& robot, const TurnParams& params);
  void mark_for_reset() { reset_pending_ = true; }
  void apply_reset_if_pending() {
    if (reset_pending_) start_time_ = -1;
  }

 private:
  long long start_time_ = -1;
  bool reset_pending_ = false;
};
