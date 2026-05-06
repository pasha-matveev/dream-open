#pragma once

#include "strategy/ball_filter.h"
#include "tracking/object.h"
#include "utils/geo/vec.h"

class Robot;

class BallTracker {
 public:
  void update(Robot& robot, Object& ball, double dt, long long now,
              bool interactive_mode);

  Vec position() const { return last_position_; }
  Vec velocity() const { return filter_.velocity(); }
  long long last_visible_ms() const { return last_visible_; }
  bool recently_visible(long long now, long long window_ms) const {
    return now - last_visible_ < window_ms;
  }
  double relative_angle(const Robot& robot) const;

 private:
  long long last_visible_ = -10000;
  Vec last_position_;
  BallFilter filter_;
};
