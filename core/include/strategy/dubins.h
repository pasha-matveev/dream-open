#pragma once

#include "utils/geo/vec.h"

class Robot;
class Object;
class Field;
class KickController;
class BallTracker;

class DubinsController {
 public:
  DubinsController(KickController* kick, BallTracker* ball)
      : kick_(kick), ball_(ball) {}

  bool dubins_hit(Robot& robot, Object& goal, Field& field, int power,
                  bool control);

  void on_tick_end() {
    last_dubins_ = cur_dubins_;
    cur_dubins_ = false;
  }
  bool was_active_last_tick() const { return last_dubins_; }

 private:
  KickController* kick_;
  BallTracker* ball_;
  bool last_dubins_ = false;
  bool cur_dubins_ = false;
  int drive_ms_ = -1;
  Vec drive_ball_position_;
};
