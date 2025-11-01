#pragma once

#include <queue>

#include "robot.h"
#include "strategy/field.h"
#include "tracking/object.h"

enum State { INITIAL };

long long millis();

class Strategy {
 private:
  string role;

  int last_ball_visible = 0;
  Vec last_ball;
  bool active_def = false;
  bool curve_kick = false;
  double target_angle = -10;
  long long throttle = -1;
  long long fired = -1;
  long long rotated_tm = -1;

  void run_keeper(Robot& robot, Object& ball, Object& goal);
  void run_attacker(Robot& robot, Object& ball, Object& goal);

  queue<Vec> q;

 public:
  State state = INITIAL;
  void run(Robot& robot, Object& ball, Object& goal, const Field& field);
  Strategy();
};
