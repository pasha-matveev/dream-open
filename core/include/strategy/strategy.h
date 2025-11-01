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
  bool active_def = false;
  bool curve_kick = false;
  double goal_direction = -10;
  long long throttle = -1;

  void run_keeper(Robot& robot, Object& ball, Object& goal);
  void run_attacker(Robot& robot, Object& ball);

  queue<Vec> q;

 public:
  State state = INITIAL;
  void run(Robot& robot, Object& ball, Object& goal, const Field& field);
  Strategy();
};
