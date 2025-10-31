#pragma once

#include <queue>

#include "robot.h"
#include "tracking/ball.h"

enum State { INITIAL };

long long millis();

class Strategy {
 private:
  string role;

  int last_ball_visible = 0;
  bool active_def = false;
  bool curve_kick = false;

  void run_keeper(Robot& robot, Ball& ball);
  void run_attacker(Robot& robot, Ball& ball);

  queue<Vec> q;

 public:
  State state = INITIAL;
  void run(Robot& robot, Ball& ball);
  Strategy();
};
