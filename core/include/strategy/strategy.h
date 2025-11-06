#pragma once

#include <queue>

#include "robot.h"
#include "strategy/field.h"
#include "tracking/object.h"

class Strategy {
 private:
  string role;

  int last_ball_visible = 0;
  Vec last_ball;
  bool active_def = false;
  bool curve_kick = false;

  string target_status = "none";
  double target_angle = -10;
  bool reset_target = true;
  long long slow_tm = -1;

  long long throttle = -1;
  long long fired = -1;

  // common actions
  void drive_target(Robot& robot, const Vec& target, double k);
  void drive_ball(Robot& robot, const Vec& ball);
  void hit(Robot& robot, Object& goal, int forward_timeout = 0,
           bool curved_rotation = true, double rotation_speed = 25,
           int kick_timeout = 0, int power = 70, double precision = 0.09);

  // different strategies
  void run_keeper(Robot& robot, Object& ball, Object& goal);
  void run_attacker(Robot& robot, Object& ball, Object& goal);
  void run_challenge(Robot& robot, Object& ball, Object& goal);

 public:
  void run(Robot& robot, Object& ball, Object& goal, const Field& field);
  Strategy();
};
