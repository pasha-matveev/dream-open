#pragma once

#include <queue>

#include "robot.h"
#include "strategy/field.h"
#include "tracking/object.h"

enum class AttackerRSide { NONE, LEFT, RIGHT };
enum class AttackerRStatus { NONE, R1, MOVE, R2 };

class Strategy {
 private:
  static constexpr int base_dribling = 30;
  static constexpr int max_dribling = 60;
  static constexpr double dribling_duration = 600;

  string role;

  long long last_ball_visible = 0;
  Vec last_ball;
  bool active_def = false;
  bool curve_kick = false;

  string kick_status = "none";
  double target_angle = -10;
  bool reset_kick = true;
  long long slow_tm = -1;

  AttackerRSide attacker_r_side = AttackerRSide::NONE;
  AttackerRStatus attacker_r = AttackerRStatus::NONE;

  long long throttle = -1;
  long long fired = -1;

  // common actions
  void drive_target(Robot& robot, const Vec& target, double k);
  void drive_ball(Robot& robot, const Vec& ball);
  void accelerated_dribbling(Robot& robot);
  void kick_dir(Robot& robot, double dir, int power = 70,
                int forward_timeout = 600, bool curved_rotation = true,
                int kick_timeout = 0, double precision = 0.015);
  void hit(Robot& robot, Object& goal, int power = 70,
           int forward_timeout = 600, bool curved_rotation = true,
           int kick_timeout = 0, double precision = 0.015);
  double compute_ricochet(Robot& robot, bool left);

  // different strategies
  void run_keeper(Robot& robot, Object& ball, Object& goal);
  void run_attacker(Robot& robot, Object& ball, Object& goal);
  void run_challenge(Robot& robot, Object& ball, Object& goal);
  void run_kickoff(Robot& robot, Object& ball, Object& goal, bool left);

 public:
  void run(Robot& robot, Object& ball, Object& goal, const Field& field);
  Strategy();
};
