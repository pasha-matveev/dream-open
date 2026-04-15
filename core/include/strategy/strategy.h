#pragma once

#include <queue>

#include "robot.h"
#include "strategy/field.h"
#include "tracking/object.h"

// const Polygon left_attacker_r{{{0, 194}, {0, 243}, {91, 243}}};
// const Polygon right_attacker_r{{{91, 243}, {182, 243}, {182, 194}}};

constexpr double DL = 26;
const Field dubins_field{
    {{DL, DL}, {DL, 243 - DL}, {182 - DL, 243 - DL}, {182 - DL, DL}}};

class Strategy {
 private:
  string role;

  // Остановить робота до этого времени
  long long stop_until = -1;

  // Последний раз, когда видели мяч
  long long last_ball_visible = -10000;
  Vec last_ball_position;
  Vec last_ball_relative_angle;

  // Kick
  enum class KickStatus { NONE, ROTATE, TIMEOUT, KICK, READY };
  KickStatus kick_status = KickStatus::NONE;
  long long kick_timeout_stamp = -10000;
  bool reset_kick = false;

  // Turn
  long long turn_start_time = -1;
  bool reset_turn = false;

  // common actions
  bool drive_target(Robot& robot, const Vec& target, double k,
                    double max_speed = 120, double min_speed = 1.0);
  void drive_ball(Robot& robot, const Vec& ball);
  void accelerated_dribbling(Robot& robot);
  void desired_dribling(Robot& robot, bool ac_dribling);

  // kick
  struct KickParams {
    double relative_dir;
    int power = 70;
    int control_time = 600;
    bool curved_rotation = true;
    int kick_timeout = 0;
    double precision = 0.015;
    bool accelerate_dribbling = true;
  };
  void kick(Robot& robot, const KickParams& params);
  void kick_to_goal(Robot& robot, Object& goal, KickParams params);

  bool take_ball(Robot& robot, long long forward_timeout);

  // turn
  struct TurnParams {
    double target_field_angle;
    bool curved_rotation = true;
    bool accelerated_dribbling = true;
  };
  bool turn(Robot& robot, const TurnParams& params);

  double compute_ricochet(Robot& robot, bool left);

  void dubins_hit(Robot& robot, Object& goal, int power, bool control);

  // Root strategies
  void run_keeper(Robot& robot, Object& ball, Object& goal, const Field& field);
  void run_attacker(Robot& robot, Object& ball, Object& goal, Field& field);
  void run_challenge(Robot& robot, Object& ball, Object& goal);
  void run_test_circle(Robot& robot);
  void run_test_dribling(Robot& robot);
  void run_test(Robot& robot, Object& goal);

 public:
  void run(Robot& robot, Object& ball, Object& goal, Field& field);
  Strategy();
};
