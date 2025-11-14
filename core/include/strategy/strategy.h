#pragma once

#include <queue>

#include "robot.h"
#include "strategy/field.h"
#include "tracking/object.h"

const Polygon left_attacker_r{{{0, 194}, {0, 243}, {91, 243}}};
const Polygon right_attacker_r{{{91, 243}, {182, 243}, {182, 194}}};
enum class DubinsSide { NONE, LEFT, RIGHT };

constexpr double DL = 26;
const Field dubins_field{
    {{DL, DL}, {DL, 243 - DL}, {182 - DL, 243 - DL}, {182 - DL, DL}}};

class Strategy {
 private:
  string role;

  long long last_ball_visible = -10000;
  Vec last_ball;
  double last_ball_relative;
  bool active_def = false;
  bool curve_kick = false;

  string kick_status = "none";
  // double target_angle = -10;
  bool reset_kick = true;
  long long slow_tm = -1;

  long long throttle = -1;
  long long fired = -1;

  // common actions
  bool drive_target(Robot& robot, const Vec& target, double k,
                    double max_speed = 120, double min_speed = 1.0);
  void drive_ball(Robot& robot, const Vec& ball);
  void accelerated_dribbling(Robot& robot);
  void desired_dribling(Robot& robot, bool ac_dribling);
  bool kick_dir(Robot& robot, double dir, int power = 70,
                int forward_timeout = 600, bool curved_rotation = true,
                int kick_timeout = 0, double precision = 0.015,
                bool ac_dribling = true);
  bool take_ball(Robot& robot, long long forward_timeout);

  bool reset_turn = false;
  long long turn_time = -1;
  bool turn(Robot& robot, double target_angle, bool curved_rotation,
            bool ac_dribling = true);

  void hit(Robot& robot, Object& goal, int power = 70,
           int forward_timeout = 600, bool curved_rotation = true,
           int kick_timeout = 0, double precision = 0.015,
           bool ac_dribling = true);
  double compute_ricochet(Robot& robot, bool left);

  bool reset_dubins = false;
  Vec dubins_ball;
  DubinsSide dubins_side = DubinsSide::NONE;
  void dubins_hit(Robot& robot, Object& goal, int power, bool control);

  // different strategies
  void run_keeper(Robot& robot, Object& ball, Object& goal, const Field& field);
  void attacker_simple(Robot& robot, Object& goal);
  void run_attacker(Robot& robot, Object& ball, Object& goal, Field& field);
  void run_challenge(Robot& robot, Object& ball, Object& goal);
  void run_kickoff(Robot& robot, Object& ball, Object& goal, bool left);
  void run_test_circle(Robot& robot);
  void run_test_dribling(Robot& robot);
  void run_test(Robot& robot, Object& goal);

 public:
  void run(Robot& robot, Object& ball, Object& goal, Field& field);
  Strategy();
};
