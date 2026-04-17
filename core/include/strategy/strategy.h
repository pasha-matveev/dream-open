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

  // Был ли уже принят хотя бы один кадр лидара (после этого переходим на EMA).
  bool has_lidar_fix = false;
  // Момент начала предыдущего тика стратегии (для вычисления реального dt).
  long long last_tick_ms = -1;
  // Последний вычисленный dt, с ограничением [0, 0.1] сек.
  double last_dt = 0.0;
  // Скользящее среднее dt для логирования фактического FPS.
  double avg_dt = 0.0;
  long long last_fps_log_ms = 0;

  // Последний раз, когда видели мяч
  long long last_ball_visible = -10000;
  Vec last_ball_position;
  double last_ball_relative_angle(Robot& robot) {
    return normalize_angle((last_ball_position - robot.position).field_angle() -
                           robot.field_angle);
  }

  // Kick
  enum class KickStatus { NONE, ROTATE, TIMEOUT, KICK, READY };
  KickStatus kick_status = KickStatus::NONE;
  long long kick_timeout_stamp = -10000;
  bool reset_kick = false;
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

  // Turn
  long long turn_start_time = -1;
  bool reset_turn = false;
  struct TurnParams {
    double target_field_angle;
    bool curved_rotation = true;
    bool accelerated_dribbling = true;
  };
  bool turn(Robot& robot, const TurnParams& params);

  // Dubins
  bool last_dubins = false;  // Был ли dubins вызван в предыдущей итерации
  bool cur_dubins = false;   // Был ли dubins вызван в текущей итерации
  bool dubins_hit(Robot& robot, Object& goal, Field& field, int power,
                  bool control);

  // common actions
  bool drive_target(Robot& robot, const Vec& target, double k,
                    double max_speed = 120, double min_speed = 0);
  void drive_ball(Robot& robot, const Vec& ball);
  void accelerated_dribbling(Robot& robot);
  void desired_dribling(Robot& robot, bool ac_dribling);

  bool take_ball(Robot& robot, long long forward_timeout);

  double compute_ricochet(Robot& robot, bool left);

  // Root strategies
  void run_keeper(Robot& robot, Object& ball, Object& goal, Field& field);
  void run_attacker(Robot& robot, Object& ball, Object& goal, Field& field);
  void run_challenge(Robot& robot, Object& ball, Object& goal);
  void run_test_circle(Robot& robot);
  void run_test_dribling(Robot& robot);
  void run_test(Robot& robot, Object& goal);

 public:
  void run(Robot& robot, Object& ball, Object& goal, Field& field);
  double get_last_dt() const { return last_dt; }
  Strategy();
};
