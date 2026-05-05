#pragma once

#include <memory>

#include "robot.h"
#include "strategy/field.h"
#include "tracking/object.h"

class BallTracker;
class TurnController;
class KickController;
class DubinsController;

class Strategy {
 private:
  // Контроллеры. Связи между ними настраиваются в Strategy() через init() —
  // см. strategy.cpp. Порядок объявления тут роли не играет.
  std::unique_ptr<BallTracker> ball_;
  std::unique_ptr<TurnController> turn_;
  std::unique_ptr<KickController> kick_;
  std::unique_ptr<DubinsController> dubins_;

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

  // Root strategies
  void run_keeper(Robot& robot, Object& ball, Object& goal, Field& field);
  void run_attacker(Robot& robot, Object& ball, Object& goal, Field& field);
  void run_challenge(Robot& robot, Object& ball, Object& goal);
  void run_test_mirror(Robot& robot, Object& ball);
  void run_test_circle(Robot& robot);
  void run_test_dribling(Robot& robot);
  void run_test(Robot& robot, Object& goal);

 public:
  Strategy();
  ~Strategy();
  Strategy(const Strategy&) = delete;
  Strategy& operator=(const Strategy&) = delete;

  void run(Robot& robot, Object& ball, Object& goal, Field& field);
  double get_last_dt() const { return last_dt; }
};
