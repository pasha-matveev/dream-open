#pragma once

#include <memory>

#include "robot.h"
#include "strategy/field.h"
#include "tracking/object.h"

class BallTracker;
class TurnController;
class KickController;
class DubinsController;
class SpinShotController;
class SpinPipelineController;
class KurwaController;
class HobubuController;
class AccelDriveController;
class TestController;
class VigilansController;

class Strategy {
 private:
  // Контроллеры. Связи между ними настраиваются в Strategy() через init() —
  // см. strategy.cpp. Порядок объявления тут роли не играет.
  std::unique_ptr<BallTracker> ball_;
  std::unique_ptr<TurnController> turn_;
  std::unique_ptr<KickController> kick_;
  std::unique_ptr<DubinsController> dubins_;
  std::unique_ptr<SpinShotController> spin_shot_;
  std::unique_ptr<SpinPipelineController> spin_pipeline_;
  std::unique_ptr<KurwaController> kurwa_;
  std::unique_ptr<HobubuController> hobubu_;
  std::unique_ptr<AccelDriveController> accel_drive_;
  // Тестовые/калибровочные роли вынесены в TestController, чтобы добавление
  // новой тест-роли не требовало пересборки всех TU, включающих strategy.h.
  std::unique_ptr<TestController> test_;
  // Режим бдительности вратаря (vigilans).
  std::unique_ptr<VigilansController> vigilans_;

  string role;

  // Остановить робота до этого времени
  long long stop_until = -1;

  // Был ли уже принят хотя бы один кадр лидара (после этого переходим на EMA).
  bool has_lidar_fix = false;
  // Когда последний раз видели свои ворота (мс). Используется для timeout
  // fallback к field_angle=0 в PAUSE.
  long long last_own_goal_seen_ms = -1;
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
  void run_kickoff(Robot& robot, Object& ball, Object& goal, Field& field,
                   bool left);
  void run_meme(Robot& robot);

 public:
  Strategy();
  ~Strategy();
  Strategy(const Strategy&) = delete;
  Strategy& operator=(const Strategy&) = delete;

  void run(Robot& robot, Object& ball, Object& goal, Object& own_goal,
           Field& field);
  double get_last_dt() const { return last_dt; }
};
