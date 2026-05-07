#pragma once

class Object;
class Robot;
class TurnController;
class Vec;

struct KickParams {
  double relative_dir;
  bool curved_rotation = true;
  int kick_timeout = 0;
  double precision = 0.015;
  bool accelerate_dribbling = true;
};

class KickController {
 public:
  enum class Status { NONE, ROTATE, TIMEOUT, KICK, READY };

  KickController() = default;

  // Двухфазная инициализация: после конструктора нужно вызвать init() с
  // зависимостями. Это разделяет создание и связывание, чтобы порядок
  // объявления полей в Strategy не был ловушкой.
  void init(TurnController* turn) { turn_ = turn; }

  void execute(Robot& robot, const KickParams& params);
  void execute_to_goal(Robot& robot, Object& goal, KickParams params);

  void mark_for_reset() { reset_pending_ = true; }
  void apply_reset_if_pending() {
    if (reset_pending_) {
      status_ = Status::NONE;
      timeout_stamp_ = -10000;
    }
  }

  static double compute_power(const Vec& position);

 private:
  TurnController* turn_ = nullptr;
  Status status_ = Status::NONE;
  long long timeout_stamp_ = -10000;
  bool reset_pending_ = false;
};
