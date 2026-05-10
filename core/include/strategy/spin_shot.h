#pragma once

#include <cmath>
#include <optional>

class Robot;

struct SpinShotParams {
  // Знак угловой скорости. +1 = field_angle растёт (влево), -1 = убывает
  // (вправо).
  int direction = +1;
  // Угол поворота за один spin (рад) (0, π].
  double sweep_angle = M_PI / 2;
  // Кэп угловой скорости на Arduino во время spin.
  double rotation_limit = 30;
  // Дриблер во время spin. nullopt = config->strategy->dribbling->value_r.
  std::optional<int> dribbling = std::nullopt;
  // Через сколько мс после старта spin выстрелить киккером. 0 = не стрелять.
  int kicker_ms = 0;
  // Сила киккера, явное значение из конфига
  int kicker_force = 0;
  // Жёсткий таймаут spin. 0 = таймаут выключен
  int spin_timeout_ms = 0;
};

class SpinShotController {
 public:
  enum class Status { IDLE, SPIN, DONE };

  SpinShotController() = default;

  // Возвращает true, когда spin завершён.
  bool execute(Robot& robot, const SpinShotParams& params);

  void mark_for_reset() { reset_pending_ = true; }
  void apply_reset_if_pending() {
    if (reset_pending_) {
      status_ = Status::IDLE;
      start_ms_ = -1;
      start_field_angle_ = 0;
      target_field_angle_ = 0;
      kicker_fired_ = false;
    }
  }

 private:
  Status status_ = Status::IDLE;
  long long start_ms_ = -1;
  double start_field_angle_ = 0;
  double target_field_angle_ = 0;
  bool kicker_fired_ = false;
  bool reset_pending_ = false;
};
