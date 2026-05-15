#pragma once

namespace cfg {
struct Attacker;
}
class Robot;
class AccelDriveController;
class SpinShotController;

// Пайплайн HIDE_TURN → DRIVE → PRE_SHOT_TURN → SPIN, который раньше жил
// inline в attacker.cpp. Делегирует к SpinShotController на фазе SPIN.
// Не путать со SpinShotController — этот контроллер выстраивает
// последовательность фаз вокруг выстрела, а SpinShotController крутит сам
// поворот в финальной фазе.
class SpinPipelineController {
 public:
  enum class Phase { IDLE, HIDE_TURN, DRIVE, PRE_SHOT_TURN, SPIN, DONE };

  SpinPipelineController() = default;

  void init(AccelDriveController* accel_drive, SpinShotController* spin_shot) {
    accel_drive_ = accel_drive;
    spin_shot_ = spin_shot;
  }

  // use_left_hint фиксируется на переходе IDLE → HIDE_TURN. Возвращает
  // true один раз, когда SPIN завершён.
  bool execute(Robot& robot, bool use_left_hint, const cfg::Attacker& acfg);

  void mark_for_reset() { reset_pending_ = true; }
  void apply_reset_if_pending() {
    if (reset_pending_) {
      phase_ = Phase::IDLE;
    }
  }

 private:
  AccelDriveController* accel_drive_ = nullptr;
  SpinShotController* spin_shot_ = nullptr;
  Phase phase_ = Phase::IDLE;
  bool use_left_ = false;
  bool reset_pending_ = false;
};
