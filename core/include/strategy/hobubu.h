#pragma once

namespace cfg {
struct Attacker;
}
class Robot;
class Object;
class AccelDriveController;
class KurwaController;

// Hobubu: FACE_WALL → DRIVE_TO_LINE → DRIVE_UP → KURWA.
// Активируется в attacker диспетчере вместо icarus при захвате мяча низко без
// врага рядом (с вероятностью hobubu.probability). Везёт мяч вверх вдоль
// ближнего бортика, затем делегирует финал существующему KurwaController.
class HobubuController {
 public:
  enum class Phase { IDLE, FACE_WALL, DRIVE_TO_LINE, DRIVE_UP, KURWA };

  HobubuController() = default;

  void init(AccelDriveController* accel_drive, KurwaController* kurwa) {
    accel_drive_ = accel_drive;
    kurwa_ = kurwa;
  }

  // use_left_hint фиксируется на переходе IDLE → FACE_WALL. Возвращает true
  // один раз, когда делегированная kurwa выстрелила. Жёсткого таймаута нет
  // (как в kurwa): при провале атаки ждём потери мяча — диспетчер сбросит
  // состояние через emitter=false.
  bool execute(Robot& robot, Object& goal, bool use_left_hint,
               const cfg::Attacker& acfg);

  void mark_for_reset() { reset_pending_ = true; }
  void apply_reset_if_pending() {
    if (reset_pending_) {
      phase_ = Phase::IDLE;
    }
  }

 private:
  AccelDriveController* accel_drive_ = nullptr;
  KurwaController* kurwa_ = nullptr;
  Phase phase_ = Phase::IDLE;
  bool use_left_ = false;
  bool reset_pending_ = false;
};
