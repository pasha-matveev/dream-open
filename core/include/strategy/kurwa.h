#pragma once

namespace cfg {
struct Attacker;
}
class Robot;
class Object;
class AccelDriveController;
class KickController;

// Kurwa: TURN1 → DRIVE1 → TURN2 → DRIVE2 → KICK.
// Активируется в attacker диспетчере с вероятностью kurwa.probability
// вместо spin-pipeline в момент захвата мяча в special_zone.
class KurwaController {
 public:
  enum class Phase { IDLE, TURN1, DRIVE1, TURN2, DRIVE2, KICK, DONE };

  KurwaController() = default;

  void init(AccelDriveController* accel_drive, KickController* kick) {
    accel_drive_ = accel_drive;
    kick_ = kick;
  }

  // use_left_hint фиксируется на переходе IDLE → TURN1. Возвращает true
  // один раз, когда KICK выстрелил. Если выстрел не происходит, ждём
  // потери мяча (естественный сброс через emitter=false из attacker).
  // Жёсткого таймаута нет намеренно: KICK-фаза может попасть внутрь
  // special_zone, и таймаут привёл бы к мгновенному перезапуску атаки.
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
  KickController* kick_ = nullptr;
  Phase phase_ = Phase::IDLE;
  bool use_left_ = false;
  bool reset_pending_ = false;
};
