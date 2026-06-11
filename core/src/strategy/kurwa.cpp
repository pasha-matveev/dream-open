#include "strategy/kurwa.h"

#include <spdlog/spdlog.h>

#include <cmath>

#include "config/strategy/attacker.h"
#include "robot.h"
#include "strategy/accel_drive.h"
#include "strategy/kick.h"
#include "tracking/object.h"
#include "utils/geo/vec.h"

bool KurwaController::execute(Robot& robot, Object& goal, bool use_left_hint,
                              const cfg::Attacker& acfg) {
  reset_pending_ = false;

  if (phase_ == Phase::IDLE) {
    use_left_ = use_left_hint;
    phase_ = Phase::TURN1;
  }

  const auto& cfg = *acfg.kurwa;
  double sign = use_left_ ? +1.0 : -1.0;
  double turn_1_target = sign * cfg.turn_1_angle;
  double turn_2_target = sign * cfg.turn_2_angle;

  // Зеркалирование позиций по той же конвенции, что и special_pos.
  Vec pos_1 = use_left_ ? Vec{cfg.pos_1->x, 240.0 - cfg.pos_1->y}
                        : Vec{182.0 - cfg.pos_1->x, 240.0 - cfg.pos_1->y};
  Vec pos_2 = use_left_ ? Vec{cfg.pos_2->x, 240.0 - cfg.pos_2->y}
                        : Vec{182.0 - cfg.pos_2->x, 240.0 - cfg.pos_2->y};

  robot.dribbling = cfg.dribbling;

  switch (phase_) {
    case Phase::IDLE:
      // Unreachable: IDLE переведён в TURN1 выше.
      break;
    case Phase::TURN1: {
      spdlog::info("KURWA TURN1");
      double delta = normalize_angle(turn_1_target - robot.field_angle);
      robot.vel = {0, 0};
      robot.rotation = delta;
      if (std::abs(delta) < cfg.ready_angle) {
        phase_ = Phase::DRIVE1;
      }
      break;
    }
    case Phase::DRIVE1: {
      spdlog::info("KURWA DRIVE1");
      AccelDriveParams p;
      p.target = pos_1;
      p.max_speed = cfg.drive_max_speed;
      accel_drive_->execute(robot, p);
      // Удерживаем ориентацию TURN1, как в spin pipeline на DRIVE.
      robot.rotation = normalize_angle(turn_1_target - robot.field_angle);
      if ((pos_1 - robot.position).len() < cfg.ready_dist) {
        phase_ = Phase::TURN2;
      }
      break;
    }
    case Phase::TURN2: {
      spdlog::info("KURWA TURN2");
      double delta = normalize_angle(turn_2_target - robot.field_angle);
      robot.vel = {0, 0};
      robot.rotation = delta;
      if (std::abs(delta) < cfg.ready_angle) {
        phase_ = Phase::DRIVE2;
      }
      break;
    }
    case Phase::DRIVE2: {
      spdlog::info("KURWA DRIVE2");
      AccelDriveParams p;
      p.target = pos_2;
      p.max_speed = cfg.drive_max_speed;
      accel_drive_->execute(robot, p);
      robot.rotation = normalize_angle(turn_2_target - robot.field_angle);
      if ((pos_2 - robot.position).len() < cfg.ready_dist) {
        phase_ = Phase::KICK;
      }
      break;
    }
    case Phase::KICK: {
      spdlog::info("KURWA KICK");
      KickParams params;
      Vec target = use_left_ ? Vec{182.0 - cfg.target->x, 243.0 - cfg.target->y}
                             : Vec{cfg.target->x, 243.0 - cfg.target->y};
      params.relative_dir =
          (target - robot.position).field_angle() - robot.field_angle;
      kick_->execute(robot, params);
      if (robot.kicker_force > 0) {
        spdlog::info("KURWA done: kicker fired");
        phase_ = Phase::IDLE;
        return true;
      }
      break;
    }
    case Phase::DONE:
      // Unreachable: транзишен KICK сразу возвращает в IDLE.
      break;
  }

  return false;
}
