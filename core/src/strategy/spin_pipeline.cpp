#include "strategy/spin_pipeline.h"

#include <spdlog/spdlog.h>

#include <cmath>

#include "config/strategy/attacker.h"
#include "robot.h"
#include "strategy/accel_drive.h"
#include "strategy/spin_shot.h"
#include "utils/geo/vec.h"
#include "utils/mapper.h"

bool SpinPipelineController::execute(Robot& robot, bool use_left_hint,
                                     const cfg::Attacker& acfg) {
  reset_pending_ = false;

  if (phase_ == Phase::IDLE) {
    use_left_ = use_left_hint;
    phase_ = Phase::HIDE_TURN;
  }

  const auto& spin_cfg = *acfg.spin_shot;

  // Цели позиции в системе с зеркалированием по x (как в attacker.cpp).
  Vec special_pos_left = {acfg.special_pos->x, 240.0 - acfg.special_pos->y};
  Vec special_pos_right = {182.0 - acfg.special_pos->x,
                           240.0 - acfg.special_pos->y};
  Vec target_pos = use_left_ ? special_pos_left : special_pos_right;

  robot.dribbling = spin_cfg.dribbling->value_r;

  double wall_angle = use_left_ ? +M_PI / 2 : -M_PI / 2;
  double pre_shot_delta =
      (use_left_ ? -1 : 1) * spin_cfg.pre_shot_angle_offset;
  double pre_shot_angle = M_PI + pre_shot_delta;

  switch (phase_) {
    case Phase::IDLE:
      // Unreachable: IDLE переведён в HIDE_TURN выше.
      break;
    case Phase::HIDE_TURN: {
      spdlog::info("SPIN HIDE_TURN");
      double delta = normalize_angle(wall_angle - robot.field_angle);
      robot.vel = {0, 0};
      robot.rotation = delta;
      if (std::abs(delta) < spin_cfg.ready_angle) {
        phase_ = Phase::DRIVE;
      }
      break;
    }
    case Phase::DRIVE: {
      spdlog::info("SPIN DRIVE");
      AccelDriveParams p;
      p.target = target_pos;
      p.max_speed = spin_cfg.drive_max_speed;
      accel_drive_->execute(robot, p);
      robot.rotation = normalize_angle(wall_angle - robot.field_angle);
      if ((target_pos - robot.position).len() < spin_cfg.ready_dist) {
        phase_ = Phase::PRE_SHOT_TURN;
      }
      break;
    }
    case Phase::PRE_SHOT_TURN: {
      spdlog::info("SPIN PRE_SHOT_TURN");
      double delta = normalize_angle(pre_shot_angle - robot.field_angle);
      robot.vel = {0, 0};
      robot.rotation = delta;
      if (std::abs(delta) < spin_cfg.ready_angle) {
        phase_ = Phase::SPIN;
      }
      break;
    }
    case Phase::SPIN: {
      spdlog::info("SPIN");

      SpinShotParams p;
      p.direction = use_left_ ? +1 : -1;
      p.sweep_angle = spin_cfg.sweep_angle;
      p.rotation_limit = spin_cfg.rotation_limit;
      p.dribbling = spin_cfg.dribbling->value_r;
      p.kicker_angle = spin_cfg.kicker_angle;
      p.kicker_force = spin_cfg.kicker_force;
      p.spin_timeout_ms = spin_cfg.spin_timeout_ms;

      bool done = spin_shot_->execute(robot, p);
      if (done) {
        phase_ = Phase::IDLE;
        return true;
      }
      break;
    }
    case Phase::DONE:
      // Unreachable: транзишен SPIN сразу возвращает в IDLE.
      break;
  }

  return false;
}
