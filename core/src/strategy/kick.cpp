#include "strategy/kick.h"

#include <spdlog/spdlog.h>

#include <cassert>

#include "config/config.h"
#include "config/strategy.h"
#include "config/strategy/control.h"
#include "robot.h"
#include "strategy/field.h"
#include "strategy/motion.h"
#include "strategy/turn.h"
#include "tracking/object.h"
#include "utils/mapper.h"
#include "utils/millis.h"

void KickController::execute(Robot& robot, const KickParams& params) {
  assert(turn_ != nullptr && "KickController::init() not called");
  reset_pending_ = false;

  // Переходы
  if (status_ == Status::TIMEOUT) {
    if (millis() >= timeout_stamp_) {
      status_ = Status::KICK;
    }
  }

  // Обработка
  if (status_ == Status::ROTATE) {
    bool finished = turn_->execute(
        robot, {.target_field_angle = robot.field_angle + params.relative_dir,
                .curved_rotation = params.curved_rotation,
                .accelerated_dribbling = params.accelerate_dribbling});

    if (finished) {
      // TODO: kick_timout в настройках
      if (params.kick_timeout) {
        status_ = Status::TIMEOUT;
        // TODO: плавное замедление
        robot.dribbling = config->strategy->dribbling_slow;
        timeout_stamp_ = millis() + params.kick_timeout;
      } else {
        status_ = Status::KICK;
        robot.dribbling = 0;
      }
      robot.rotation = 0;
      robot.vel = {0, 0};
    }

  } else if (status_ == Status::TIMEOUT) {
    robot.dribbling = config->strategy->dribbling_slow;
    robot.vel = {0, 0};
  } else if (status_ == Status::KICK) {
    robot.kicker_force = compute_power(robot.position);
    if (params.kick_timeout) {
      robot.dribbling = 0;
    } else {
      robot.dribbling = config->strategy->dribbling->value_r;
    }
    robot.rotation = 0;
    robot.vel = {0, 0};
  }
}

void KickController::execute_to_goal(Robot& robot, Object& goal,
                                     KickParams params) {
  double target_angle;
  if (goal.camera_visible) {
    target_angle = normalize_angle(goal.relative_angle);
  } else {
    Vec simple_route = ENEMY_GOAL_CENTER - robot.position;
    target_angle =
        normalize_angle(simple_route.field_angle() - robot.field_angle);
  }
  params.relative_dir = target_angle;
  execute(robot, params);
}

double KickController::compute_power(const Vec& position) {
  double distance = (ENEMY_GOAL_CENTER - position).len();
  return config->strategy->control->kick_power->map(distance);
}
