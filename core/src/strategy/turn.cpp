#include "strategy/turn.h"

#include <spdlog/spdlog.h>

#include "config/config.h"
#include "config/strategy.h"
#include "robot.h"
#include "strategy/motion.h"
#include "utils/millis.h"

bool in_enemy_goal_zone(const Vec& pos) {
  return pos.y > 240 - 60 && 40 < pos.x && pos.x < 182 - 40;
}

// TODO: параметры поворота в настройках
constexpr int TURN_ACCEL_TIME = 600;
constexpr double CURVED_VEL = 19 * 1.2;
constexpr double CURVED_ROTATION = 15 * 1.2;

bool TurnController::execute(Robot& robot, const TurnParams& params) {
  reset_pending_ = false;
  double delta = params.target_field_angle - robot.field_angle;
  long long passed = millis() - start_time_;
  double k = min(1.0, (double)passed / TURN_ACCEL_TIME);
  bool near_enemy_goal = in_enemy_goal_zone(robot.position);
  bool curved = params.curved_rotation && !near_enemy_goal;
  if (near_enemy_goal) {
    spdlog::info("IN-PLACE TURN");
  }
  if (curved) {
    if (abs(delta) <= 0.035) {
      robot.vel = {0, 0};
      robot.rotation_limit = CURVED_ROTATION;
    } else {
      Vec vel{robot.field_angle};
      if (delta > 0) {
        vel = vel.turn_right();
        vel = vel.rotate(0.1);
      } else {
        vel = vel.turn_left();
        vel = vel.rotate(-0.1);
      }
      // TODO: вынести в настройки
      vel = vel.resize(CURVED_VEL * k);
      robot.vel = vel;
      robot.rotation_limit = CURVED_ROTATION * k;
    }
  } else {
    robot.vel = {0, 0};
    robot.rotation_limit = 15.0 * k;
  }
  robot.rotation = delta;
  desired_dribbling(robot, params.accelerated_dribbling);
  bool low_precision = near_enemy_goal;
  double prec = low_precision ? 0.1 : config->strategy->turn_precision;
  if (low_precision) {
    spdlog::info("LOW PRECISION");
  }
  if (abs(delta) <= prec) {
    // поворот закончен
    return true;
  }
  return false;
}
