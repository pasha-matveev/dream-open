#include "strategy/turn.h"

#include <spdlog/spdlog.h>

#include "config/config.h"
#include "config/strategy.h"
#include "config/strategy/control.h"
#include "robot.h"
#include "utils/millis.h"

bool in_enemy_goal_zone(const Vec& pos) {
  return pos.y > 240 - 60 && 40 < pos.x && pos.x < 182 - 40;
}

bool TurnController::execute(Robot& robot, const TurnParams& params) {
  reset_pending_ = false;
  if (start_time_ < 0) start_time_ = millis();
  double delta = params.target_field_angle - robot.field_angle;
  bool near_enemy_goal = in_enemy_goal_zone(robot.position);

  double wall_dist = DBL_MAX;
  wall_dist = min(wall_dist, robot.position.x);
  wall_dist = min(wall_dist, robot.position.y);
  wall_dist = min(wall_dist, 182 - robot.position.x);
  wall_dist = min(wall_dist, 243 - robot.position.y);
  bool near_wall = wall_dist < config->strategy->control->curved_turn->dist;

  bool curved = params.curved_rotation && !near_enemy_goal && !near_wall;

  int accel_time = curved ? config->strategy->control->curved_turn->accel_time
                          : config->strategy->control->simple_turn->accel_time;

  long long passed = millis() - start_time_;
  double k = min(1.0, (double)passed / accel_time);

  if (curved) {
    if (abs(delta) <= 0.035) {
      robot.vel = {0, 0};
      robot.rotation_limit =
          config->strategy->control->curved_turn->rotation_limit;
    } else {
      Vec vel{robot.field_angle};
      if (delta > 0) {
        vel = vel.turn_right();
        vel = vel.rotate(0.1);
      } else {
        vel = vel.turn_left();
        vel = vel.rotate(-0.1);
      }
      vel = vel.resize(config->strategy->control->curved_turn->vel * k);
      robot.vel = vel;
      robot.rotation_limit =
          config->strategy->control->curved_turn->rotation_limit * k;
    }
  } else {
    robot.vel = {0, 0};
    robot.rotation_limit =
        config->strategy->control->simple_turn->rotation_limit * k;
  }
  robot.rotation = delta;
  robot.dribbling = params.dribbling;
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
