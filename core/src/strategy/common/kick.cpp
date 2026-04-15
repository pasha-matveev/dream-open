#include "strategy/strategy.h"
#include "utils/config.h"
#include "utils/millis.h"

void Strategy::kick(Robot& robot, const KickParams& params) {
  reset_kick = false;

  // Переходы
  if (kick_status == KickStatus::NONE) {
    if (robot.first_time + params.control_time >= millis()) {
      kick_status = KickStatus::ROTATE;
    }
  } else if (kick_status == KickStatus::TIMEOUT) {
    if (millis() >= kick_timeout_stamp) {
      kick_status = KickStatus::KICK;
    }
  }

  // Обработка
  if (kick_status == KickStatus::NONE) {
    long long passed_time = millis() - robot.first_time;
    long long left_time = params.control_time - passed_time;
    double speed = 10.0 * left_time / params.control_time;
    robot.vel = Vec{robot.field_angle}.resize(speed);
    desired_dribling(robot, params.accelerate_dribbling);
    robot.rotation = 0;
    robot.rotation_limit = 0;
  } else if (kick_status == KickStatus::ROTATE) {
    bool finished = turn(
        robot, {.target_field_angle = robot.field_angle + params.relative_dir,
                .curved_rotation = params.curved_rotation,
                .accelerated_dribbling = params.accelerate_dribbling});

    if (finished) {
      if (params.kick_timeout) {
        kick_status = KickStatus::TIMEOUT;
        robot.dribling = config.strategy.dribbling.value_l;
        kick_timeout_stamp = millis() + params.kick_timeout;
      } else {
        kick_status = KickStatus::KICK;
        robot.dribling = 0;
      }
      robot.rotation = 0;
      robot.vel = {0, 0};
    }

  } else if (kick_status == KickStatus::TIMEOUT) {
    robot.rotation = 0;
    robot.dribling = config.strategy.dribbling.value_l;
    robot.vel = {0, 0};
  } else if (kick_status == KickStatus::KICK) {
    robot.kicker_force = params.power;
    robot.dribling = 0;
    robot.rotation = 0;
    robot.vel = {0, 0};
  }
}