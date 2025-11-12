#include <spdlog/spdlog.h>

#include "strategy/strategy.h"
#include "utils/config.h"
#include "utils/millis.h"

bool Strategy::kick_dir(Robot& robot, double dir, int power,
                        int forward_timeout, bool curved_rotation,
                        int kick_timeout, double precision) {
  reset_kick = false;
  auto old_status = kick_status;
  if (kick_status == "none") {
    spdlog::info("NONE");
    long long passed_time = millis() - robot.first_time;
    long long left_time = forward_timeout - passed_time;
    if (left_time <= 0) {
      kick_status = "rotate";
    } else {
      Vec vel{-1 * sin(robot.field_angle) * 10.0 * left_time / forward_timeout,
              cos(robot.field_angle) * 10.0 * left_time / forward_timeout};
      robot.vel = vel;
      accelerated_dribbling(robot);
      robot.rotation_limit = 0;
    }

  } else if (kick_status == "rotate") {
    spdlog::info("ROTATE");
    dir -= 0.01;

    if (abs(dir) <= precision) {
      if (kick_timeout) {
        kick_status = "slow";
        robot.dribling = 15;
        slow_tm = millis() + kick_timeout;
      } else {
        kick_status = "kick";
        robot.dribling = 0;
      }
      robot.rotation = 0;
      robot.vel = {0, 0};
    } else {
      if (curved_rotation) {
        Vec vel{robot.field_angle};
        if (dir > 0) {
          vel = vel.turn_right();
          vel = vel.rotate(0.1);
        } else {
          vel = vel.turn_left();
          vel = vel.rotate(-0.1);
        }
        if (abs(dir) <= 0.05) {
          robot.vel = {0, 0};
          robot.rotation_limit = 15;
        } else {
          vel = vel.resize(19);
          robot.vel = vel;
          robot.rotation_limit = 15;
        }
      } else {
        robot.vel = {0, 0};
        robot.rotation_limit = 15;
      }
      robot.rotation = dir;
      accelerated_dribbling(robot);
    }

  } else if (kick_status == "slow") {
    spdlog::info("SLOW");
    robot.rotation = 0;
    robot.dribling = config.strategy.slow_dribling;
    robot.vel = {0, 0};
    if (slow_tm < millis()) {
      kick_status = "kick";
    }

  } else if (kick_status == "kick" || kick_status == "ready") {
    spdlog::info("KICK");
    robot.kicker_force = power;
    robot.dribling = 0;
    robot.rotation = 0;
    robot.vel = {0, 0};
    kick_status = "ready";
  }
  return old_status == "ready";
}