#include <spdlog/spdlog.h>

#include <algorithm>
#include <cmath>
#include <optional>

#include "strategy/strategy.h"
#include "utils/millis.h"

void Strategy::run_keeper(Robot& robot, Object& ball, Object& goal) {
  if (millis() - last_ball_visible < 1000 || robot.emitter) {
    if (last_ball.y < 90) {
      active_def = true;
    } else if (last_ball.y > 95) {
      active_def = false;
    }

    if (robot.emitter) {
      active_def = true;
    }

    if (active_def) {
      robot.dribling = 60;
      if (robot.emitter) {
        spdlog::info("HIT");
        hit(robot, goal);
      } else {
        spdlog::info("DRIVE");
        drive_ball(robot, last_ball);
      }
    } else {
      spdlog::info("PASSIVE");
      Vec target;
      if (last_ball.y > 50.0) {
        Vec our{91, 0};
        double delta_x = last_ball.x - our.x;
        double delta_y1 = last_ball.y;
        double delta_y2 = 50;
        double x1 = delta_x / delta_y1 * delta_y2;
        double final_x = x1 + our.x;
        target = {clamp(final_x, 50.0, 130.0), 52.0};
      } else {
        target = {clamp(last_ball.x, 50.0, 130.0), 52.0};
      }
      drive_target(robot, target, 3);
    }
  } else {
    spdlog::info("CONTR");
    optional<Vec> nearest_obstacle;
    if (robot.lidar) {
      for (const auto& obstacle : robot.lidar->obstacles_data) {
        if (!nearest_obstacle.has_value() ||
            (obstacle - robot.position).len() <
                (*nearest_obstacle - robot.position).len()) {
          nearest_obstacle = obstacle;
        }
      }
    }
    Vec target;
    if (!nearest_obstacle.has_value() ||
        (*nearest_obstacle - robot.position).len() > 70 ||
        nearest_obstacle->y < 30) {
      target = {91.0, 35.0};
    } else {
      target = {nearest_obstacle->x, 35.0};
    }
    drive_target(robot, target, 3);
  }
}
