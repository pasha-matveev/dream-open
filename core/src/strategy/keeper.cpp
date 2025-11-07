#include <spdlog/spdlog.h>

#include <algorithm>
#include <cmath>
#include <optional>

#include "strategy/strategy.h"
#include "utils/millis.h"

using namespace std;

static optional<Vec> nearest_obstacle(Robot& robot) {
  optional<Vec> nearest_obstacle;
  if (robot.lidar) {
    for (const auto& obstacle : robot.lidar->obstacles_data) {
      if (obstacle.y < 10 || obstacle.y > 80) {
        continue;
      }
      if (!nearest_obstacle.has_value() || obstacle.y < nearest_obstacle->y) {
        nearest_obstacle = obstacle;
      }
    }
  }
  return nearest_obstacle;
}

static long long last_piter_visible = LONG_LONG_MIN;
static Vec last_piter;

void Strategy::run_keeper(Robot& robot, Object& ball, Object& goal) {
  auto obstacle = nearest_obstacle(robot);

  if (obstacle.has_value()) {
    last_piter_visible = millis();
    last_piter = *obstacle;
  }

  long long ball_visible_tm = millis() - last_ball_visible;
  long long piter_visible_tm = millis() - last_piter_visible;
  bool ball_ok = ball_visible_tm <= 2000;
  bool piter_ok = piter_visible_tm <= 2000;

  bool is_piter;
  if (ball.visible) {
    is_piter = false;
  } else if (!piter_ok && ball_ok) {
    is_piter = false;
  } else {
    is_piter = true;
  }
  if (!is_piter) {
    if (last_ball.y < 90) {
      active_def = true;
    } else if (last_ball.y > 95) {
      active_def = false;
    }

    if (robot.emitter) {
      active_def = true;
    }

    if (active_def) {
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
      robot.rotation =
          (last_ball - robot.position).field_angle() - robot.field_angle;
    }
  } else {
    spdlog::info("CONTR");
    Vec target;
    if (piter_ok) {
      target = {last_piter.x, 35.0};
    } else {
      target = {91.0, 52.0};
    }
    drive_target(robot, target, 3);
    robot.rotation = -robot.field_angle;
  }
}
