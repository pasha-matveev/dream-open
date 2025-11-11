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

static Vec compute_point(const Vec& p, const Field& field) {
  Vec fallback = {clamp(p.x, 30.0, 150.0), 35.0};
  Vec center{91.0, 0.0};
  Vec dir = p - center;
  dir = dir.resize(1000000);
  auto [np, idx] = field.find_intersection(center, dir);
  if (idx == -1) {
    spdlog::error("Cannot find intersection. P({}, {})", p.x, p.y);
    return fallback;
  }
  double desired_len = (np - center).len() + 2;
  Vec res = center + dir.resize(desired_len);
  return res;
}

static long long last_piter_visible = -10000;
static Vec last_piter;

void Strategy::run_keeper(Robot& robot, Object& ball, Object& goal,
                          const Field& field) {
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
  if (ball.visible || robot.emitter) {
    is_piter = false;
  } else if (!piter_ok && ball_ok) {
    is_piter = false;
  } else {
    is_piter = false;
  }
  if (!is_piter) {
    if (ball_ok && last_ball.y < 90) {
      active_def = true;
    } else if (last_ball.y > 95) {
      active_def = false;
    }

    if (robot.emitter) {
      active_def = true;
      last_ball = robot.ball_hole_position();
    }

    if (active_def) {
      if (robot.emitter) {
        spdlog::info("HIT");
        hit(robot, goal, 100);
      } else {
        spdlog::info("DRIVE");
        drive_ball(robot, last_ball);
      }
    } else {
      spdlog::info("PASSIVE");
      Vec target;
      if (ball_ok) {
        target = compute_point(last_ball, field);
      } else {
        target = {91.0, 52.0};
      }
      drive_target(robot, target, 3);
      if (ball_ok) {
        robot.rotation =
            (last_ball - robot.position).field_angle() - robot.field_angle;
      } else {
        robot.rotation = -robot.field_angle;
      }
    }
  } else {
    spdlog::info("CONTR");
    Vec target;
    if (piter_ok) {
      target = compute_point(last_piter, field);
    } else {
      target = {91.0, 52.0};
    }
    drive_target(robot, target, 3);
    robot.rotation = -robot.field_angle;
  }
}
