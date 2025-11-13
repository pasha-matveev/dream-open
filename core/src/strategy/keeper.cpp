#include <spdlog/spdlog.h>

#include <algorithm>
#include <cmath>
#include <optional>

#include "strategy/strategy.h"
#include "utils/config.h"
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

static Vec compute_radial(const Vec& p, const Field& field) {
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

static Vec compute_contr_point(const Vec& p, const Field& field) {
  if (p.y > 35) {
    // Точка сверху, стоим на проекции
    double x = clamp<double>(p.x, 12 + 39, 182 - 12 - 39);
    return {x, 35.0};
  } else {
    // Точка внизу, стоим на окружности
    return compute_radial(p, field);
  }
}

static long long last_piter_visible = -10000;
static Vec last_piter;
static bool last_dubins = false;
static const int dubins_y = 12 + 28;

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
    is_piter = piter_ok;
  }

  bool cur_dubins = false;
  if (!is_piter) {
    if (robot.emitter) {
      // Мяч в лунке
      if (last_dubins) {
        // Подъехали по dubins, продолжаем использовать эту стратегию
        cur_dubins = true;
        dubins_hit(robot, goal, 100, config.strategy.dubins.keeper_control);
      } else {
        // Просто целимся и стреляем
        hit(robot, goal, 100, 200, true, 0, 0.02, true);
      }
    } else {
      if (ball_ok) {
        // Видим мяч
        if (!field.inside(last_ball)) {
          // Мяч за нашей зоной, защищаем ворота
          Vec target = compute_contr_point(last_ball, field);
          drive_target(robot, target, 3);
          robot.rotation = last_ball_relative;
        } else if (last_ball.y >= dubins_y) {
          // Мяч в зоне удара, используем dubins_path
          cur_dubins = true;
          dubins_hit(robot, goal, 100, config.strategy.dubins.keeper_control);
        } else {
          // Мяч близко, бьем как обычно
          drive_ball(robot, last_ball);
        }
      } else {
        // Не видим мяч
        Vec target{91, 35};
        drive_target(robot, target, 2);
        robot.rotation = -robot.field_angle;
      }
    }
  } else {
    // Контрим питер
    spdlog::info("CONTR");
    Vec target;
    target = compute_radial(last_piter, field);
    drive_target(robot, target, 3);
    robot.rotation =
        (last_piter - robot.position).field_angle() - robot.field_angle;
  }
  last_dubins = cur_dubins;
}
