#include <spdlog/spdlog.h>

#include <algorithm>
#include <cmath>
#include <optional>

#include "strategy/strategy.h"
#include "strategy/visualization.h"
#include "utils/config.h"
#include "utils/millis.h"

using namespace std;

static optional<Vec> nearest_obstacle(Robot& robot) {
  optional<Vec> nearest_obstacle = nullopt;
  if (robot.lidar) {
    for (const auto& obstacle : robot.lidar->obstacles_data) {
      if (obstacle.y < 13 || obstacle.y > 80) {
        continue;
      }
      if (obstacle.x < 3 || obstacle.x > 182 - 3) {
        continue;
      }
      if (!nearest_obstacle.has_value() || obstacle.y < nearest_obstacle->y) {
        nearest_obstacle = obstacle;
      }
    }
  }
  return nearest_obstacle;
}

// Как защищать ворота от Питера
static Vec compute_radial(const Vec& p, const Field& field) {
  // Хотим стоять на границе поля
  Vec center{91.0, 0.0};  // центр псевдоокружности (за границами поля)
  Vec dir = p - center;
  dir = dir.resize(1000000);
  // ищем пересечение луча и нижней границы поля
  auto [np, idx] = field.find_intersection(center, dir);
  if (idx == -1) {
    double xl = config.strategy.keeper.line.padding;
    double xr = 180 - config.strategy.keeper.line.padding;
    double y = config.strategy.keeper.line.y;

    Vec fallback = {clamp(p.x, xl, xr), y};
    // что-то странное, вместо окружности просто стоим на прямой
    return fallback;
  }
  double desired_len = (np - center).len() + 2;
  Vec res = center + dir.resize(desired_len);
  return res;
}

// Как защищать ворота от мяча
static Vec compute_contr_point(const Vec& p, const Field& field) {
  if (p.y > 35) {
    // Точка сверху, стоим на проекции
    double xl = config.strategy.keeper.line.padding;
    double xr = 180 - config.strategy.keeper.line.padding;
    double y = config.strategy.keeper.line.y;

    double x = clamp(p.x, xl, xr);
    return {x, y};
  } else {
    // Точка внизу, стоим на окружности
    return compute_radial(p, field);
  }
}

static long long last_piter_visible = -10000;
static Vec last_piter;

void Strategy::run_keeper(Robot& robot, Object& ball, Object& goal,
                          Field& field) {
  auto obstacle = nearest_obstacle(robot);

  if (obstacle.has_value() && (*obstacle - robot.position).len() >= 11 &&
      (field.inside(*obstacle) || field.dist(*obstacle) <= 6)) {
    last_piter_visible = millis();
    last_piter = *obstacle;
    Vec dir = last_piter - robot.position;
    spdlog::error("Piter {} {}", last_piter.x, last_piter.y);
    spdlog::error("Dir {} {}, {}", dir.x, dir.y, dir.len());
  }

  long long ball_visible_tm = millis() - last_ball_visible;
  long long piter_visible_tm = millis() - last_piter_visible;
  bool ball_ok = ball_visible_tm <= 2000;
  // bool piter_ok = piter_visible_tm <= 2000;
  bool piter_ok = false;

  bool is_piter;
  if (ball.visible || robot.emitter) {
    is_piter = false;
  } else if (!piter_ok && ball_ok) {
    is_piter = false;
  } else {
    is_piter = piter_ok;
  }

  // if (piter_ok) {
  //   if (!sfml_window || !sfml_window->isOpen()) {
  //     return;
  //   }

  //   auto sfml_r = cm_to_px(9);
  //   auto shape = sf::CircleShape(sfml_r);
  //   shape.setPosition(toSFML(last_piter) - Vec{sfml_r, sfml_r});
  //   sf::Color color = sf::Color(255, 255, 255);
  //   shape.setOutlineColor(color);
  //   shape.setOutlineThickness(2);
  //   shape.setFillColor(sf::Color::Transparent);
  //   sfml_window->draw(shape);
  // }

  robot.dribling = config.strategy.dribbling.value_l;
  if (!is_piter) {
    if (robot.emitter) {
      // Мяч в лунке
      if (last_dubins) {
        // Подъехали по dubins, продолжаем использовать эту стратегию
        spdlog::info("DUBINS KICK");
        dubins_hit(robot, goal, field, 100, false);
      } else {
        // Просто целимся и стреляем
        spdlog::info("SIMPLE KICK");
        kick_to_goal(robot, goal, {});
        // hit(robot, goal, 100, 200, true, 0, 0.02, true);
      }
    } else {
      if (ball_ok) {
        // Видим мяч
        if (last_ball_position.y >= config.strategy.keeper.global_border) {
          // Мяч за нашей зоной, защищаем ворота
          Vec target = compute_contr_point(last_ball_position, field);
          spdlog::info("LONG PROTECT {} {}", target.x, target.y);
          drive_target(robot, target, 20, 100);
          robot.rotation = last_ball_relative_angle;
        } else if (last_ball_position.y >=
                       config.strategy.keeper.dubins_border &&
                   dubins_hit(robot, goal, field, 100, false)) {
          // Мяч в зоне удара, используем dubins_path
          spdlog::info("DUBINS PROTECT");
        } else if (last_ball_position.y > robot.position.y) {
          // Просто бьем мяч корпусом
          spdlog::info("RAM");
          drive_target(robot, last_ball_position, 3, 120, 50);
          robot.rotation = -robot.field_angle;
        } else {
          // Мяч близко, _аккуратно_ его берем
          spdlog::info("NEAR PROTECT {} {} {}", ball_visible_tm,
                       last_ball_position.x, last_ball_position.y);
          drive_ball(robot, last_ball_position);
        }
      } else {
        // Не видим мяч
        // spdlog::info("IDLE");
        Vec target{91.0, config.strategy.keeper.line.y};
        drive_target(robot, target, 2);
        robot.rotation = -robot.field_angle;
      }
    }
  } else {
    // Контрим питер
    spdlog::info("CONTR");
    Vec target;
    target = compute_radial(last_piter, field);
    drive_target(robot, target, 4);
    robot.rotation = normalize_angle(
        (last_piter - robot.position).field_angle() - robot.field_angle);
  }
}
