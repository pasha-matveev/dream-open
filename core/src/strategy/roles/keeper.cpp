#include "config/strategy/keeper.h"

#include <spdlog/spdlog.h>

#include <algorithm>
#include <optional>

#include "config/config.h"
#include "config/strategy.h"
#include "strategy/ball_tracker.h"
#include "strategy/dubins.h"
#include "strategy/field.h"
#include "strategy/kick.h"
#include "strategy/motion.h"
#include "strategy/strategy.h"
#include "strategy/zones.h"
#include "utils/millis.h"

using namespace std;

static optional<Vec> nearest_obstacle(Robot& robot) {
  optional<Vec> nearest_obstacle = nullopt;
  if (robot.lidar) {
    for (const auto& obstacle : robot.lidar->obstacles_data) {
      if (obstacle.y < 13 || obstacle.y > 80) {
        continue;
      }
      if (obstacle.x < 3 || obstacle.x > FIELD_WIDTH - 3) {
        continue;
      }
      if (!nearest_obstacle.has_value() || obstacle.y < nearest_obstacle->y) {
        nearest_obstacle = obstacle;
      }
    }
  }
  return nearest_obstacle;
}

// Точка на луче из {91,0} через target, ВНУТРИ keeper field (на 2 см глубже
// границы пересечения), с клампом по минимальному Y.
//
// Используется и для защиты от мяча (ray_min_y из конфига), и для защиты от
// вражеского робота — is_piter (ray_min_y = line.y, поведение как раньше).
//
// При hit.y < ray_min_y встаём в крыло возле стойки (x=51 или x=131) на
// высоте ray_min_y, чтобы не пытаться стоять на нижнем горизонтальном
// отрезке зоны (тот закрывает не ворота, а боковую полку).
static Vec compute_ray_target(const Vec& target, const Field& field,
                              double ray_min_y) {
  Vec center{91.0, 0.0};
  Vec dir = (target - center).resize(1'000'000);

  auto [hit, idx] = field.find_intersection(center, dir);

  if (idx == -1) {
    // Дегенеративный случай: луч не пересёк границу. Wing-fallback.
    return {(target.x < 91.0) ? 51.0 : FIELD_WIDTH - 51.0, ray_min_y};
  }

  if (hit.y < ray_min_y) {
    // Точка пересечения слишком низко — стоим в крыле возле стойки.
    return {(target.x < 91.0) ? 51.0 : FIELD_WIDTH - 51.0, ray_min_y};
  }

  double desired_len = (hit - center).len() + 2.0;
  return center + dir.resize(desired_len);
}

static long long last_piter_visible = -10000;
static Vec last_piter;

void Strategy::run_keeper(Robot& robot, Object& ball, Object& goal,
                          Field& field) {
  auto obstacle = nearest_obstacle(robot);

  // Если препятствие действительно можно считать роботом
  if (obstacle.has_value() && (*obstacle - robot.position).len() >= 11 &&
      (field.inside(*obstacle) || field.dist(*obstacle) <= 6)) {
    last_piter_visible = millis();
    last_piter = *obstacle;
    spdlog::warn("Piter: {} {}", last_piter.x, last_piter.y);
    Vec dir = last_piter - robot.position;
    spdlog::warn("Dir: {} {}", dir.x, dir.y);
  }

  long long piter_visible_tm = millis() - last_piter_visible;
  bool ball_ok = ball_->recently_visible(millis(), 3000);
  bool piter_ok = piter_visible_tm <= 2000;

  bool is_piter;
  if (ball.visible || robot.emitter) {
    is_piter = false;
  } else if (!piter_ok && ball_ok) {
    is_piter = false;
  } else {
    is_piter = piter_ok;
  }

  const auto& keeper_cfg = *config->strategy->keeper;

  if (!is_piter) {
    if (robot.emitter) {
      // Мяч в лунке
      if (stabilize_capture(robot,
                            {.dribbling = *config->strategy->dribbling})) {
        // Стабилизация только что захваченного мяча.
      } else {
        // Просто целимся и стреляем
        kick_->execute_to_goal(robot, goal, {});
      }
    } else if (ball_ok) {
      Vec ball_pos = ball_->position();

      // Мяч в дополнительной зоне ответственности (полоса вдоль задней
      // линии под keeper-зоной): робот туда не заедет — там либо ворота,
      // либо угловой карман за стойкой, но field.apply() прижмёт его к
      // границе зоны, и у нас есть шанс достать. По x ограничиваем
      // диапазоном keeper-зоны, чтобы не пытаться ехать за мячом,
      // улетевшим в дальние углы поля.
      bool additional_keeper_responsibility =
          ball_pos.x >= KEEPER_ZONE_SIDE_DIST &&
          ball_pos.x <= FIELD_WIDTH - KEEPER_ZONE_SIDE_DIST &&
          ball_pos.y < KEEPER_ADDITIONAL_RESPONSIBILITY_Y_MAX;

      if (field.inside(ball_pos) || additional_keeper_responsibility) {
        // Мяч внутри keeper-зоны либо в её дополнительной зоне
        // ответственности — забираем.
        drive_ball(robot, ball_pos);
      } else if (ball_pos.y >= keeper_cfg.projection_border) {
        // Мяч далеко — стоим на проекции на горизонтальную линию.
        double xl = keeper_cfg.line->padding;
        double xr = FIELD_WIDTH - keeper_cfg.line->padding;
        Vec target{clamp(ball_pos.x, xl, xr), keeper_cfg.line->y};
        drive_target(robot, target);
        robot.rotation = ball_->relative_angle(robot);
      } else {
        // Мяч между линией вратаря и projection_border — лучевая защита.
        Vec target =
            compute_ray_target(ball_pos, field, keeper_cfg.line->ray_min_y);
        drive_target(robot, target);
        robot.rotation = ball_->relative_angle(robot);
      }
    } else {
      // Не видим мяч — IDLE.
      Vec target{91.0, keeper_cfg.line->y};
      drive_target(robot, target);
      robot.rotation = -robot.field_angle;
    }
  } else {
    // Контрим Питер. ray_min_y = line.y воспроизводит прежнее поведение
    // (без жёсткого клампа по минимальному Y на крыле).
    spdlog::info("CONTR");
    Vec target =
        compute_ray_target(last_piter, field, keeper_cfg.line->ray_min_y);
    drive_target(robot, target);
    robot.rotation = normalize_angle(
        (last_piter - robot.position).field_angle() - robot.field_angle);
  }
}
