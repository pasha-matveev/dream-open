#include "config/strategy/keeper.h"

#include <spdlog/spdlog.h>

#include <algorithm>
#include <limits>
#include <optional>

#include "config/config.h"
#include "config/lidar.h"
#include "config/strategy.h"
#include "strategy/ball_tracker.h"
#include "strategy/dubins.h"
#include "strategy/field.h"
#include "strategy/kick.h"
#include "strategy/motion.h"
#include "strategy/strategy.h"
#include "strategy/zones.h"
#include "utils/mapper.h"
#include "utils/millis.h"

using namespace std;

static optional<Vec> nearest_obstacle(Robot& robot) {
  if (!robot.lidar) return nullopt;
  static Polygon full_markup = make_full_field_markup();

  const auto& piter_cfg = *config->lidar->piter;

  optional<Vec> best = nullopt;
  double best_dist = std::numeric_limits<double>::infinity();
  for (const auto& obs : robot.lidar->obstacles_data) {
    double self_dist = (obs - robot.position).len();
    if (self_dist < piter_cfg.self_exclusion_radius) continue;
    if (!full_markup.inside(obs) &&
        full_markup.dist(obs) > piter_cfg.markup_tolerance) {
      continue;
    }
    if (self_dist < best_dist) {
      best_dist = self_dist;
      best = obs;
    }
  }
  return best;
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

// Состояние тарана
static bool ram_prev_visible = false;
static bool ram_active = false;
static bool was_ram = false;

// Ricochet-режим: когда вратарь за проекцией своих ворот, бить рикошетом
// от ближней стенки вместо поворота в сторону чужих ворот.
// Сбрасывается при падении emitter
enum class RicochetMode { NONE, LEFT, RIGHT };
static RicochetMode ricochet_mode = RicochetMode::NONE;
static double ricochet_target_field_angle = 0.0;
static bool ricochet_target_computed = false;

void Strategy::run_keeper(Robot& robot, Object& ball, Object& goal,
                          Field& field) {
  auto obstacle = nearest_obstacle(robot);

  // nearest_obstacle уже фильтрует по полной разметке поля и зоне
  // самоисключения — здесь достаточно факта наличия валидной точки.
  if (obstacle.has_value()) {
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

  bool cur_visible =
      ball_->recently_visible(millis(), keeper_cfg.ram->blind_min_ms);
  bool fresh = cur_visible && !ram_prev_visible;
  ram_prev_visible = cur_visible;
  ram_active = false;

  // Падающий фронт emitter → сброс ricochet-состояния. На следующем
  // захвате режим решается заново.
  if (!robot.emitter) {
    ricochet_mode = RicochetMode::NONE;
    ricochet_target_computed = false;
  }

  if (!is_piter) {
    if (robot.emitter) {
      // Мяч в лунке
      if (stabilize_capture(robot,
                            {.dribbling = *config->strategy->dribbling})) {
        // Стабилизация только что захваченного мяча.
      } else {
        // Гистерезис по robot.position.x: левее x_padding → LEFT;
        // правее FIELD_WIDTH − x_padding → RIGHT. Выход из режима
        // только когда x ушёл от порога вглубь на hysteresis.
        const auto& rc_cfg = *keeper_cfg.ricochet;
        double xl = rc_cfg.x_padding;
        double xr = FIELD_WIDTH - rc_cfg.x_padding;
        double rx = robot.position.x;
        switch (ricochet_mode) {
          case RicochetMode::NONE:
            if (rx < xl) {
              ricochet_mode = RicochetMode::LEFT;
              ricochet_target_computed = false;
            } else if (rx > xr) {
              ricochet_mode = RicochetMode::RIGHT;
              ricochet_target_computed = false;
            }
            break;
          case RicochetMode::LEFT:
            if (rx > xl + rc_cfg.hysteresis) {
              ricochet_mode = RicochetMode::NONE;
              ricochet_target_computed = false;
            }
            break;
          case RicochetMode::RIGHT:
            if (rx < xr - rc_cfg.hysteresis) {
              ricochet_mode = RicochetMode::NONE;
              ricochet_target_computed = false;
            }
            break;
        }

        if (ricochet_mode != RicochetMode::NONE) {
          bool left = (ricochet_mode == RicochetMode::LEFT);
          const auto& t = left ? rc_cfg.target_left : rc_cfg.target_right;
          double field_angle;
          if (rc_cfg.recompute_angle_each_tick) {
            field_angle = compute_ricochet_field_angle(
                robot.ball_hole_position(), {t.x, t.y}, left);
          } else {
            if (!ricochet_target_computed) {
              ricochet_target_field_angle = compute_ricochet_field_angle(
                  robot.ball_hole_position(), {t.x, t.y}, left);
              ricochet_target_computed = true;
            }
            field_angle = ricochet_target_field_angle;
          }
          double relative_dir =
              normalize_angle(field_angle - robot.field_angle);
          kick_->execute(robot, {.relative_dir = relative_dir});
        } else {
          // Прямой удар по чужим воротам.
          kick_->execute_to_goal(robot, goal, {});
        }
      }
    } else if (ball_ok) {
      Vec ball_pos = ball_->position();

      // Мяч в дополнительной зоне ответственности (полоса вдоль задней
      // линии под keeper-зоной): робот туда не заедет — там либо ворота,
      // либо угловой карман за стойкой, но field.apply() прижмёт его к
      // границе зоны, и у нас есть шанс достать. По x ограничиваем
      // диапазоном, чтобы не пытаться ехать за мячом, улетевшим в дальние
      // углы поля.
      const auto& ar_cfg = *keeper_cfg.additional_responsibility;
      bool additional_keeper_responsibility =
          ball_pos.x >= ar_cfg.x_padding &&
          ball_pos.x <= FIELD_WIDTH - ar_cfg.x_padding &&
          ball_pos.y < ar_cfg.y_max;

      if (field.inside(ball_pos) || additional_keeper_responsibility) {
        // Мяч внутри keeper-зоны либо в её дополнительной зоне
        // ответственности — забираем.
        const auto& ram_cfg = *keeper_cfg.ram;
        double dist = (ball_pos - robot.position).len();
        bool safety_ok = (ball_pos.y - robot.position.y) >= ram_cfg.safety_y;
        bool far = dist >= ram_cfg.far_min_dist;
        // Гистерезис: пока мяч в зоне ответственности и safety не нарушен,
        // продолжаем ram, начатый в предыдущем тике. Если мяч выйдет — мы
        // окажемся в другой ветке (projection/ray) и ram сам сбросится.
        if (ram_cfg.enabled && safety_ok && (fresh || far || was_ram)) {
          // RAM: едем на мяч на физически безопасной скорости, минуя мапер.
          drive_target(robot, ball_pos, ram_cfg.max_speed, ram_cfg.min_speed,
                       /*is_ball=*/false);
          robot.dribbling = config->strategy->dribbling->value_l;
          robot.rotation_limit = 40;
          ram_active = true;
        } else {
          drive_ball(robot, ball_pos);
        }
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
  was_ram = ram_active;
}
