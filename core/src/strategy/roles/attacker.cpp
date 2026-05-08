#include <spdlog/spdlog.h>

#include "config/config.h"
#include "config/strategy.h"
#include "strategy/ball_tracker.h"
#include "strategy/dubins.h"
#include "strategy/field.h"
#include "strategy/kick.h"
#include "strategy/motion.h"
#include "strategy/strategy.h"
#include "utils/geo/polygon.h"
#include "utils/mapper.h"
#include "utils/millis.h"
#include "utils/switch.h"

// enum class AttackerRSide { NONE, LEFT, RIGHT };
// enum class AttackerRStatus { NONE, TAKE_BALL, ROTATE_1, MOVE, ROTATE_2, KICK
// };

// static AttackerRSide r_side = AttackerRSide::NONE;
// static AttackerRStatus r_status = AttackerRStatus::NONE;

#define SPECIAL_HEIGHT config->strategy->attacker->special_height

static Polygon create_left_polygon() {
  return {{{0.0, 240.0 - SPECIAL_HEIGHT}, {0.0, 240.0}, ENEMY_GOAL_CENTER}};
}

static Polygon create_right_polygon() {
  return {{{182.0, 240.0 - SPECIAL_HEIGHT}, ENEMY_GOAL_CENTER, {182.0, 240.0}}};
}

static double weighted_dist(const Polygon& pol, const Vec& pos) {
  double ans = pol.dist(pos);
  if (!pol.inside(pos)) {
    ans *= -1;
  }
  return ans;
}

static Switch polygon_oscil{0, 2};

void Strategy::run_attacker(Robot& robot, Object& ball, Object& goal,
                            Field& field) {
  Polygon left_polygon = create_left_polygon();
  Polygon right_polygon = create_right_polygon();

  double left_polygon_dist = weighted_dist(left_polygon, robot.position);
  double right_polygon_dist = weighted_dist(right_polygon, robot.position);
  double polygon_dist = max(left_polygon_dist, right_polygon_dist);
  bool use_left_special = left_polygon_dist > right_polygon_dist;

  bool outside_special = polygon_oscil.compute(polygon_dist);
  bool inside_special = !outside_special;

  // ASSERTION
  if (!left_polygon.inside(robot.position) &&
      !right_polygon.inside(robot.position)) {
    assert(polygon_dist <= 0);
  }
  if (left_polygon.inside(robot.position) ||
      right_polygon.inside(robot.position)) {
    assert(polygon_dist >= 0);
  }

  Vec special_pos_left = {config->strategy->attacker->special_pos->x,
                          240.0 - config->strategy->attacker->special_pos->y};
  Vec special_pos_right = {182.0 - config->strategy->attacker->special_pos->x,
                           240.0 - config->strategy->attacker->special_pos->y};

  if (robot.emitter) {
    // Взяли мяч
    if (dubins_->was_active_last_tick()) {
      spdlog::info("KICK DUBINS");
      // Подъехали по окружности, используем удар оттуда
      dubins_->dubins_hit(robot, goal, field,
                          KickController::compute_power(robot.position), false);
    } else {
      if (inside_special) {
        spdlog::info("SPECIAL");
        Vec target = use_left_special ? special_pos_left : special_pos_right;
        robot.dribbling = config->strategy->dribbling->value_r;
        drive_target(robot, target, 30);
        robot.rotation = 0;
      } else {
        spdlog::info("KICK GOAL");
        kick_->execute_to_goal(robot, goal, {});
      }

      // Vec hole_position = robot.ball_hole_position();
      // if (!robot.prev_emitter) {
      //   // Только что взяли мяч
      //   // Проверяем, не нужно ли отъехать
      //   if (left_attacker_r.inside(hole_position)) {
      //     r_side = AttackerRSide::LEFT;
      //     r_status = AttackerRStatus::TAKE_BALL;
      //   } else if (right_attacker_r.inside(hole_position)) {
      //     r_side = AttackerRSide::RIGHT;
      //     r_status = AttackerRStatus::TAKE_BALL;
      //   }
      // }
      // bool r_left = r_side == AttackerRSide::LEFT;
      // if (r_status == AttackerRStatus::NONE) {
      //   // Поворачиваемся и бьем
      //   spdlog::info("SIMPLE");
      //   kick_to_goal(robot, goal, {});
      //   // attacker_simple(robot, goal);
      // } else if (r_status == AttackerRStatus::TAKE_BALL) {
      //   // Особая стратегия: нужно взять мяч
      //   spdlog::info("TAKE_BALL");
      //   bool finished = take_ball(robot, 600);
      //   if (finished) {
      //     r_status = AttackerRStatus::ROTATE_1;
      //   }
      // }
      // if (r_status == AttackerRStatus::ROTATE_1) {
      //   // Особая стратегия: поворот
      //   spdlog::info("ROTATE_1");
      //   robot.dribbling = config.strategy.max_dribbling;
      //   double target_angle;
      //   if (r_left) {
      //     target_angle = M_PI / 2;
      //   } else {
      //     target_angle = -M_PI / 2;
      //   }
      //   bool finished = turn(robot, target_angle, true);
      //   if (finished) {
      //     r_status = AttackerRStatus::MOVE;
      //   }
      // } else if (r_status == AttackerRStatus::MOVE) {
      //   // Особая стратегия: движение
      //   spdlog::info("MOVE");
      //   Vec target;
      //   double target_angle;
      //   if (r_left) {
      //     target = Vec{30.0, 190.0};
      //     target_angle = M_PI / 2;
      //   } else {
      //     target = Vec{182.0 - 30.0, 190.0};
      //     target_angle = -M_PI / 2;
      //   }
      //   bool finished = drive_target(robot, target, 2, 20, 5);
      //   robot.rotation = normalize_angle(target_angle - robot.field_angle);
      //   robot.dribbling = config.strategy.max_dribbling;
      //   if (finished) {
      //     r_status = AttackerRStatus::ROTATE_2;
      //   }
      // } else if (r_status == AttackerRStatus::ROTATE_2) {
      //   // Особая стратегия: поворот и удар
      //   spdlog::info("ROTATE 2");
      //   if (config->strategy->attacker_ricochet->enabled) {
      //     double alpha;
      //     alpha = compute_ricochet(robot, r_left);
      //     kick_dir(robot, alpha, 100, 0, true, 0, 0.01);
      //   } else {
      //     attacker_simple(robot, goal);
      //   }
      // }
    }
  } else {
    // Не взяли мяч
    // r_status = AttackerRStatus::NONE;
    // r_side = AttackerRSide::NONE;
    bool recently_visible = ball_->recently_visible(millis(), 3000);
    Vec ball_pos = ball_->position();
    if (!recently_visible || ball_pos.y < config->strategy->attacker->border) {
      // Мяч у вратаря или мы его не видим
      Vec target;
      if (recently_visible && ball_pos.x < 91) {
        // Едем к левому краю
        target = {20, 120};
      } else {
        // Едем к правому краю
        target = {160, 120};
      }
      spdlog::info("DRIVE PASSIVE");
      drive_target(robot, target);
      robot.rotation = ball_->relative_angle(robot);
    } else {
      // Мяч на нашей половине

      // if (drive_state == DriveState::NONE) {
      //   if (field.inside(last_ball_position) &&
      //       field.dist(last_ball_position) >=
      //           config.strategy.dubins.border_dist_2) {
      //     drive_state = DriveState::DUBINS;
      //   } else {
      //     drive_state = DriveState::EDGE;
      //   }
      // } else if (drive_state == DriveState::DUBINS &&
      //            (!field.inside(last_ball_position) ||
      //             field.dist(last_ball_position) <
      //                 config.strategy.dubins.border_dist_1)) {
      //   drive_state = DriveState::EDGE;
      // } else if (drive_state == DriveState::EDGE &&
      //            field.inside(last_ball_position) &&
      //            field.dist(last_ball_position) >
      //                config.strategy.dubins.border_dist_2) {
      //   drive_state = DriveState::DUBINS;
      // }

      // Мяч далеко от бортов, используем dubins path

      bool res = false;
      if (config->strategy->attacker->dubins_enabled) {
        res = dubins_->dubins_hit(robot, goal, field,
                                  KickController::compute_power(robot.position),
                                  false);
      }
      if (!res) {
        // Мяч близко к бортам, играем как обычно
        drive_ball(robot, ball_pos);
        // Компенсация скорости мяча: если мяч катится, добавляем его
        // скорость к команде, чтобы относительная скорость сближения
        // соответствовала тому, что drive_target насчитал для статичной цели.
        const auto& ff = *config->strategy->attacker->ff;
        if (ff.enabled && ball_->recently_visible(millis(), ff.stale_ms) &&
            ball_->velocity().len() >= ff.v_min) {
          robot.vel += ball_->velocity() * ff.gain;
        }
      }
    }
  }

  robot.vel = robot.vel.resize(min(robot.vel.len(), 120.0));
}