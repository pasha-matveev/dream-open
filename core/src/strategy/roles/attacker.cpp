#include <spdlog/spdlog.h>

#include "strategy/ball_tracker.h"
#include "strategy/dubins.h"
#include "strategy/kick.h"
#include "strategy/motion.h"
#include "strategy/strategy.h"
#include "utils/config.h"
#include "utils/geo/polygon.h"
#include "utils/millis.h"

// enum class AttackerRSide { NONE, LEFT, RIGHT };
// enum class AttackerRStatus { NONE, TAKE_BALL, ROTATE_1, MOVE, ROTATE_2, KICK
// };

// static AttackerRSide r_side = AttackerRSide::NONE;
// static AttackerRStatus r_status = AttackerRStatus::NONE;

void Strategy::run_attacker(Robot& robot, Object& ball, Object& goal,
                            Field& field) {
  if (robot.emitter) {
    // Взяли мяч
    if (dubins_->was_active_last_tick()) {
      spdlog::info("KICK DUBINS");
      // Подъехали по окружности, используем удар оттуда
      dubins_->dubins_hit(robot, goal, field,
                          KickController::compute_power(robot.position.y),
                          false);
    } else {
      spdlog::info("KICK GOAL");
      kick_->execute_to_goal(robot, goal, {});
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
      //   robot.dribling = config.strategy.max_dribling;
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
      //   robot.dribling = config.strategy.max_dribling;
      //   if (finished) {
      //     r_status = AttackerRStatus::ROTATE_2;
      //   }
      // } else if (r_status == AttackerRStatus::ROTATE_2) {
      //   // Особая стратегия: поворот и удар
      //   spdlog::info("ROTATE 2");
      //   if (config.strategy.attacker_ricochet.enabled) {
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
    if (!recently_visible || ball_pos.y < config.strategy.attacker.border) {
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
      drive_target(robot, target, 4);
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
      if (config.strategy.attacker.dubins_enabled) {
        res = dubins_->dubins_hit(
            robot, goal, field, KickController::compute_power(robot.position.y),
            false);
      }
      if (!res) {
        // Мяч близко к бортам, играем как обычно
        drive_ball(robot, ball_pos);
      }
    }
  }

  robot.vel = robot.vel.resize(min(robot.vel.len(), 120.0));
}