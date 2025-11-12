#include <spdlog/spdlog.h>

#include "strategy/strategy.h"
#include "utils/config.h"
#include "utils/geo/polygon.h"
#include "utils/millis.h"

enum class AttackerRSide { NONE, LEFT, RIGHT };
enum class AttackerRStatus { NONE, TAKE_BALL, ROTATE_1, MOVE, ROTATE_2, KICK };

static AttackerRSide r_side = AttackerRSide::NONE;
static AttackerRStatus r_status = AttackerRStatus::NONE;

void Strategy::attacker_simple(Robot& robot, Object& goal) {
  int power = 70;
  if (robot.position.y < 146) {
    power = 40;
  } else if (robot.position.y < 180) {
    power = 20;
  } else {
    power = 15;
  }
  hit(robot, goal, power, 800);
}

void Strategy::run_attacker(Robot& robot, Object& ball, Object& goal) {
  const int BORDER = 80;

  if (robot.emitter) {
    Vec hole_position = robot.ball_hole_position();
    if (!robot.prev_emitter) {
      // только что взяли мяч
      if (left_attacker_r.inside(hole_position)) {
        r_side = AttackerRSide::LEFT;
        r_status = AttackerRStatus::TAKE_BALL;
      } else if (right_attacker_r.inside(hole_position)) {
        r_side = AttackerRSide::RIGHT;
        r_status = AttackerRStatus::TAKE_BALL;
      }
    }
    bool r_left = r_side == AttackerRSide::LEFT;
    if (r_status == AttackerRStatus::NONE) {
      spdlog::info("SIMPLE");
      attacker_simple(robot, goal);
    } else if (r_status == AttackerRStatus::TAKE_BALL) {
      spdlog::info("TAKE_BALL");
      bool finished = take_ball(robot, 600);
      if (finished) {
        r_status = AttackerRStatus::ROTATE_1;
      }
    } else if (r_status == AttackerRStatus::ROTATE_1) {
      spdlog::info("ROTATE_1");
      robot.dribling = config.strategy.max_dribling;
      double target_angle;
      if (r_left) {
        target_angle = M_PI / 2;
      } else {
        target_angle = -M_PI / 2;
      }
      bool finished = turn(robot, target_angle, true);
      if (finished) {
        r_status = AttackerRStatus::MOVE;
      }
    } else if (r_status == AttackerRStatus::MOVE) {
      spdlog::info("MOVE");
      Vec target;
      double target_angle;
      if (r_left) {
        target = Vec{30.0, 190.0};
        target_angle = M_PI / 2;
      } else {
        target = Vec{182.0 - 30.0, 190.0};
        target_angle = -M_PI / 2;
      }
      bool finished = drive_target(robot, target, 2, 20, 5);
      robot.rotation = normalize_angle(target_angle - robot.field_angle);
      robot.dribling = config.strategy.max_dribling;
      if (finished) {
        r_status = AttackerRStatus::ROTATE_2;
      }
    } else if (r_status == AttackerRStatus::ROTATE_2) {
      spdlog::info("ROTATE 2");
      if (config.strategy.attacker_ricochet.enabled) {
        double alpha;
        alpha = compute_ricochet(robot, r_left);
        kick_dir(robot, alpha, 100, 0, true, 0, 0.01);
      } else {
        attacker_simple(robot, goal);
      }
    }
  } else {
    r_status = AttackerRStatus::NONE;
    r_side = AttackerRSide::NONE;
    if (millis() - last_ball_visible < 3000) {
      if (last_ball.y < BORDER) {
        Vec target;
        if (last_ball.x < 91) {
          target = {20, 120};
        } else {
          target = {160, 120};
        }
        spdlog::info("DRIVE PASSIVE");
        drive_target(robot, target, 4);
        robot.rotation =
            (last_ball - robot.position).field_angle() - robot.field_angle;
      } else {
        spdlog::info("DRIVE_BALL {} {}", last_ball.x, last_ball.y);
        drive_ball(robot, last_ball);
      }
    } else {
      spdlog::info("DRIVE CENTER");
      drive_target(robot, {91, 121}, 3);
      robot.rotation = 0;
    }
  }

  robot.vel = robot.vel.resize(min(robot.vel.len(), 120.0));
}