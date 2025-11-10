#include "strategy/strategy.h"
#include "utils/geo/polygon.h"
#include "utils/millis.h"

enum class AttackerRSide { NONE, LEFT, RIGHT };
enum class AttackerRStatus { NONE, TAKE_BALL, ROTATE_1, MOVE, ROTATE_2, KICK };

static AttackerRSide r_side = AttackerRSide::NONE;
static AttackerRStatus r_status = AttackerRStatus::NONE;

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
      int power = 70;
      if (robot.position.y < 146) {
        power = 40;
      } else if (robot.position.y < 180) {
        power = 20;
      } else {
        power = 15;
      }
      hit(robot, goal, power);
    } else if (r_status == AttackerRStatus::TAKE_BALL) {
      bool finished = take_ball(robot, 600);
      if (finished) {
        r_status = AttackerRStatus::ROTATE_1;
      }
    } else if (r_status == AttackerRStatus::ROTATE_1) {
      robot.dribling = max_dribling;
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
      Vec target;
      double target_angle;
      if (r_left) {
        target = Vec{45.0, 170.0};
        target_angle = M_PI / 2;
      } else {
        target = Vec{182.0 - 45.0, 170.0};
        target_angle = -M_PI / 2;
      }
      bool finished = drive_target(robot, target, 2, 20, 10);
      robot.rotation = normalize_angle(target_angle - robot.field_angle);
      if (finished) {
        r_status = AttackerRStatus::ROTATE_2;
      }
    } else if (r_status == AttackerRStatus::ROTATE_2) {
      double alpha = compute_ricochet(robot, r_left);
      kick_dir(robot, alpha, 100, 0, true, 0, 0.01);
    }
  } else {
    r_status = AttackerRStatus::NONE;
    r_side = AttackerRSide::NONE;
    if (millis() - last_ball_visible < 3000) {
      if (last_ball.y < BORDER) {
        drive_target(robot, {160, 120}, 4);
        robot.rotation =
            (last_ball - robot.position).field_angle() - robot.field_angle;
      } else {
        drive_ball(robot, last_ball);
      }
    } else {
      drive_target(robot, {91, 121}, 3);
      robot.rotation = 0;
    }
  }

  robot.vel = robot.vel.resize(min(robot.vel.len(), 120.0));
}