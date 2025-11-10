#include "strategy/strategy.h"
#include "utils/geo/polygon.h"
#include "utils/millis.h"

static Polygon left_special{{{0, 194}, {0, 243}, {63, 243}}};
static Polygon right_special{{{119, 243}, {182, 243}, {182, 194}}};

void Strategy::run_attacker(Robot& robot, Object& ball, Object& goal) {
  const int BORDER = 80;

  if (robot.emitter) {
    Vec hole_position = robot.ball_hole_position();
    if (!robot.prev_emitter) {
      // только что взяли мяч
      if (left_special.inside(hole_position)) {
        attacker_r_side = AttackerRSide::LEFT;
        attacker_r = AttackerRStatus::R1;
      } else if (right_special.inside(hole_position)) {
        attacker_r_side = AttackerRSide::RIGHT;
        attacker_r = AttackerRStatus::R1;
      }
    }
    bool attacker_r_left = attacker_r_side == AttackerRSide::LEFT;
    if (attacker_r == AttackerRStatus::NONE) {
      int power = 70;
      if (robot.position.y < 146) {
        power = 40;
      } else if (robot.position.y < 180) {
        power = 20;
      } else {
        power = 15;
      }
      hit(robot, goal, power);
    } else if (attacker_r == AttackerRStatus::R1) {
      robot.dribling = max_dribling;
      double target_angle;
      if (attacker_r_left) {
        target_angle = M_PI / 2;
      } else {
        target_angle = -M_PI / 2;
      }
      double delta = target_angle - robot.field_angle;
      robot.rotation = delta;
      robot.rotation_limit = 10;
      if (abs(delta) < 0.05) {
        attacker_r = AttackerRStatus::MOVE;
      }
    } else if (attacker_r == AttackerRStatus::MOVE) {
      robot.dribling = max_dribling;
      Vec target;
      if (attacker_r_left) {
        target = Vec{45.0, 170.0};
      } else {
        target = Vec{182.0 - 45.0, 170.0};
      }
      Vec vel = target - robot.position;
      vel *= 1.5;
      vel = vel.resize(min(vel.len(), 15.0));
      robot.vel = vel;
      if (vel.len() <= 2.0) {
        attacker_r = AttackerRStatus::R2;
      }
    } else if (attacker_r == AttackerRStatus::R2) {
      double alpha = compute_ricochet(robot, attacker_r_left);
      kick_dir(robot, alpha, 100, 0, true, 0, 0.01);
    }
  } else {
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