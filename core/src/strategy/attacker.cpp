#include "strategy/strategy.h"

void Strategy::run_attacker(Robot& robot, Object& ball) {
  bool straight_path = true;

  if (millis() - last_ball_visible < 3000) {
    if (!robot.emitter) {
      if (!straight_path) {
      } else {
        robot.vel = ball.field_position - robot.position;
        robot.vel *= 3;
        robot.rotation = ball.relative_angle;
        robot.rotation_limit = 30;
        robot.dribling = 40;
      }
    } else {
      if (millis() - robot.first_time < 500) {
        robot.vel = ball.field_position - robot.position;
        robot.vel = robot.vel.resize(10);
        robot.dribling = 60;
        robot.rotation_limit = 0;

        // curve kick planning
      } else {
        if (!curve_kick) {
          // recognition
          Vec target{91, 237};
          double angle =
              (target - robot.position).field_angle() - robot.field_angle;
          robot.rotation = angle;
          robot.vel = {10, 0};
          robot.rotation_limit = 20;
          robot.dribling = 60;
          if (abs(angle) <= 0.1) {
            robot.kicker_force = 70;
          }
        } else {
          // curve kick
        }
      }
    }
  } else {
    Vec target{91, 121};
    robot.vel = target - robot.position;
    robot.rotation = 0;
    robot.rotation_limit = 30;
  }
}

// void Strategy::run_attacker(Robot &robot, Object &ball) {
//   if (robot.emitter) {
//     Vec target{91, 243};
//     Vec v = target - robot.position;
//     Vec robot_dir{-1 * sin(robot.field_angle), cos(robot.field_angle)};
//     double ang = atan2(robot_dir % v, robot_dir * v);
//     robot.rotation_limit = 10;
//     robot.dribling = 50;
//     robot.rotation = ang;
//     robot.speed = 0;
//     robot.kicker_force = 0;
//     if (abs(ang) <= 0.05) {
//       robot.kicker_force = 30;
//     }
//   } else if (ball.visible) {
//     robot.dribling = 40;
//     robot.rotation = ball.relative_angle;
//     robot.rotation_limit = 30;
//     if (ball.get_cm() > 10) {
//       robot.speed = 40;
//     } else {
//       robot.speed = 10;
//     }
//     robot.direction = ball.relative_angle;
//     robot.kicker_force = 0;
//   } else {
//     robot.rotation = 0;
//     robot.speed = 0;
//     robot.kicker_force = 0;
//   }
// }