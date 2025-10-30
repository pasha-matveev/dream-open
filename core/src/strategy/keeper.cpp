#include <algorithm>
#include <cmath>
#include <optional>

#include "strategy/strategy.h"

void Strategy::run_keeper(Robot& robot, Ball& ball) {
  robot.kicker_force = 0;
  if (millis() - last_ball_visible < 1000) {
    if (ball.field_position.y < 100) {
      active_def = true;
    } else if (ball.field_position.y > 110) {
      active_def = false;
    }

    if (active_def) {
      if (robot.emitter) {
        if (millis() - robot.first_time < 500) {
          robot.speed = 10;
          robot.dribling = 60;
          robot.rotation_limit = 0;
        } else {
          robot.rotation_limit = 10;
          Vec target{91, 237};
          Vec route = target - robot.position;
          double target_angle = route.field_angle();
          if (abs(robot.field_angle - target_angle) <= 0.1) {
            robot.kicker_force = 70;  // здесь было 100
            robot.rotation = robot.field_angle;
            robot.speed = 0;
          } else {
            robot.rotation = target_angle - robot.field_angle;
            robot.rotation_limit = 10;
            robot.speed = 0;
          }
        }
      } else {
        Vec target{clamp(50.0, 130.0, ball.field_position.x), max(15.0, ball.field_position.y)};
        Vec vel = target - robot.position;
        double desired_speed; 
        if (vel.len() <= 7) {
          desired_speed = 1;
        } else {
          desired_speed = vel.len() * 2;
        }

        robot.direction = vel.field_angle() - robot.field_angle;
        robot.speed = desired_speed;
        robot.dribling = 60;
        robot.rotation = vel.field_angle() - robot.field_angle;
        robot.rotation_limit = 30;
      }
    } else {
      Vec target{clamp(ball.field_position.x, 50.0, 130.0), 55.0};
      Vec vel = target - robot.position;
      vel *= 3;
      robot.dribling = 0;
      robot.speed = min(vel.len(), 50.0);
      robot.direction = normalize_angle(vel.field_angle() - robot.field_angle);
      robot.rotation = ball.relative_angle;
      robot.rotation_limit = 20;
    }
  } else {
    robot.speed = 0;
    robot.rotation_limit = 0;
  }
}

// void Strategy::run_keeper(Robot& robot, Ball& ball) {
//   if (!ball.visible) {
//     robot.speed = 0;
//     robot.rotation = 0;
//     while (!q.empty()) {
//       q.pop();
//     }
//     return;
//   }

//   std::optional<Vec> last_position;
//   q.push(ball.field_position);
//   if (q.size() >= 3) {
//     last_position = q.front();
//     q.pop();
//   }

//   bool line_movement = false;

//   Vec target_position = {ball.field_position.x, 55.0};
//   if (last_position.has_value()) {
//     double delta_y = last_position->y - ball.field_position.y;
//     double target_delta_y = last_position->y - 55;
//     if (delta_y > 1 && target_delta_y > 0) {
//       line_movement = true;
//       double delta_x = ball.field_position.x - last_position->x;
//       double target_delta_x = delta_x / delta_y * target_delta_y;
//       double target_x = last_position->x + target_delta_x;
//       target_position = {target_x, 55.0};
//     }
//   }
//   target_position.x = std::clamp(target_position.x, 40.0, 130.0);
//   Vec vel = target_position - robot.position;

//   if (ball.field_position.y < 85 || line_movement) {
//     robot.speed = std::abs(vel.x) * 20;
//   } else {
//     robot.speed = std::abs(vel.x) * 10;
//   }

//   if (ball.field_position.y < 50) {
//     robot.speed = 0;
//   }
//   // robot.speed = min(robot.speed, 30.0f);
//   robot.direction = vel.field_angle() - robot.field_angle;
//   robot.rotation = ball.relative_angle;
//   robot.rotation_limit = 10;
// }
