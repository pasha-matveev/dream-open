#include <algorithm>
#include <cmath>
#include <optional>

#include "strategy/strategy.h"

void Strategy::run_keeper(Robot& robot, Object& ball, Object& goal) {
  // return;
  if (millis() - last_ball_visible < 1000) {
    if (ball.field_position.y < 100) {
      active_def = true;
    } else if (ball.field_position.y > 110) {
      active_def = false;
    }

    if (active_def) {
      // cout << ball.visible << '\n';
      // cout << ball.field_position.x << " " << ball.field_position.y << '\n';
      // cout << "---" << endl;
      robot.dribling = 60;
      if (robot.emitter) {
        if (millis() - robot.first_time < 500) {
          robot.vel = ball.field_position - robot.position;
          robot.vel = robot.vel.resize(10);
          robot.rotation_limit = 0;
          goal_direction = -10;
        } else {
          robot.rotation_limit = 10;
          robot.vel = robot.vel.resize(0);

          Vec target{91, 237};
          Vec route = target - robot.position;
          double target_angle;
          if (goal_direction != -10) {
            target_angle = goal_direction;
          } else if (goal.visible) {
            target_angle = goal.relative_angle + robot.gyro_angle;
            goal_direction = target_angle;
          } else {
            target_angle =
                route.field_angle() - robot.field_angle + robot.gyro_angle;
            goal_direction = target_angle;
          }

          cout << "diff: " << abs(target_angle - robot.gyro_angle) << endl;
          if (abs(target_angle - robot.gyro_angle) <= 0.1) {
            cout << "fire !" << endl;
            robot.kicker_force = 70;  // здесь было 100
            robot.rotation = robot.field_angle;
            throttle = millis() + 1000;
          } else {
            robot.rotation = target_angle - robot.gyro_angle;
          }
        }
      } else {
        // Vec target{clamp(ball.field_position.x, 15.0, 160.0),
        //            max(10.0, ball.field_position.y)};
        Vec vel = ball.field_position - robot.position;
        if (vel.len() <= 7) {
          vel = vel.resize(1);
        } else {
        }

        robot.vel = vel;
        robot.dribling = 60;
        robot.rotation = vel.field_angle() - robot.field_angle;
        robot.rotation_limit = 30;
      }
    } else {
      Vec target{clamp(ball.field_position.x, 50.0, 130.0), 40.0};
      Vec vel = target - robot.position;
      vel *= 3;
      robot.dribling = 0;
      vel = vel.resize(min(vel.len(), 50.0));
      robot.vel = vel;
      robot.rotation = ball.relative_angle;
      robot.rotation_limit = 20;
    }
  } else {
    optional<Vec> nearest_obstacle;
    if (robot.lidar) {
      for (const auto& obstacle : robot.lidar->obstacles_data) {
        if (!nearest_obstacle.has_value() ||
            (obstacle - robot.position).len() <
                (*nearest_obstacle - robot.position).len()) {
          nearest_obstacle = obstacle;
        }
      }
    }
    Vec target;
    if (!nearest_obstacle.has_value() ||
        (*nearest_obstacle - robot.position).len() > 70) {
      target = {91.0, 35.0};
    } else {
      target = {nearest_obstacle->x, 35.0};
    }
    Vec vel = target - robot.position;
    vel *= 3;
    robot.dribling = 0;
    vel = vel.resize(min(vel.len(), 50.0));
    robot.vel = vel;
    robot.rotation = -1 * robot.field_angle;
    robot.rotation_limit = 20;
  }
}

// void Strategy::run_keeper(Robot& robot, Object& ball) {
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
