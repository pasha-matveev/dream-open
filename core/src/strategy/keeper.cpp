#include <algorithm>
#include <cmath>
#include <optional>

#include "strategy/strategy.h"

void Strategy::run_keeper(Robot& robot, Object& __, Object& goal) {
  if (millis() - last_ball_visible < 1000 || robot.emitter) {
    if (last_ball.y < 100) {
      active_def = true;
    } else if (last_ball.y > 110) {
      active_def = false;
    }

    if (robot.emitter) {
      active_def = true;
    }

    if (active_def) {
      robot.dribling = 60;
      if (robot.emitter) {
        if (millis() - robot.first_time < 300) {
          cout << "DELAY" << endl;
          robot.vel = last_ball - robot.position;
          robot.vel = robot.vel.resize(10);
          robot.rotation_limit = 0;
          target_angle = -10;
        } else {
          robot.rotation_limit = 10;
          robot.vel = robot.vel.resize(0);

          if (target_angle == -10) {
            if (goal.visible) {
              target_angle = goal.relative_angle + robot.gyro_angle;
            } else {
              Vec target{91, 237};
              Vec route = target - robot.position;
              target_angle =
                  route.field_angle() - robot.field_angle + robot.gyro_angle;
            }
          }

          double delta = normalize_angle(target_angle - robot.gyro_angle);

          if (abs(delta) <= 0.1) {
            cout << "FIRE" << endl;
            robot.kicker_force = 70;  // здесь было 100
            robot.rotation = 0;
            // throttle = millis() + 1000;
            fired = millis();
          } else {
            cout << "ROTATION " << abs(delta) << endl;
            robot.rotation = target_angle - robot.gyro_angle;
          }
        }
      } else {
        cout << "TAKE" << endl;
        cout << last_ball.x << " " << last_ball.y << endl;
        // Vec target{clamp(last_ball.x, 15.0, 160.0),
        //            max(10.0, last_ball.y)};
        Vec vel = last_ball - robot.position;
        if (vel.len() <= 7) {
          vel = vel.resize(1);
        } else {
          vel = vel.resize(min(vel.len() * 2, 50.0));
        }

        robot.vel = vel;
        robot.dribling = 60;
        robot.rotation = vel.field_angle() - robot.field_angle;
        robot.rotation_limit = 30;
      }
    } else {
      cout << "PASSIVE" << endl;
      // Vec target{clamp(last_ball.x, 50.0, 130.0), 40.0};
      Vec target{90.0, 52.0};
      Vec vel = target - robot.position;
      vel *= 3;
      robot.dribling = 0;
      vel = vel.resize(min(vel.len(), 50.0));
      robot.vel = vel;
      robot.rotation =
          (last_ball - robot.position).field_angle() - robot.field_angle;
      robot.rotation_limit = 20;
    }
  } else {
    cout << "PASSIVE" << endl;
    // Vec target{clamp(last_ball.x, 50.0, 130.0), 40.0};
    Vec target{90.0, 52.0};
    Vec vel = target - robot.position;
    vel *= 3;
    robot.dribling = 0;
    vel = vel.resize(min(vel.len(), 50.0));
    robot.vel = vel;
    robot.rotation =
        (last_ball - robot.position).field_angle() - robot.field_angle;
    robot.rotation_limit = 20;
    // cout << "CONTR" << endl;
    // optional<Vec> nearest_obstacle;
    // if (robot.lidar) {
    //   for (const auto& obstacle : robot.lidar->obstacles_data) {
    //     if (!nearest_obstacle.has_value() ||
    //         (obstacle - robot.position).len() <
    //             (*nearest_obstacle - robot.position).len()) {
    //       nearest_obstacle = obstacle;
    //     }
    //   }
    // }
    // Vec target;
    // if (!nearest_obstacle.has_value() ||
    //     (*nearest_obstacle - robot.position).len() > 70 ||
    //     nearest_obstacle->y < 30) {
    //   target = {91.0, 35.0};
    // } else {
    //   target = {nearest_obstacle->x, 35.0};
    // }
    // Vec vel = target - robot.position;
    // vel *= 3;
    // robot.dribling = 0;
    // vel = vel.resize(min(vel.len(), 50.0));
    // robot.vel = vel;
    // robot.rotation = -1 * robot.field_angle;
    // robot.rotation_limit = 20;
  }
}
