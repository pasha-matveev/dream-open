#include <algorithm>
#include <cmath>
#include <optional>

#include "strategy/strategy.h"
#include "utils/millis.h"

void Strategy::run_keeper(Robot& robot, Object& ball, Object& goal) {
  cout << "emitter: " << robot.emitter << endl;
  cout << "visible: " << ball.visible << endl;
  cout << "last_visible " << last_ball_visible << endl;
  cout << "last_y " << last_ball.y << endl;

  if (millis() - last_ball_visible < 1000 || robot.emitter) {
    if (last_ball.y < 90) {
      active_def = true;
    } else if (last_ball.y > 95) {
      active_def = false;
    }

    if (robot.emitter) {
      active_def = true;
    }

    if (active_def) {
      robot.dribling = 60;
      if (robot.emitter) {
        hit(robot, goal, false, 70, false);
      } else {
        cout << "TAKE" << endl;
        cout << last_ball.x << " " << last_ball.y << endl;
        // Vec target{clamp(last_ball.x, 15.0, 160.0),
        //            max(10.0, last_ball.y)};
        Vec vel = last_ball - robot.position;
        if (vel.len() <= 7) {
          vel = vel.resize(5);
        } else {
          vel = vel.resize(min(vel.len() * 2, 80.0));
        }

        robot.vel = vel;
        robot.dribling = 60;
        robot.rotation = vel.field_angle() - robot.field_angle;
        robot.rotation_limit = 30;
      }
    } else {
      cout << "PASSIVE" << endl;
      Vec target;
      if (last_ball.y > 50.0) {
        Vec our{91, 0};
        double delta_x = last_ball.x - our.x;
        double delta_y1 = last_ball.y;
        double delta_y2 = 50;
        double x1 = delta_x / delta_y1 * delta_y2;
        double final_x = x1 + our.x;
        target = {clamp(final_x, 50.0, 130.0), 52.0};
      } else {
        target = {clamp(last_ball.x, 50.0, 130.0), 52.0};
      }

      Vec vel = target - robot.position;
      vel *= 3;
      robot.dribling = 0;
      vel = vel.resize(min(vel.len(), 80.0));
      robot.vel = vel;
      robot.rotation =
          (last_ball - robot.position).field_angle() - robot.field_angle;
      robot.rotation_limit = 20;
    }
  } else {
    cout << "CONTR" << endl;
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
        (*nearest_obstacle - robot.position).len() > 70 ||
        nearest_obstacle->y < 30) {
      target = {91.0, 35.0};
    } else {
      target = {nearest_obstacle->x, 35.0};
    }
    Vec vel = target - robot.position;
    vel *= 4;
    robot.dribling = 0;
    vel = vel.resize(min(vel.len(), 100.0));
    robot.vel = vel;
    robot.rotation = -1 * robot.field_angle;
    robot.rotation_limit = 20;
  }
}
