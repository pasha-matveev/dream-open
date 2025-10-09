#include "strategy/strategy.h"

#include <spdlog/spdlog.h>

#include "utils/vec.h"

void Strategy::run(Robot &robot, Ball &ball) {
  // cout << ball.get_cm() << endl;
  robot.compute_lidar();

  if (ball.visible) {
    if (robot.emitter) {
      Vec target{91, 243};
      Vec v = target - robot.position;
      Vec robot_dir{-1 * sin(robot.field_angle), cos(robot.field_angle)};
      double ang = atan2(robot_dir % v, robot_dir * v);
      robot.rotation_limit = 10;
      robot.dribling = 30;
      robot.rotation = ang;
      robot.speed = 0;
      robot.kicker_force = 0;
      if (abs(ang) <= 0.01) {
        robot.kicker_force = 10;
      }
    } else {
      robot.dribling = 30;
      robot.rotation = ball.relative_angle;
      robot.rotation_limit = 50;
      robot.speed = 30;
      robot.direction = ball.relative_angle;
      robot.kicker_force = 0;
    }
  } else {
    robot.rotation = 0;
    robot.speed = 0;
    robot.kicker_force = 0;
  }
}