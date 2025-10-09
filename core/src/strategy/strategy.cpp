#include "strategy/strategy.h"

#include <spdlog/spdlog.h>

#include "utils/vec.h"

void Strategy::run(Robot &robot, Ball &ball) {
  // cout << ball.get_cm() << endl;
  robot.compute_lidar();

  if (ball.visible) {
    if (ball.get_cm() < 8.2) {
      if (abs(robot.field_angle) <= 0.1) {
        robot.kicker_force = 10;
      }
      robot.rotation_limit = 10;
      robot.dribling = 30;
      robot.rotation = robot.field_angle * -1;
      robot.speed = 0;
      robot.kicker_force = 20;
    } else {
      robot.dribling = 30;
      robot.rotation = ball.relative_angle;
      robot.rotation_limit = 50;
      robot.speed = 10;
      robot.direction = ball.relative_angle;
      robot.kicker_force = 0;
    }
  } else {
    robot.rotation = 0;
    robot.speed = 0;
    robot.kicker_force = 0;
  }
}