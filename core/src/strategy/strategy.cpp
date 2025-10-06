#include "strategy/strategy.h"

#include <spdlog/spdlog.h>

#include "utils/vec.h"

void Strategy::run(Robot &robot, Ball &ball) {
  if (ball.visible) {
    // robot.rotation = ball.relative_angle - robot.gyro_angle;
    // robot.rotation = robot.gyro_angle - ball.relative_angle;
    if (robot.emitter) {
      robot.dribling = 0;
      robot.rotation = 0;
      robot.speed = 0;
      robot.kicker_force = 20;
    } else {
      robot.dribling = 30;
      robot.rotation = ball.relative_angle;
      robot.speed = 30;
      robot.direction = ball.relative_angle;
      robot.kicker_force = 0;
    }
  } else {
    robot.rotation = 0;
    robot.speed = 0;
    robot.kicker_force = 0;
    cout << "invisible" << endl;
  }
  cout << ball.get_cm() << endl;
}