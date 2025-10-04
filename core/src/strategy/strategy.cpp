#include "strategy/strategy.h"

#include <spdlog/spdlog.h>

#include "utils/vec.h"

void Strategy::run(Robot &robot, Ball &ball) {
  if (ball.visible) {
    // robot.rotation = ball.relative_angle - robot.gyro_angle;
    // robot.rotation = robot.gyro_angle - ball.relative_angle;
    robot.rotation = ball.relative_angle;

  } else {
    cout << "invisible" << endl;
  }
  cout << robot.gyro_angle << " | " << ball.relative_angle << endl;
}