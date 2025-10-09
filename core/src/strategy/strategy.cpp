#include "strategy/strategy.h"

#include <spdlog/spdlog.h>

#include "utils/vec.h"

void Strategy::run(Robot &robot, Ball &ball) {
  robot.compute_lidar();

  cout << robot.emitter << endl;

  // if (ball.visible) {
  //   if (robot.emitter) {
  //     robot.dribling = 0;
  //     robot.rotation = 0;
  //     robot.speed = 0;
  //     robot.kicker_force = 20;
  //   } else {
  //     robot.dribling = 50;
  //     robot.rotation = ball.relative_angle;
  //     robot.rotation_limit = 50;
  //     robot.speed = 80;
  //     robot.direction = ball.relative_angle;
  //     robot.kicker_force = 0;
  //   }
  // } else {
  //   robot.rotation = 0;
  //   robot.speed = 0;
  //   robot.kicker_force = 0;
  // }
}