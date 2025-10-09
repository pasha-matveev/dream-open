#include "strategy/strategy.h"

#include <spdlog/spdlog.h>

#include "utils/vec.h"

void Strategy::run(Robot &robot, Ball &ball) {
  robot.compute_lidar();
  cout << robot.field_angle << endl;
  // if (robot.lidar) {
  //   auto [computed, v] = robot.lidar->compute();
  //   if (computed) {
  //     Vec center = {100, 100};
  //     robot.position = center + v;
  //     cout << "Position: " << robot.position.x << " " << robot.position.y
  //          << endl;
  //   } else {
  //     // cout << "Not computed" << endl;
  //   }
  // }
  // if (ball.visible) {
  //   if (robot.emitter) {
  //     robot.dribling = 0;
  //     robot.rotation = 0;
  //     robot.speed = 0;
  //     robot.kicker_force = 20;
  //   } else {
  //     robot.dribling = 30;
  //     robot.rotation = ball.relative_angle;
  //     robot.speed = 30;
  //     robot.direction = ball.relative_angle;
  //     robot.kicker_force = 0;
  //   }
  // } else {
  //   robot.rotation = 0;
  //   robot.speed = 0;
  //   robot.kicker_force = 0;
  // }
}