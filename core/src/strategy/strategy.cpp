#include "strategy/strategy.h"

#include <spdlog/spdlog.h>

#include "utils/vec.h"

void Strategy::run(Robot &robot, Ball &ball) {
  if (ball.visible) {
    robot.rotation = ball.relative_angle;
  }
}