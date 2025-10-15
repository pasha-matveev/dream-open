#include "strategy/strategy.h"

void Strategy::run_keeper(Robot &robot, Ball &ball) {
  robot.rotation = -M_PI / 2;
  robot.rotation_limit = 10;
}