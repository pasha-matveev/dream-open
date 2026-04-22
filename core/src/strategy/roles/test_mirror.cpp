#include <spdlog/spdlog.h>

#include "strategy/strategy.h"

void Strategy::run_test_mirror(Robot& robot, Object& ball) {
  spdlog::info("{}", ball.get_pixels_dist());
}