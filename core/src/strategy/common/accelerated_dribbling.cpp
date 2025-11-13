#include "strategy/strategy.h"
#include "utils/config.h"
#include "utils/millis.h"

void Strategy::accelerated_dribbling(Robot& robot) {
  double power =
      config.strategy.base_dribling +
      (config.strategy.max_dribling - config.strategy.base_dribling) *
          ((millis() - robot.first_time) / config.strategy.dribling_duration);
  robot.dribling = min(100.0, power);
}

void Strategy::desired_dribling(Robot& robot, bool ac_dribling) {
  if (ac_dribling) {
    accelerated_dribbling(robot);
  } else {
    robot.dribling = config.strategy.base_dribling;
  }
}