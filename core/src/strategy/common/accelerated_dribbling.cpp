#include "strategy/strategy.h"
#include "utils/config.h"
#include "utils/millis.h"

void Strategy::accelerated_dribbling(Robot& robot) {
  robot.dribling = config.strategy.dribbling.map(millis() - robot.first_time);
}

void Strategy::desired_dribling(Robot& robot, bool ac_dribling) {
  if (ac_dribling) {
    accelerated_dribbling(robot);
  } else {
    robot.dribling = config.strategy.dribbling.value_r;
  }
}