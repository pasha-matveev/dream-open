#include "strategy/motion.h"
#include "config/config.h"
#include "config/strategy.h"
#include "utils/mapper.h"
#include "utils/millis.h"

void accelerated_dribbling(Robot& robot) {
  robot.dribbling = config->strategy->dribbling->map(millis() - robot.first_time);
}

void desired_dribbling(Robot& robot, bool ac_dribbling) {
  if (ac_dribbling) {
    accelerated_dribbling(robot);
  } else {
    robot.dribbling = config->strategy->dribbling->value_r;
  }
}
