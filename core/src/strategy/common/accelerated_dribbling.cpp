#include "strategy/motion.h"
#include "utils/mapper.h"
#include "utils/millis.h"

void accelerated_dribbling(Robot& robot, const Mapper& dribbling) {
  robot.dribbling = dribbling.map(millis() - robot.first_time);
}
