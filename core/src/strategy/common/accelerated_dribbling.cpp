#include "strategy/strategy.h"
#include "utils/millis.h"

void Strategy::accelerated_dribbling(Robot& robot) {
  double power =
      base_dribling + (max_dribling - base_dribling) *
                          ((millis() - robot.first_time) / dribling_duration);
  robot.dribling = min(100.0, power);
}