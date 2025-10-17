#include "strategy/strategy.h"

#include <spdlog/spdlog.h>

#include "utils/config.h"
#include "utils/vec.h"

Strategy::Strategy() { role = config["strategy"]["role"].GetString(); }

void Strategy::run(Robot &robot, Ball &ball) {
  robot.compute_lidar();
  if (role == "attacker") {
    run_attacker(robot, ball);
  } else {
    run_keeper(robot, ball);
  }
}