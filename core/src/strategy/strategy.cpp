#include "strategy/strategy.h"

#include <spdlog/spdlog.h>

#include "utils/config.h"
#include "utils/vec.h"

Strategy::Strategy() { role = config["strategy"]["role"].GetString(); }

void Strategy::run(Robot& robot, Ball& ball) {
  bool lidar_data = robot.compute_lidar();
  if (!lidar_data) {
    robot.predict_position();
  }
  if (!config["visualization"]["enabled"].GetBool()) {
    ball.compute_field_position(robot);
  }

  if (role == "attacker") {
    run_attacker(robot, ball);
  } else {
    run_keeper(robot, ball);
  }
}
