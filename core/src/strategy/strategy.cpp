#include "strategy/strategy.h"

#include <spdlog/spdlog.h>

#include <chrono>
#include <cstdint>

#include "utils/config.h"
#include "utils/vec.h"

using namespace std::chrono;

Strategy::Strategy() { role = config.strategy.role; }

long long millis() {
  return duration_cast<milliseconds>(steady_clock::now().time_since_epoch())
      .count();
}

void Strategy::run(Robot& robot, Ball& ball) {
  bool lidar_data = robot.compute_lidar();
  if (!lidar_data) {
    robot.predict_position();
  }
  if (!config.visualization.interactive) {
    ball.compute_field_position(robot);
  }

  if (ball.visible) {
    last_ball_visible = millis();
  }

  if (role == "attacker") {
    run_attacker(robot, ball);
  } else {
    run_keeper(robot, ball);
  }
}
