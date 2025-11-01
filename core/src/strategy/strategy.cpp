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

void Strategy::run(Robot& robot, Object& ball, Object& goal,
                   const Field& field) {
  if (millis() < throttle) {
    robot.vel = robot.vel.resize(0);
    robot.rotation_limit = 0;
    return;
  }

  if (config.serial.enabled) {
    robot.compute_gyro_angle();
  }
  bool lidar_data = robot.compute_lidar();
  if (!lidar_data && config.serial.enabled) {
    robot.predict_position();
  }
  if (!config.visualization.interactive && ball.visible) {
    ball.compute_field_position(robot);
  }

  if (ball.visible) {
    last_ball_visible = millis();
    last_ball = ball.field_position;
  }

  if (robot.emitter && !robot.prev_emitter) {
    robot.first_time = millis();
  }
  robot.prev_emitter = robot.emitter;

  robot.kicker_force = 0;
  robot.dribling = 0;

  if (role == "attacker") {
    run_attacker(robot, ball);
  } else {
    run_keeper(robot, ball, goal);
  }
  field.apply(robot);
}
