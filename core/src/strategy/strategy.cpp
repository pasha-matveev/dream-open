#include "strategy/strategy.h"

#include <spdlog/spdlog.h>

#include <chrono>
#include <cstdint>

#include "utils/config.h"
#include "utils/geo/vec.h"
#include "utils/millis.h"

using namespace std::chrono;

Strategy::Strategy() { role = config.strategy.role; }

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

  robot.kicker_force = 0;
  robot.dribling = 0;
  robot.rotation = 0;
  robot.vel = {0, 0};
  if (robot.emitter) {
    robot.rotation_limit = 15;
  } else {
    robot.rotation_limit = 20;
  }

  if (config.strategy.enabled) {
    reset_kick = true;
    if (robot.state == RobotState::RUNNING) {
      if (role == "attacker") {
        run_attacker(robot, ball, goal);
      } else if (role == "keeper") {
        run_keeper(robot, ball, goal, field);
      } else {
        run_challenge(robot, ball, goal);
      }
    } else if (robot.state == KICKOFF_LEFT) {
      run_kickoff(robot, ball, goal, true);
    } else if (robot.state == KICKOFF_RIGHT) {
      run_kickoff(robot, ball, goal, false);
    }

    if (reset_kick) {
      kick_status = "none";
      slow_tm = -1;
    }
    field.apply(robot);
  }
  robot.prev_emitter = robot.emitter;
}
