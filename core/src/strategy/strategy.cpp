#include "strategy/strategy.h"

#include <spdlog/spdlog.h>

#include <atomic>
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
  if (!config.visualization.interactive && ball.visible &&
      robot.camera->new_data()) {
    ball.compute_field_position(robot);
  }

  if (ball.visible) {
    last_ball_visible = millis();
    last_ball = ball.field_position;
    last_ball_relative = ball.relative_angle;
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
    reset_turn = true;
    reset_dubins = true;
    if (robot.state == RobotState::RUNNING) {
      if (role == "attacker") {
        run_attacker(robot, ball, goal);
      } else if (role == "keeper") {
        run_keeper(robot, ball, goal, field);
      } else if (role == "challenge") {
        run_challenge(robot, ball, goal);
      } else if (role == "test_circle") {
        run_test_circle(robot);
      } else if (role == "test_dribling") {
        run_test_dribling(robot);
      } else if (role == "test") {
        run_test(robot, goal);
      } else {
        spdlog::error("Unknown role: {}", role);
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
    if (reset_turn) {
      turn_time = -1;
    }
    if (reset_dubins) {
      dubins_side = DubinsSide::NONE;
    }
    field.apply(robot);
  }
  robot.prev_emitter = robot.emitter;
}
