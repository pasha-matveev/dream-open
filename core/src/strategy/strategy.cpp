#include "strategy/strategy.h"

#include <spdlog/spdlog.h>

#include <chrono>
#include <cstdint>

#include "utils/config.h"
#include "utils/millis.h"
#include "utils/vec.h"

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
  robot.prev_emitter = robot.emitter;

  robot.kicker_force = 0;
  robot.dribling = 0;

  if (config.strategy.enabled) {
    reset_target = true;
    if (role == "attacker") {
      run_attacker(robot, ball, goal);
    } else {
      run_keeper(robot, ball, goal);
    }
    if (reset_target) {
      target_status = "none";
      target_angle = -1;
      slow_tm = -1;
    }
    field.apply(robot);
  }
}

void Strategy::hit(Robot& robot, Object& goal, bool slow, int power) {
  reset_target = false;
  if (target_status == "none") {
    Vec vel{-1 * sin(robot.field_angle) * 10.0, cos(robot.field_angle) * 10.0};
    robot.vel = vel;
    robot.dribling = 60;
    robot.rotation_limit = 0;
    if (millis() > robot.first_time + 500) {
      target_status = "rotate";
    }
  } else if (target_status == "rotate") {
    if (goal.visible) {
      target_angle = normalize_angle(goal.relative_angle + robot.gyro_angle);
    } else {
      Vec center{91, 237};
      Vec simple_route = center - robot.position;
      target_angle = normalize_angle(simple_route.field_angle() -
                                     robot.field_angle + robot.gyro_angle);
    }
    double delta = normalize_angle(target_angle - robot.gyro_angle);
    if (abs(delta) <= M_PI / 3) {
      target_status = "ac";
    }
    robot.rotation = delta;
    robot.rotation_limit = 15;
    robot.dribling = 60;
    robot.vel = {0, 0};
  } else if (target_status == "ac") {
    double delta = normalize_angle(target_angle - robot.gyro_angle);
    if (abs(delta) <= 0.1) {
      if (slow) {
        target_status = "slow";
        slow_tm = millis() + 400;
      } else {
        target_status = "kick";
      }
      robot.rotation = 0;
      robot.dribling = 0;
      robot.vel = {0, 0};
    } else {
      robot.rotation = delta;
      robot.rotation_limit = 15;
      robot.dribling = 70;
      robot.vel = {0, 0};
    }
  } else if (target_status == "slow") {
    robot.rotation = 0;
    robot.dribling = 0;
    robot.vel = {0, 0};
    if (slow_tm < millis()) {
      target_status = "kick";
    }
  } else if (target_status == "kick") {
    robot.kicker_force = power;
    robot.dribling = 0;
    robot.rotation = 0;
    robot.vel = {0, 0};
  }
}