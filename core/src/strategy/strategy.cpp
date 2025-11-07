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
  robot.rotation = 0;
  robot.vel = {0, 0};

  if (config.strategy.enabled) {
    reset_target = true;
    if (robot.state == RobotState::RUNNING) {
      if (role == "attacker") {
        run_attacker(robot, ball, goal);
      } else if (role == "keeper") {
        run_keeper(robot, ball, goal);
      } else {
        run_challenge(robot, ball, goal);
      }
    } else if (robot.state == KICKOFF_LEFT) {
      run_kickoff(robot, ball, goal, true);
    } else if (robot.state == KICKOFF_RIGHT) {
      run_kickoff(robot, ball, goal, false);
    }

    if (reset_target) {
      target_status = "none";
      target_angle = -1;
      slow_tm = -1;
    }
    field.apply(robot);
  }
}

void Strategy::drive_target(Robot& robot, const Vec& target, double k) {
  Vec vel = target - robot.position;
  robot.rotation = normalize_angle(vel.field_angle() - robot.field_angle);
  vel *= k;
  vel = vel.resize(min(vel.len(), 120.0));
  robot.vel = vel;
}

void Strategy::drive_ball(Robot& robot, const Vec& ball) {
  double k;
  double dist = (ball - robot.position).len();
  if (dist < 16) {
    k = 1;
  } else if (dist < 40) {
    k = 2.5;
  } else {
    k = 4;
  }
  drive_target(robot, ball, k);
  robot.dribling = 50;
  robot.rotation_limit = 40;
}

void Strategy::accelerated_dribbling(Robot& robot) {
  robot.dribling =
      min(100.0, base_dribling +
                     (max_dribling - base_dribling) *
                         ((millis() - robot.first_time) / dribling_duration));
}

void Strategy::hit(Robot& robot, Object& goal, int forward_timeout,
                   bool curved_rotation, int kick_timeout, int power,
                   double precision) {
  reset_target = false;
  if (target_status == "none") {
    spdlog::info("NONE");
    long long passed_time = millis() - robot.first_time;
    long long left_time = forward_timeout - passed_time;
    if (left_time <= 0) {
      target_status = "rotate";
    } else {
      Vec vel{-1 * sin(robot.field_angle) * 10.0 * left_time / forward_timeout,
              cos(robot.field_angle) * 10.0 * left_time / forward_timeout};
      robot.vel = vel;
      accelerated_dribbling(robot);
      robot.rotation_limit = 0;
      if (millis() > robot.first_time + forward_timeout) {
        target_status = "rotate";
      }
    }

  } else if (target_status == "rotate") {
    spdlog::info("ROTATE");
    if (goal.visible) {
      target_angle = normalize_angle(goal.relative_angle + robot.gyro_angle);
    } else {
      Vec center{91, 237};
      Vec simple_route = center - robot.position;
      target_angle = normalize_angle(simple_route.field_angle() -
                                     robot.field_angle + robot.gyro_angle);
    }
    target_angle -= 0.01;
    double delta = normalize_angle(target_angle - robot.gyro_angle);

    if (abs(delta) <= precision) {
      if (kick_timeout) {
        target_status = "slow";
        robot.dribling = 15;
        slow_tm = millis() + kick_timeout;
      } else {
        target_status = "kick";
        robot.dribling = 0;
      }
      robot.rotation = 0;
      robot.vel = {0, 0};
    } else {
      if (curved_rotation) {
        robot.vel = {-5, -5};
        robot.rotation_limit = 15;
      } else {
        robot.vel = {0, 0};
        robot.rotation_limit = 15;
      }
      if (abs(delta) <= 0.05) {
        robot.rotation_limit = 5;
      }
      robot.rotation = delta;
      accelerated_dribbling(robot);
    }

    // if (abs(delta) <= M_PI / 3) {
    //   target_status = "ac";
    // }
    // if (curved_rotation) {
    //   robot.vel = {-25, 0};
    // } else {
    //   robot.vel = {0, 0};
    // }
    // robot.rotation = delta;
    // robot.rotation_limit = rotation_speed;
    // accelerated_dribbling(robot);

  } else if (target_status == "ac") {
    spdlog::info("AC");
    // double delta = normalize_angle(target_angle - robot.gyro_angle);
    // if (abs(delta) <= precision) {
    //   if (kick_timeout) {
    //     target_status = "slow";
    //     robot.dribling = 15;
    //     slow_tm = millis() + kick_timeout;
    //   } else {
    //     target_status = "kick";
    //     robot.dribling = 0;
    //   }
    //   robot.rotation = 0;
    //   robot.vel = {0, 0};
    // } else {
    //   robot.rotation = delta;
    //   robot.rotation_limit = 12;
    //   accelerated_dribbling(robot);
    //   robot.vel = {0, 0};
    // }

  } else if (target_status == "slow") {
    spdlog::info("SLOW");
    robot.rotation = 0;
    robot.dribling = 15;
    robot.vel = {0, 0};
    if (slow_tm < millis()) {
      target_status = "kick";
    }

  } else if (target_status == "kick") {
    spdlog::info("KICK");
    robot.kicker_force = power;
    if (kick_timeout) {
      robot.dribling = 0;
    } else {
      robot.dribling = max_dribling;
    }
    robot.rotation = 0;
    robot.vel = {0, 0};
  }
}