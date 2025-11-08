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
  if (!robot.emitter) {
    attacker_r = AttackerRStatus::NONE;
  }

  robot.kicker_force = 0;
  robot.dribling = 0;
  robot.rotation = 0;
  robot.vel = {0, 0};

  if (config.strategy.enabled) {
    reset_kick = true;
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

    if (reset_kick) {
      kick_status = "none";
      target_angle = -1;
      slow_tm = -1;
    }
    field.apply(robot);
  }
  robot.prev_emitter = robot.emitter;
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
  robot.dribling = base_dribling;
  robot.rotation_limit = 40;
}

void Strategy::accelerated_dribbling(Robot& robot) {
  double power =
      base_dribling + (max_dribling - base_dribling) *
                          ((millis() - robot.first_time) / dribling_duration);
  robot.dribling = min(100.0, power);
}

double Strategy::compute_ricochet(Robot& robot, bool left) {
  Vec ball_position = robot.ball_hole_position();
  Vec target{91, 230};
  double a;
  if (left) {
    a = ball_position.x;
  } else {
    a = 182 - ball_position.x;
  }
  double b = 217.5 - ball_position.y;
  double c = b / 2;
  // cout << ball_position.x << " " << ball_position.y << " "
  //      << 243.0 - ball_position.y << endl;
  // cout << a << " " << c << endl;
  // exit(0);
  double alpha = atan(a / c);
  double s = (left) ? 1 : -1;
  double res = alpha * s - robot.field_angle;
  return res;
}

void Strategy::kick_dir(Robot& robot, double dir, int power,
                        int forward_timeout, bool curved_rotation,
                        int kick_timeout, double precision) {
  spdlog::warn("FW: {}", forward_timeout);
  reset_kick = false;
  if (kick_status == "none") {
    spdlog::info("NONE");
    long long passed_time = millis() - robot.first_time;
    long long left_time = forward_timeout - passed_time;
    if (left_time <= 0) {
      kick_status = "rotate";
    } else {
      Vec vel{-1 * sin(robot.field_angle) * 10.0 * left_time / forward_timeout,
              cos(robot.field_angle) * 10.0 * left_time / forward_timeout};
      robot.vel = vel;
      accelerated_dribbling(robot);
      robot.rotation_limit = 0;
    }

  } else if (kick_status == "rotate") {
    spdlog::info("ROTATE");
    dir -= 0.01;

    if (abs(dir) <= precision) {
      if (kick_timeout) {
        kick_status = "slow";
        robot.dribling = 15;
        slow_tm = millis() + kick_timeout;
      } else {
        kick_status = "kick";
        robot.dribling = 0;
      }
      robot.rotation = 0;
      robot.vel = {0, 0};
    } else {
      if (curved_rotation) {
        Vec vel{robot.field_angle};
        if (dir > 0) {
          vel = vel.turn_right();
          vel = vel.rotate(0.1);
        } else {
          vel = vel.turn_left();
          vel = vel.rotate(-0.1);
        }
        if (abs(dir) <= 0.05) {
          robot.vel = {0, 0};
          robot.rotation_limit = 15;
        } else {
          vel = vel.resize(17);
          robot.vel = vel;
          robot.rotation_limit = 15;
        }
      } else {
        robot.vel = {0, 0};
        robot.rotation_limit = 15;
      }
      robot.rotation = dir;
      accelerated_dribbling(robot);
    }

  } else if (kick_status == "slow") {
    spdlog::info("SLOW");
    robot.rotation = 0;
    robot.dribling = 15;
    robot.vel = {0, 0};
    if (slow_tm < millis()) {
      kick_status = "kick";
    }

  } else if (kick_status == "kick") {
    spdlog::info("KICK");
    robot.kicker_force = power;
    if (kick_timeout) {
      robot.dribling = 0;
    } else {
      robot.dribling = 15;
    }
    robot.rotation = 0;
    robot.vel = {0, 0};
  }
}
void Strategy::hit(Robot& robot, Object& goal, int power, int forward_timeout,
                   bool curved_rotation, int kick_timeout, double precision) {
  double target_angle;
  if (goal.visible) {
    target_angle = normalize_angle(goal.relative_angle);
  } else {
    Vec center{91, 237};
    Vec simple_route = center - robot.position;
    target_angle =
        normalize_angle(simple_route.field_angle() - robot.field_angle);
  }
  kick_dir(robot, target_angle, power, forward_timeout, curved_rotation,
           kick_timeout, precision);
}