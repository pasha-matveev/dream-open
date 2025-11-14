#include <spdlog/spdlog.h>

#include <cmath>

#include "strategy/strategy.h"
#include "utils/config.h"
#include "utils/millis.h"

static long long kickoff_start = -1;
static bool shot = false;
static bool fw = false;
static bool has_target = false;
static double target = 0;
static long long go_time = -1;

static void finish(Robot& robot) {
  robot.state = RobotState::RUNNING;
  kickoff_start = -1;
  shot = false;
  fw = false;
  has_target = false;
  target = 0;
  go_time = -1;
}

void Strategy::run_kickoff(Robot& robot, Object& ball, Object& goal,
                           bool left) {
  if (kickoff_start == -1) {
    kickoff_start = millis();
  }
  long long elapsed = millis() - kickoff_start;
  if (elapsed > 3000) {
    spdlog::warn("KICKOFF TIMEOUT");
    finish(robot);
    return;
  }
  if (robot.emitter) {
    robot.vel = Vec{robot.field_angle}.resize(20);

    if (millis() - robot.first_time >= 200) {
      robot.kicker_force = 100;
      spdlog::info("KICKOFF FINISH");
      finish(robot);
      return;
    }
  } else {
    double alpha = 60.0 / 180.0 * M_PI;
    if (left) {
      alpha *= 1;
    } else {
      alpha *= -1;
    }
    double dir = normalize_angle(alpha - robot.field_angle);
    if (abs(dir) <= 0.04) {
      robot.vel = Vec{robot.field_angle}.resize(20);
    } else {
      robot.rotation = dir;
      robot.rotation_limit = 18;
      if (left) {
        robot.vel = Vec{robot.field_angle}.turn_right().resize(25);
      } else {
        robot.vel = Vec{robot.field_angle}.turn_left().resize(25);
      }
    }
  }
}