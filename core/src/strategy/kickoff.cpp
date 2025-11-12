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
  robot.state = RobotState::PAUSE;
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
  if (elapsed > 5000) {
    spdlog::warn("KICKOFF TIMEOUT");
    finish(robot);
    return;
  }
  if (robot.emitter) {
    // 1.7
    double alpha = compute_ricochet(robot, left);
    kick_dir(robot, alpha, 100, 600, true, 1000, 0.01);
  } else {
    robot.vel = Vec{robot.field_angle + ball.relative_angle}.resize(5);
    robot.rotation = ball.relative_angle;
    robot.dribling = config.strategy.base_dribling;
  }
}