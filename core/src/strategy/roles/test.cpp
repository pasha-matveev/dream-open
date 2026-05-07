#include "strategy/test.h"

#include <spdlog/spdlog.h>

#include <vector>

#include "config/config.h"
#include "config/strategy.h"
#include "robot.h"
#include "strategy/ball_tracker.h"
#include "strategy/motion.h"
#include "tracking/object.h"
#include "utils/geo/vec.h"
#include "utils/mapper.h"
#include "utils/millis.h"

namespace {

void run_circle(TestContext& ctx) {
  Robot& robot = ctx.robot;
  robot.dribling = config->strategy->dribbling->value_r;
  double dir = M_PI / 2;
  Vec vel{robot.field_angle};
  vel = vel.turn_right();
  vel = vel.rotate(0.1);
  vel = vel.resize(19);
  robot.vel = vel;
  robot.rotation_limit = 15;
  robot.rotation = dir;
}

void run_dribbling(TestContext& ctx) {
  ctx.robot.dribling = config->strategy->dribbling->value_r;
}

void run_ball_dist(TestContext& ctx) {
  if (ctx.tracker.recently_visible(millis(), 1)) {
    double dist = (ctx.tracker.position() - ctx.robot.position).len();
    spdlog::warn("{}", dist);
  }
}

// Состояние конечного автомата для маршрута квадратом — оставляем как было,
// файловое статическое состояние эквивалентно прежнему `static int state` в
// старом test.cpp.
int square_state = 0;

void run_square(TestContext& ctx) {
  const double PADDING = 70;
  Robot& robot = ctx.robot;

  if (robot.emitter) {
    std::vector<Vec> points = {{PADDING, PADDING},
                               {182 - PADDING, PADDING},
                               {182 - PADDING, 220 - PADDING},
                               {PADDING, 220 - PADDING}};
    std::vector<double> point_rotation = {M_PI / 2, M_PI, -M_PI / 2, 0};

    auto handle = [&](Robot& robot, Vec& target) {
      robot.rotation =
          normalize_angle(point_rotation[square_state] - robot.field_angle);
      robot.rotation_limit = 15;
      drive_target(robot, target, 1, 80, 80);
      if ((target - robot.position).len() <= 10) {
        square_state = (square_state + 1) % 4;
      }
    };

    handle(robot, points[square_state]);
    robot.dribling = config->strategy->dribbling->value_r;
  }
}

void run_emitter(TestContext& ctx) {
  spdlog::info("{}", ctx.robot.raw_emitter);
}

void run_mirror(TestContext& ctx) {
  spdlog::info("{}", ctx.ball.get_pixels_dist());
}

}  // namespace

bool TestController::run(const std::string& role, TestContext& ctx) {
  if (role == "test_circle")
    run_circle(ctx);
  else if (role == "test_dribbling")
    run_dribbling(ctx);
  else if (role == "test_mirror")
    run_mirror(ctx);
  else if (role == "test_emitter")
    run_emitter(ctx);
  else if (role == "test_ball_dist")
    run_ball_dist(ctx);
  else if (role == "test")
    run_square(ctx);
  else
    return false;
  return true;
}
