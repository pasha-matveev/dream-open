#include "strategy/strategy.h"
#include "utils/config.h"
#include "utils/millis.h"

void Strategy::run_test_circle(Robot& robot) {
  robot.dribling = config.strategy.dribbling.value_r;
  double dir = M_PI / 2;
  Vec vel{robot.field_angle};
  vel = vel.turn_right();
  vel = vel.rotate(0.1);
  vel = vel.resize(19);
  robot.vel = vel;
  robot.rotation_limit = 15;
  robot.rotation = dir;
}

void Strategy::run_test_dribling(Robot& robot) {
  robot.dribling = config.strategy.dribbling.value_r;
}

static int state = 0;

void Strategy::run_test(Robot& robot, Object& goal) {
  const double PADDING = 70;

  if (robot.emitter) {
    vector<Vec> points = {{PADDING, PADDING},
                          {182 - PADDING, PADDING},
                          {182 - PADDING, 220 - PADDING},
                          {PADDING, 220 - PADDING}};
    vector<double> point_rotation = {M_PI / 2, M_PI, -M_PI / 2, 0};

    auto handle = [&](Robot& robot, Vec& target) {
      robot.rotation =
          normalize_angle(point_rotation[state] - robot.field_angle);
      robot.rotation_limit = 15;
      drive_target(robot, target, 1, 80, 80);
      if ((target - robot.position).len() <= 10) {
        state = (state + 1) % 4;
      }
    };

    handle(robot, points[state]);
    robot.dribling = config.strategy.dribbling.value_r;
  } else {
  }
}