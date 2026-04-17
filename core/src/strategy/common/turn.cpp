#include "strategy/strategy.h"
#include "utils/config.h"
#include "utils/millis.h"

constexpr int TURN_ACCEL_TIME = 300;

bool Strategy::turn(Robot& robot, const TurnParams& params) {
  reset_turn = false;
  double delta = params.target_field_angle - robot.field_angle;
  long long passed = millis() - turn_start_time;
  double k = min(1.0, (double)passed / TURN_ACCEL_TIME);
  if (params.curved_rotation) {
    if (abs(delta) <= 0.035) {
      robot.vel = {0, 0};
      robot.rotation_limit = 15.0;
    } else {
      Vec vel{robot.field_angle};
      if (delta > 0) {
        vel = vel.turn_right();
        vel = vel.rotate(0.1);
      } else {
        vel = vel.turn_left();
        vel = vel.rotate(-0.1);
      }
      // Длина (17) и скорость поворота (15) подобраны
      vel = vel.resize(19.0 * k);
      robot.vel = vel;
      robot.rotation_limit = 15.0 * k;
    }
  } else {
    robot.vel = {0, 0};
    robot.rotation_limit = 15.0 * k;
  }
  robot.rotation = delta;
  desired_dribling(robot, params.accelerated_dribbling);
  if (abs(delta) <= config.strategy.turn_precision) {
    // поворот закончен
    return true;
  }
  return false;
}