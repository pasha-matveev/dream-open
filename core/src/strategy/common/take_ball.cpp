#include "strategy/strategy.h"
#include "utils/millis.h"

bool Strategy::take_ball(Robot& robot, long long forward_timeout) {
  long long passed_time = millis() - robot.first_time;
  long long left_time = forward_timeout - passed_time;
  bool finished = false;
  if (left_time < 0) {
    left_time = 0;
    finished = true;
  }
  robot.vel = Vec{robot.field_angle}.resize(10.0 * left_time / forward_timeout);
  accelerated_dribbling(robot);
  robot.rotation = 0;
  robot.rotation_limit = 0;
  return finished;
}