#include "strategy/motion.h"
#include "utils/mapper.h"
#include "utils/millis.h"

bool stabilize_capture(Robot& robot, const StabilizeCaptureParams& params) {
  const int control_time = params.dribbling.param_r;
  long long passed_time = millis() - robot.first_time;
  if (passed_time >= control_time) return false;

  long long left_time = control_time - passed_time;
  double speed = params.initial_speed * left_time / control_time;
  robot.vel = Vec{robot.field_angle}.resize(speed);
  accelerated_dribbling(robot, params.dribbling);
  robot.rotation = 0;
  robot.rotation_limit = 0;
  return true;
}
