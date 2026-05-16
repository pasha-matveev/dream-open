#include "strategy/accel_drive.h"

#include "config/config.h"
#include "config/strategy.h"
#include "config/strategy/control.h"
#include "robot.h"
#include "strategy/motion.h"
#include "utils/millis.h"

bool AccelDriveController::execute(Robot& robot,
                                   const AccelDriveParams& params) {
  reset_pending_ = false;
  long long now = millis();
  double dt_ms = (last_tick_ms_ < 0) ? 0.0 : (double)(now - last_tick_ms_);
  last_tick_ms_ = now;

  double max_accel = config->strategy->control->ball_drive->max_accel;
  double step = max_accel * dt_ms / 1000.0;
  current_speed_cap_ = min(params.max_speed, current_speed_cap_ + step);

  return drive_target(robot, params.target,
                      {.max_speed = current_speed_cap_,
                       .min_speed = params.min_speed});
}
