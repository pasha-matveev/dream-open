#include "strategy/spin_shot.h"

#include <spdlog/spdlog.h>

#include <cassert>
#include <cmath>

#include "config/config.h"
#include "config/strategy.h"
#include "robot.h"
#include "strategy/kick.h"
#include "utils/geo/vec.h"
#include "utils/mapper.h"
#include "utils/millis.h"

bool SpinShotController::execute(Robot& robot, const SpinShotParams& params) {
  reset_pending_ = false;

  if (status_ == Status::IDLE) {
    assert(params.sweep_angle > 0 && params.sweep_angle <= M_PI &&
           "SpinShot: sweep_angle must be in (0, pi]");
    assert((params.direction == 1 || params.direction == -1) &&
           "SpinShot: direction must be +1 or -1");
    assert(params.kicker_angle >= 0 &&
           params.kicker_angle <= params.sweep_angle &&
           "SpinShot: kicker_angle must be in [0, sweep_angle]");
    start_ms_ = millis();
    start_field_angle_ = robot.field_angle;
    target_field_angle_ = normalize_angle(
        start_field_angle_ + params.direction * params.sweep_angle);
    kicker_fired_ = false;
    status_ = Status::SPIN;
    spdlog::info("SPIN start: target={:.2f}", target_field_angle_);
  }

  if (status_ == Status::SPIN) {
    long long elapsed = millis() - start_ms_;

    double delta = normalize_angle(target_field_angle_ - robot.field_angle);
    robot.rotation = delta;
    robot.rotation_limit = params.rotation_limit;
    robot.vel = {0, 0};
    robot.dribbling = params.dribbling.has_value()
                          ? *params.dribbling
                          : config->strategy->dribbling->value_r;

    double swept =
        std::abs(normalize_angle(robot.field_angle - start_field_angle_));

    if (params.kicker_angle > 0 && !kicker_fired_ &&
        swept >= params.kicker_angle) {
      robot.kicker_force = params.kicker_force;
      kicker_fired_ = true;
      spdlog::info("SPIN kicker fire: power={}, swept={:.2f}",
                   robot.kicker_force, swept);
    }

    bool angle_done = swept >= params.sweep_angle;
    bool timeout_done =
        params.spin_timeout_ms > 0 && elapsed >= params.spin_timeout_ms;
    if (angle_done || timeout_done) {
      status_ = Status::DONE;
      spdlog::info("SPIN done: swept={:.2f}, elapsed={}ms", swept, elapsed);
    }
  }

  if (status_ == Status::DONE) {
    robot.vel = {0, 0};
    robot.rotation = 0;
    robot.dribbling = 0;
    return true;
  }

  return false;
}
