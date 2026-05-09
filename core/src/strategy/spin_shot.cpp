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
    // sweep_angle > DONE_EPS, чтобы spin не завершился на первом же тике из-за
    // тонкой проверки swept >= sweep - DONE_EPS. DONE_EPS=0.05 ниже.
    assert(params.sweep_angle > 0.05 && params.sweep_angle <= M_PI &&
           "SpinShot: sweep_angle must be in (DONE_EPS, pi]");
    assert((params.direction == 1 || params.direction == -1) &&
           "SpinShot: direction must be +1 or -1");
    start_ms_ = millis();
    start_field_angle_ = robot.field_angle;
    target_field_angle_ = normalize_angle(start_field_angle_ +
                                          params.direction * params.sweep_angle);
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

    if (params.kicker_ms > 0 && !kicker_fired_ && elapsed >= params.kicker_ms) {
      robot.kicker_force = params.kicker_force
                               ? params.kicker_force
                               : (int)KickController::compute_power(robot.position);
      kicker_fired_ = true;
      spdlog::info("SPIN kicker fire: power={}", robot.kicker_force);
    }

    double swept = std::abs(
        normalize_angle(robot.field_angle - start_field_angle_));
    constexpr double DONE_EPS = 0.05;
    bool angle_done = swept >= params.sweep_angle - DONE_EPS;
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
