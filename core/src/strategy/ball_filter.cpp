
#include "strategy/ball_filter.h"

#include <algorithm>
#include <cmath>

#include "utils/config.h"

void BallFilter::predict(double dt) {
  if (!initialized) return;
  pos += vel * dt;
  const double tau = config.strategy.ball_filter.friction_tau;
  if (tau > 0) {
    vel *= std::exp(-dt / tau);
  }
}

void BallFilter::update(const Vec& z, long long now_ms) {
  const auto& cfg = config.strategy.ball_filter;

  if (!initialized) {
    pos = z;
    vel = {0, 0};
    last_update_ms = now_ms;
    initialized = true;
    return;
  }

  Vec residual = z - pos;
  if (residual.len() > cfg.max_jump) {
    pos = z;
    vel = {0, 0};
    last_update_ms = now_ms;
    return;
  }

  pos += residual * cfg.alpha_xy;

  if (last_update_ms >= 0) {
    double meas_dt = std::clamp((now_ms - last_update_ms) / 1000.0, 0.0, 0.1);
    if (meas_dt > 0) {
      vel += residual * (cfg.beta_xy / meas_dt);
    }
  }
  last_update_ms = now_ms;
}

void BallFilter::lost(long long now_ms) {
  if (!initialized) return;
  if (last_update_ms >= 0 &&
      now_ms - last_update_ms > config.strategy.ball_filter.lost_timeout_ms) {
    initialized = false;
    vel = {0, 0};
  }
}

Vec BallFilter::position() const {
  return pos + vel * (config.strategy.ball_filter.latency_ms / 1000.0);
}
