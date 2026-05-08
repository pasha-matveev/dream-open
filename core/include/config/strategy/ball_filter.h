#pragma once

#include <rapidjson/fwd.h>

namespace cfg {

struct BallFilter {
  BallFilter(const rapidjson::Value& doc);
  ~BallFilter();
  BallFilter(const BallFilter&) = delete;
  BallFilter& operator=(const BallFilter&) = delete;

  bool enabled;
  double alpha_xy;
  double beta_xy;
  double friction_tau;
  double latency_ms;
  double max_jump;
  double lost_timeout_ms;
};

}  // namespace cfg
