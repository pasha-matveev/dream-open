#include "config/strategy/ball_filter.h"

using namespace cfg;

BallFilter::~BallFilter() = default;

BallFilter::BallFilter(const rapidjson::Value& doc) {
  enabled = doc["enabled"].GetBool();
  alpha_xy = doc["alpha_xy"].GetDouble();
  beta_xy = doc["beta_xy"].GetDouble();
  friction_tau = doc["friction_tau"].GetDouble();
  latency_ms = doc["latency_ms"].GetDouble();
  max_jump = doc["max_jump"].GetDouble();
  lost_timeout_ms = doc["lost_timeout_ms"].GetDouble();
}
