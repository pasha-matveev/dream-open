#pragma once

#include <rapidjson/fwd.h>

struct BrakeProfile {
  double wall_limit;
  double decel_k;

  BrakeProfile() = default;
  BrakeProfile(const rapidjson::Value& value);
};