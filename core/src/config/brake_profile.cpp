#include "config/brake_profile.h"

#include <rapidjson/rapidjson.h>

#include <cassert>

BrakeProfile::BrakeProfile(const rapidjson::Value& value) {
  wall_limit = value["wall_limit"].GetDouble();
  decel_k = value["decel_k"].GetDouble();
  assert(decel_k <= 1.0);
}