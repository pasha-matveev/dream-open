#pragma once

#include <rapidjson/fwd.h>

struct Mapper {
  double param_l, param_r;
  double value_l, value_r;
  Mapper() = default;
  Mapper(const rapidjson::Value& value);
  double map(double val) const;
};