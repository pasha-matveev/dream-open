#pragma once

#include <rapidjson/fwd.h>

#include <memory>

struct Mapper;
class Switch;

namespace cfg {

struct Dubins {
  Dubins(const rapidjson::Value& doc);
  ~Dubins();
  Dubins(const Dubins&) = delete;
  Dubins& operator=(const Dubins&) = delete;

  double bonus;
  double separate;
  double radius;
  double deep_inside;
  double camera_target_dist;
  double aim_bonus;
  std::unique_ptr<Switch> kick_precision;
  std::unique_ptr<Mapper> speed;
};

}  // namespace cfg
