#pragma once

#include <rapidjson/fwd.h>

#include <memory>

namespace cfg {

struct Keeper {
  Keeper(const rapidjson::Value& doc);
  ~Keeper();
  Keeper(const Keeper&) = delete;
  Keeper& operator=(const Keeper&) = delete;

  double global_border;
  double dubins_border;
  bool ram_enabled;

  struct Line {
    double padding;
    double y;
  };
  std::unique_ptr<Line> line;
};

}  // namespace cfg
