#pragma once

#include <rapidjson/fwd.h>

#include <string>

namespace cfg {

struct Visualization {
  Visualization(const rapidjson::Value&);
  ~Visualization();
  Visualization(const Visualization&) = delete;
  Visualization& operator=(const Visualization&) = delete;

  bool enabled;
  std::string window_name;
  int frames;
  bool interactive;
};

}  // namespace cfg
