#pragma once

#include <rapidjson/fwd.h>

#include <string>

namespace cfg {

struct Meme {
  Meme(const rapidjson::Value&);
  Meme(const Meme&) = delete;
  Meme& operator=(const Meme&) = delete;

  std::string composition;
  double radius;
  double linear_speed;
  double spin_speed;
};

}  // namespace cfg
