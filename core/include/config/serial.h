#pragma once

#include <rapidjson/fwd.h>

#include <memory>
#include <string>

namespace cfg {

struct Serial {
  Serial(const rapidjson::Value&);
  ~Serial();
  Serial(const Serial&) = delete;
  Serial& operator=(const Serial&) = delete;

  bool enabled;
  std::string device;
  int rate;
  bool interference;
  struct Emitter {
    double threshold;
    int optimist;
  };
  std::unique_ptr<Emitter> emitter;

  struct Battery {
    float low_threshold;
    int warning_interval_ms;
    int pause_warning_interval_ms;
  };
  std::unique_ptr<Battery> battery;
};

}  // namespace cfg
