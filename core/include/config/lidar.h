#pragma once

#include <rapidjson/fwd.h>

#include <memory>
#include <string>

namespace cfg {

struct Lidar {
  Lidar(const rapidjson::Value&);
  ~Lidar();
  Lidar(const Lidar&) = delete;
  Lidar& operator=(const Lidar&) = delete;

  bool enabled;
  std::string path;

  struct Calibration {
    bool enabled;
    int delay;
    int threshold;
    double movement;
    double angle;
  };
  std::unique_ptr<Calibration> calibration;
};

}  // namespace cfg
