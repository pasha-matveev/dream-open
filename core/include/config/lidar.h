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
    int window;
    int threshold;
    double movement;
    double angle;
    // Сколько мс не видим свои ворота в PAUSE, прежде чем считать,
    // что робот направлен на ворота противника (field_angle=0).
    int own_goal_timeout;
  };
  std::unique_ptr<Calibration> calibration;
};

}  // namespace cfg
