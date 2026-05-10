#pragma once

#include <rapidjson/fwd.h>

#include <memory>

#include "utils/mapper.h"

namespace cfg {

struct Control {
  Control(const rapidjson::Value& doc);
  ~Control();
  Control(const Control&) = delete;
  Control& operator=(const Control&) = delete;

  std::unique_ptr<Mapper> speed;
  std::unique_ptr<Mapper> kick_power;

  struct CurveTurn {
    int accel_time;
    double rotation_limit;
    double vel;
    double dist;
  };
  std::unique_ptr<CurveTurn> curved_turn;

  struct SimpleTurn {
    int accel_time;
    double rotation_limit;
  };
  std::unique_ptr<SimpleTurn> simple_turn;

  struct BallDrive {
    // Ограничение ускорения при движении с захваченным мячом
    double max_accel;
  };
  std::unique_ptr<BallDrive> ball_drive;
};

}  // namespace cfg