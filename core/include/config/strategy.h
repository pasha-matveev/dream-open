#pragma once

#include <rapidjson/fwd.h>

#include <memory>
#include <string>

struct Mapper;

namespace cfg {

struct Control;
struct BallFilter;
struct Attacker;
struct Keeper;
struct Kickoff;
struct Dubins;

struct Strategy {
  Strategy(const rapidjson::Value&);
  ~Strategy();
  Strategy(const Strategy&) = delete;
  Strategy& operator=(const Strategy&) = delete;

  bool enabled;
  std::string role;
  double turn_precision;

  struct Predict {
    bool enabled;
    double alpha_xy;
  };
  std::unique_ptr<Predict> predict;

  std::unique_ptr<BallFilter> ball_filter;

  struct Motion {
    double max_linear_accel;
    double max_angular_accel;
    double decel_k;
    double wall_limit;
  };
  std::unique_ptr<Motion> motion;

  std::unique_ptr<Mapper> dribbling;
  double dribbling_slow;

  std::unique_ptr<Attacker> attacker;
  std::unique_ptr<Keeper> keeper;
  std::unique_ptr<Kickoff> kickoff;
  std::unique_ptr<Dubins> dubins;
  std::unique_ptr<Control> control;
};

}  // namespace cfg
