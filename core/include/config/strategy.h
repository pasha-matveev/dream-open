#pragma once

#include <rapidjson/fwd.h>

#include <memory>
#include <string>

struct Mapper;
class Switch;

namespace cfg {

struct Control;

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

  struct BallFilter {
    bool enabled;
    double alpha_xy;
    double beta_xy;
    double friction_tau;
    double latency_ms;
    double max_jump;
    double lost_timeout_ms;
  };
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

  struct Attacker {
    double border;
    bool dubins_enabled;
    struct Ff {
      bool enabled;
      double v_min;
      long long stale_ms;
      double gain;
    };
    std::unique_ptr<Ff> ff;
  };
  std::unique_ptr<Attacker> attacker;

  struct Keeper {
    double global_border;
    double dubins_border;
    bool ram_enabled;
    struct Line {
      double padding;
      double y;
    };
    std::unique_ptr<Line> line;
  };
  std::unique_ptr<Keeper> keeper;

  struct Dubins {
    double bonus;
    double separate;
    double radius;
    double deep_inside;
    double camera_target_dist;
    std::unique_ptr<Switch> kick_precision;
    double aim_bonus;
    std::unique_ptr<Mapper> speed;
  };
  std::unique_ptr<Dubins> dubins;

  std::unique_ptr<Control> control;

  struct AttackerRicochet {
    bool enabled;
  };
  std::unique_ptr<AttackerRicochet> attacker_ricochet;
};

}  // namespace cfg
